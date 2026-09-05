#include "osc.h"

#include <errno.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "bno.h"
#include "osc_config.h"
#include "led_output.h"
#include "battery.h"
#include "zinc_time.h"
#include "wifi_manager.h"
#include "wifi_web_config.h"
#include "esp_wifi.h"
#include "state_machine.h"

static const char *TAG = "osc";

static int osc_sock = -1;
static uint32_t target_ip = 0; /* network byte order; 0 = none registered yet */
static uint16_t target_port = OSC_DEFAULT_SEND_PORT;
static osc_rx_cb_t rx_handler = NULL;
static uint32_t bundle_rate_count = 0;
static int64_t bundle_rate_started_us = 0;
static uint32_t last_status_stream_ms = 0;
static uint32_t last_imu_stream_ms = 0;
static bool osc_task_initialized = false; // reset on state change
static uint8_t send_failure_count = 0;
#define OSC_MAX_CONSECUTIVE_SEND_FAILURES 5

/* OSC strings (address pattern, type tag) are null-terminated and then
 * zero-padded so the total length is a multiple of 4 bytes. */
static uint16_t osc_padded_len(uint16_t len_with_nul) {
  return (uint16_t)((len_with_nul + 3U) & ~3U);
}

static uint16_t osc_write_string(uint8_t *buf, const char *s) {
  const uint16_t len = (uint16_t)(strlen(s) + 1U); /* + nul terminator */
  const uint16_t padded = osc_padded_len(len);
  memset(buf, 0, padded);
  memcpy(buf, s, len);
  return padded;
}

static uint16_t osc_write_bits32(uint8_t *buf, uint32_t bits) {
  buf[0] = (uint8_t)(bits >> 24);
  buf[1] = (uint8_t)(bits >> 16);
  buf[2] = (uint8_t)(bits >> 8);
  buf[3] = (uint8_t)(bits);
  return 4U;
}

static uint16_t osc_write_float32(uint8_t *buf, float value) {
  uint32_t bits;
  memcpy(&bits, &value, sizeof(bits));
  return osc_write_bits32(buf, bits);
}

static uint16_t osc_write_int32(uint8_t *buf, int32_t value) {
  return osc_write_bits32(buf, (uint32_t)value);
}

static bool osc_sendto_target(const uint8_t *packet, uint16_t len) {
  if (osc_sock < 0 || target_ip == 0) return false;

  struct sockaddr_in dest = {0};
  dest.sin_family = AF_INET;
  dest.sin_port = htons(target_port);
  dest.sin_addr.s_addr = target_ip;

  for (int attempt = 0; attempt < 3; attempt++) {
    int sent = sendto(osc_sock, packet, len, MSG_DONTWAIT, (const struct sockaddr *)&dest, sizeof(dest));
    if (sent == len) {
      send_failure_count = 0;
      return true;
    }

    if (sent < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == ENOBUFS || errno == ENOMEM)) {
      vTaskDelay(pdMS_TO_TICKS(2));
      continue;
    }

    if (sent < 0) {
      ESP_LOGW(TAG, "sendto failed: errno=%d", errno);
    }
    return false;
  }

  ESP_LOGW(TAG, "sendto dropped after retries (errno=%d)", errno);

  /* A target that never ARP-resolves (e.g. the registered client went away
   * without deregistering) causes every send attempt to burn through lwIP's
   * fixed-size pbuf pool waiting on ARP, which can starve *inbound* packet
   * processing too (pbufs are a shared resource) - including any new
   * /glowstick/register message that would fix this by pointing target_ip
   * somewhere reachable. Give up on a target after repeated consecutive
   * failures so the queue drains and new registrations can get through. The
   * plain sendto() above (rather than connect()+send()) also means the
   * socket is never "locked" to one peer, so it keeps accepting inbound
   * register packets from anyone regardless of this state. */
  if (++send_failure_count >= OSC_MAX_CONSECUTIVE_SEND_FAILURES) {
    struct in_addr addr = {.s_addr = target_ip};
    ESP_LOGW(TAG, "giving up on unreachable OSC target %s:%u after %u consecutive failures - "
                  "awaiting re-registration",
             inet_ntoa(addr), target_port, (unsigned)send_failure_count);
    target_ip = 0;
    send_failure_count = 0;
  }
  return false;
}

void osc_send_floats(const char *address, const float *values, uint8_t count) {
  if (count > 8) count = 8;
  uint8_t buf[80];
  char tags[10] = ",";
  for (uint8_t i = 0; i < count; i++) tags[1 + i] = 'f';
  tags[1 + count] = '\0';

  uint16_t offset = osc_write_string(buf, address);
  offset += osc_write_string(buf + offset, tags);
  for (uint8_t i = 0; i < count; i++) offset += osc_write_float32(buf + offset, values[i]);
  osc_sendto_target(buf, offset);
}

void osc_send_ints(const char *address, const int32_t *values, uint8_t count) {
  if (count > 8) count = 8;
  uint8_t buf[80];
  char tags[10] = ",";
  for (uint8_t i = 0; i < count; i++) tags[1 + i] = 'i';
  tags[1 + count] = '\0';

  uint16_t offset = osc_write_string(buf, address);
  offset += osc_write_string(buf + offset, tags);
  for (uint8_t i = 0; i < count; i++) offset += osc_write_int32(buf + offset, values[i]);
  osc_sendto_target(buf, offset);
}

void osc_send_float1(const char *address, float value) { osc_send_floats(address, &value, 1); }

void osc_send_float3(const char *address, float x, float y, float z) {
  const float values[3] = {x, y, z};
  osc_send_floats(address, values, 3);
}

// AI: below section was generated by an AI
static bool osc_bundle_add(osc_bundle_t *bundle, const char *address, const void *values, uint8_t count,
                           char type) {
  if (bundle == NULL || address == NULL || (values == NULL && count != 0) || count > 8) return false;

  const size_t address_length = strnlen(address, OSC_BUNDLE_CAPACITY);
  if (address_length == OSC_BUNDLE_CAPACITY) return false;

  const uint16_t padded_address_length = osc_padded_len((uint16_t)address_length + 1U);
  const uint16_t padded_tag_length = osc_padded_len((uint16_t)count + 2U);
  const uint16_t message_length = padded_address_length + padded_tag_length + (uint16_t)count * 4U;
  if ((uint32_t)bundle->length + 4U + message_length > OSC_BUNDLE_CAPACITY) return false;

  uint8_t *element = bundle->data + bundle->length;
  osc_write_bits32(element, message_length);
  uint16_t offset = 4U;
  offset += osc_write_string(element + offset, address);

  char tags[10] = ",";
  memset(tags + 1, type, count);
  tags[count + 1U] = '\0';
  offset += osc_write_string(element + offset, tags);

  for (uint8_t index = 0; index < count; index++) {
    if (type == 'f') {
      offset += osc_write_float32(element + offset, ((const float *)values)[index]);
    } else {
      offset += osc_write_int32(element + offset, ((const int32_t *)values)[index]);
    }
  }

  bundle->length += offset;
  return true;
}

void osc_bundle_init(osc_bundle_t *bundle) {
  if (bundle == NULL) return;
  static const uint8_t immediate_bundle_header[16] = {
      '#', 'b', 'u', 'n', 'd', 'l', 'e', '\0', 0, 0, 0, 0, 0, 0, 0, 1,
  };
  memcpy(bundle->data, immediate_bundle_header, sizeof(immediate_bundle_header));
  bundle->length = sizeof(immediate_bundle_header);
}

bool osc_bundle_add_floats(osc_bundle_t *bundle, const char *address, const float *values, uint8_t count) {
  return osc_bundle_add(bundle, address, values, count, 'f');
}

bool osc_bundle_add_ints(osc_bundle_t *bundle, const char *address, const int32_t *values, uint8_t count) {
  return osc_bundle_add(bundle, address, values, count, 'i');
}

void osc_bundle_send(const osc_bundle_t *bundle) {
  if (bundle == NULL || bundle->length <= 16U || bundle->length > OSC_BUNDLE_CAPACITY) return;
  if (!osc_sendto_target(bundle->data, bundle->length)) return;

  const int64_t now_us = esp_timer_get_time();
  if (bundle_rate_started_us == 0) bundle_rate_started_us = now_us;
  bundle_rate_count++;
  if (bundle_rate_count >= 500U) {
    const int64_t elapsed_us = now_us - bundle_rate_started_us;
    const uint32_t rate_tenths_hz = (uint32_t)(((uint64_t)bundle_rate_count * 10000000ULL) /
                                               (uint64_t)elapsed_us);
    ESP_LOGI(TAG, "OSC bundle TX rate: %u.%u Hz", rate_tenths_hz / 10U, rate_tenths_hz % 10U);
    bundle_rate_count = 0;
    bundle_rate_started_us = now_us;
  }
}
// AI: end

void osc_set_target(uint32_t ip, uint16_t port) {
  if (!wifi_manager_is_same_subnet(ip)) {
    struct in_addr addr = {.s_addr = ip};
    ESP_LOGW(TAG, "ignoring OSC target %s:%u (device ip=%s on different subnet)",
             inet_ntoa(addr), port, wifi_manager_get_ip_str());
    target_ip = 0;
    target_port = port;
    return;
  }

  target_ip = ip;
  target_port = port;
  send_failure_count = 0;

  struct in_addr addr = {.s_addr = ip};
  ESP_LOGI(TAG, "OSC target set to %s:%u", inet_ntoa(addr), port);
}

bool osc_has_target(void) { return target_ip != 0; }

/* Parses one incoming OSC message: address pattern, type tag string, then
 * 'i' (int32) / 'f' (float32) arguments in wire order. Any other type tag
 * character stops parsing further arguments (rare in practice for a simple
 * control device like this), since we have no use for blobs/strings/etc.
 * here. */
void osc_parse_and_dispatch(const uint8_t *buf, uint16_t len, uint32_t sender_ip, uint16_t sender_port) {
  if (len < 4 || buf[0] != '/') return; /* not an OSC message (bundles start with '#') */

  const char *address = (const char *)buf;
  const size_t address_length = strnlen(address, len);
  if (address_length == len) return;
  uint16_t offset = osc_padded_len((uint16_t)address_length + 1U);
  if (offset >= len) return;

  if (buf[offset] != ',') return; /* no type tag string, e.g. no-argument message form we don't emit */
  const char *typetags = (const char *)&buf[offset];
  const size_t typetag_available = (size_t)(len - offset);
  const size_t typetag_length = strnlen(typetags, typetag_available);
  if (typetag_length == typetag_available) return;
  offset += osc_padded_len((uint16_t)typetag_length + 1U);
  if (offset > len) return;

  int32_t int_args[4];
  float float_args[4];
  uint8_t int_count = 0, float_count = 0;

  for (const char *tag = typetags + 1; *tag != '\0'; tag++) {
    if (offset + 4U > len) break;
    uint32_t bits = ((uint32_t)buf[offset] << 24) | ((uint32_t)buf[offset + 1] << 16) |
                    ((uint32_t)buf[offset + 2] << 8) | (uint32_t)buf[offset + 3];
    offset += 4U;

    if (*tag == 'f' && float_count < 4) {
      memcpy(&float_args[float_count++], &bits, sizeof(bits));
    } else if (*tag == 'i' && int_count < 4) {
      int_args[int_count++] = (int32_t)bits;
    }
  }

  if (rx_handler) rx_handler(address, int_args, int_count, float_args, float_count, sender_ip, sender_port);
}

void osc_tx_task(void *arg)
{
  (void)arg;
  uint8_t buf[128];
  bool was_osc_mode = false;
  for (;;)
  {
    if (device_state == DEVICE_STATE_OSC_SWING_MODE)
    {
      osc_init();
      was_osc_mode = true;
      const uint32_t now = now_ms();
      // Implement transmission logic here if needed
      // stream_imu_over_osc(now, linear_acceleration_ms2); // send all instead
      stream_status_over_osc(now);
      stream_imu_over_osc(now);

      struct sockaddr_in source = {0};
      socklen_t source_len = sizeof(source);
      int received = recvfrom(osc_sock, buf, sizeof(buf) - 1, MSG_DONTWAIT,
                              (struct sockaddr *)&source, &source_len);
      if (received >= 0)
      {
        osc_parse_and_dispatch(buf, (uint16_t)received, source.sin_addr.s_addr, ntohs(source.sin_port));
      }
      else if (errno != EAGAIN && errno != EWOULDBLOCK)
      {
        ESP_LOGW(TAG, "recvfrom failed: errno=%d", errno);
      }
    }
    else if (was_osc_mode)
    {
      osc_task_initialized = false;
      osc_off();
      was_osc_mode = false;
    }
    /* Only stream_status_over_osc/stream_imu_over_osc's own elapsed-time
     * checks should decide the actual send rate (down to stream_period_ms,
     * as low as 10ms / 100Hz - see MIN_STREAM_PERIOD_MS in osc_config.c).
     * A slow fixed delay here was silently capping every stream to at most
     * 1000/delay Hz regardless of that config, e.g. 10Hz at a 100ms delay.
     * IMPORTANT: CONFIG_FREERTOS_HZ=100 here means one tick = 10ms, so
     * pdMS_TO_TICKS() truncates anything below that to 0 ticks -
     * vTaskDelay(0) does not actually sleep, so a smaller value here just
     * busy-spins this (priority 5) task and starves the IDLE task, tripping
     * the task watchdog. 10ms is the fastest delay this tick rate can
     * express, which conveniently matches the 100Hz floor above. Fall back
     * to a slow idle poll outside OSC mode so this task isn't busy-looping
     * for no reason. */
    vTaskDelay(pdMS_TO_TICKS(device_state == DEVICE_STATE_OSC_SWING_MODE ? 10 : 100));
  }
}

void osc_set_rx_handler(osc_rx_cb_t cb) { rx_handler = cb; }

void osc_init(void)
{
  if (!osc_task_initialized)
  {
    wifi_manager_init();
    osc_config_init();
    if (wifi_manager_is_ap_mode())
    {
      wifi_web_config_start();
    }

    osc_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (osc_sock < 0)
    {
      ESP_LOGE(TAG, "failed to create UDP socket: errno=%d", errno);
      return;
    }

    struct sockaddr_in bind_addr = {0};
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    bind_addr.sin_port = htons(OSC_LISTEN_PORT);

    if (bind(osc_sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) != 0)
    {
      ESP_LOGE(TAG, "failed to bind UDP socket to port %d: errno=%d", OSC_LISTEN_PORT, errno);
      close(osc_sock);
      osc_sock = -1;
      return;
    }

    ESP_LOGI(TAG, "listening for OSC on UDP port %d", OSC_LISTEN_PORT);
    osc_task_initialized = true;
  }
}
void osc_task_init(void) {
  xTaskCreate(osc_tx_task, "osc", 4096, NULL, 5, NULL);
}
/* Streams raw linear acceleration plus the rest of the BNO055 pose data
 * (orientation quaternion, gyro, magnetometer, gravity vector, and sensor
 * calibration status) as OSC messages, gated by the live
 * (OSC-configurable) enable flag and rate limit - see osc_config.h for
 * the full list of addresses/argument layouts. */
void stream_imu_over_osc(uint32_t now) {
  const osc_config_t *cfg = osc_config_get();
  if (!cfg->stream_enabled) return;
  if ((now - last_imu_stream_ms) < cfg->stream_period_ms) return;
  last_imu_stream_ms = now;
  if (!osc_has_target()) return;

  osc_bundle_t bundle;
  osc_bundle_init(&bundle);
  if (!osc_bundle_add_floats(&bundle, "/glowstick/accel", (const float[]){(float)last_bno_teleplot.linacc[0] / 100.0f,
                                                                 (float)last_bno_teleplot.linacc[1] / 100.0f,
                                                                 (float)last_bno_teleplot.linacc[2] / 100.0f}, 3)) return;

  const float quat[4] = {
      (float)last_bno_teleplot.quat[0] / 16384.0f,
      (float)last_bno_teleplot.quat[1] / 16384.0f,
      (float)last_bno_teleplot.quat[2] / 16384.0f,
      (float)last_bno_teleplot.quat[3] / 16384.0f,
  };
  if (!osc_bundle_add_floats(&bundle, "/glowstick/orientation", quat, 4)) return;

  const float gyro[3] = {
      (float)last_bno_teleplot.gyro[0] / 16.0f,
      (float)last_bno_teleplot.gyro[1] / 16.0f,
      (float)last_bno_teleplot.gyro[2] / 16.0f,
  };
  if (!osc_bundle_add_floats(&bundle, "/glowstick/gyro", gyro, 3)) return;

  const float mag[3] = {
      (float)last_bno_teleplot.mag[0] / 16.0f,
      (float)last_bno_teleplot.mag[1] / 16.0f,
      (float)last_bno_teleplot.mag[2] / 16.0f,
  };
  if (!osc_bundle_add_floats(&bundle, "/glowstick/mag", mag, 3)) return;

  const float gravity[3] = {
      (float)last_bno_teleplot.gravity[0] / 100.0f,
      (float)last_bno_teleplot.gravity[1] / 100.0f,
      (float)last_bno_teleplot.gravity[2] / 100.0f,
  };
  if (!osc_bundle_add_floats(&bundle, "/glowstick/gravity", gravity, 3)) return;

  const int32_t calib[4] = {
      (last_bno_teleplot.calib >> 6) & 0x03, /* system */
      (last_bno_teleplot.calib >> 4) & 0x03, /* gyro */
      (last_bno_teleplot.calib >> 2) & 0x03, /* accel */
      last_bno_teleplot.calib & 0x03,        /* mag */
  };
  if (!osc_bundle_add_ints(&bundle, "/glowstick/calib", calib, 4)) return;

	const int32_t ndof[3] = {
			last_bno_teleplot.op_mode,
			last_bno_teleplot.sys_status,
			last_bno_teleplot.sys_error,
	};
	if (!osc_bundle_add_ints(&bundle, "/glowstick/ndof", ndof, 3)) return;
	osc_bundle_send(&bundle);
}

/* Streams device housekeeping data (battery, BNO055 die temperature,
 * Wi-Fi RSSI) at a fixed slow rate, independent of the IMU stream
 * enable/period settings - this is cheap and useful to have on hand for
 * any receiver UI even without a full accel stream running. */
void stream_status_over_osc(uint32_t now) {
  if ((now - last_status_stream_ms) < STATUS_STREAM_PERIOD_MS) return;
  last_status_stream_ms = now;
  if (!osc_has_target()) return;

  osc_send_floats("/glowstick/battery", (const float[]){battery_get_voltage(), battery_get_percent()}, 2);
  osc_send_float1("/glowstick/temp", (float)last_bno_teleplot.temp_c);
  const int32_t rssi = wifi_manager_get_rssi();
  osc_send_ints("/glowstick/rssi", &rssi, 1);
  /* Power-on self-test result (set once at boot, resent here so it's
   * visible without a serial connection): bit0=MCU bit1=gyro bit2=accel
   * bit3=mag, 1=pass. If bit3 (mag) is 0, the board's magnetometer
   * hardware failed self-test and /glowstick/mag will always read zero
   * regardless of firmware - a common issue on some BNO055 clone modules. */
  const int32_t selftest[4] = {
      last_bno_teleplot.selftest & 0x01,
      (last_bno_teleplot.selftest >> 1) & 0x01,
      (last_bno_teleplot.selftest >> 2) & 0x01,
      (last_bno_teleplot.selftest >> 3) & 0x01,
  };
  osc_send_ints("/glowstick/selftest", selftest, 4);

	const float mag_probe[3] = {
			(float)last_bno_teleplot.mag_probe[0] / 16.0f,
			(float)last_bno_teleplot.mag_probe[1] / 16.0f,
			(float)last_bno_teleplot.mag_probe[2] / 16.0f,
	};
	osc_send_floats("/glowstick/magprobe", mag_probe, 3);
}

// Turn off OSC streaming and all radio modules
void osc_off(void){
  if (osc_sock >= 0) {
    close(osc_sock);
    osc_sock = -1;
  }
  wifi_web_config_stop();
  wifi_manager_stop();
  target_ip = 0;
}