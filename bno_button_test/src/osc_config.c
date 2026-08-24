#include "osc_config.h"

#include <string.h>

#include "esp_log.h"
#include "nvs.h"

#include "osc.h"

static const char *TAG = "osc_config";
static const char *NVS_NAMESPACE = "osccfg";
static const char *NVS_KEY = "cfg";
static const int32_t MIN_STREAM_PERIOD_MS = 10;
static const int32_t MAX_STREAM_PERIOD_MS = 10000;

static const osc_config_t default_config = {
    .target_ip = 0,
    .target_port = OSC_DEFAULT_SEND_PORT,
    .stream_enabled = false,
  .stream_period_ms = MIN_STREAM_PERIOD_MS,
};

static osc_config_t config;

static void persist_config(void) {
  nvs_handle_t handle;
  esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "nvs_open (write) failed: %d", err);
    return;
  }
  err = nvs_set_blob(handle, NVS_KEY, &config, sizeof(config));
  if (err == ESP_OK) err = nvs_commit(handle);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "persisting config failed: %d", err);
  }
  nvs_close(handle);
}

static void load_config(void) {
  config = default_config;

  nvs_handle_t handle;
  esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
  if (err != ESP_OK) {
    ESP_LOGI(TAG, "no stored config yet, using defaults");
    return;
  }

  size_t required_size = sizeof(config);
  osc_config_t stored;
  err = nvs_get_blob(handle, NVS_KEY, &stored, &required_size);
  nvs_close(handle);

  if (err == ESP_OK && required_size == sizeof(stored)) {
    config = stored;
    ESP_LOGI(TAG, "loaded config: port=%u stream=%d/%ums", config.target_port, config.stream_enabled,
             config.stream_period_ms);
  } else {
    ESP_LOGW(TAG, "stored config unreadable (err=%d), using defaults", err);
  }
}

/* Handles every inbound OSC message: recognized /glowstick/... config
 * addresses update+persist the config (and re-register the transport
 * target where relevant); anything else is ignored by this layer. */
static void handle_osc_rx(const char *address, const int32_t *int_args, uint8_t int_arg_count,
                           const float *float_args, uint8_t float_arg_count, uint32_t sender_ip,
                           uint16_t sender_port) {
  (void)float_args;
  (void)float_arg_count;
  (void)sender_port;

  bool changed = false;

  if (strcmp(address, "/glowstick/register") == 0) {
    changed = config.target_ip != sender_ip;
    config.target_ip = sender_ip;
    osc_set_target(config.target_ip, config.target_port);
  } else if (strcmp(address, "/glowstick/config/port") == 0 && int_arg_count >= 1 &&
             int_args[0] >= 1 && int_args[0] <= UINT16_MAX) {
    const uint16_t port = (uint16_t)int_args[0];
    changed = config.target_port != port;
    config.target_port = port;
    if (config.target_ip != 0) osc_set_target(config.target_ip, config.target_port);
  } else if (strcmp(address, "/glowstick/config/stream") == 0 && int_arg_count >= 1 &&
             (int_args[0] == 0 || int_args[0] == 1)) {
    const bool enabled = int_args[0] == 1;
    changed = config.stream_enabled != enabled;
    config.stream_enabled = enabled;
  } else if (strcmp(address, "/glowstick/config/streamperiod") == 0 && int_arg_count >= 1 &&
             int_args[0] >= MIN_STREAM_PERIOD_MS && int_args[0] <= MAX_STREAM_PERIOD_MS) {
    const uint16_t period = (uint16_t)int_args[0];
    changed = config.stream_period_ms != period;
    config.stream_period_ms = period;
  }

  if (changed) {
    ESP_LOGI(TAG, "config updated via %s", address);
    persist_config();
  }
}

void osc_config_init(void) {
  load_config();
  if (config.target_ip != 0) {
    osc_set_target(config.target_ip, config.target_port);
  }
  osc_set_rx_handler(handle_osc_rx);
}

const osc_config_t *osc_config_get(void) { return &config; }
