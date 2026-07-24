#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_SDA_GPIO 22
#define I2C_SCL_GPIO 23
#define I2C_FREQ_HZ          400000 // 100kHz Standard Mode

#define BUTTON_GPIO GPIO_NUM_0

#define BATTERY_GPIO GPIO_NUM_1
#define BATTERY_DIVIDER 2.0f
#define BATTERY_VMIN 3.40f
#define BATTERY_VMAX 4.20f

#define LED_GPIO GPIO_NUM_18
#define LED_COUNT 9

#define BNO_ADDR_PRIMARY 0x29
#define BNO_ADDR_SECONDARY 0x28

#define BNO_REG_CHIP_ID 0x00
#define BNO_REG_OPR_MODE 0x3D
#define BNO_REG_PAGE_ID 0x07
#define BNO_REG_CALIB_STAT 0x35
#define BNO_REG_LINACC_DATA 0x28
#define BNO_REG_GYRO_DATA 0x14
#define BNO_REG_QUATERNION_DATA 0x20

#define BNO_MODE_CONFIG 0x00
#define BNO_MODE_NDOF 0x0C
#define BNO_SAMPLE_PERIOD_MS 10
#define BUTTON_LONG_PRESS_MS 2000

typedef struct {
  uint8_t r;
  uint8_t g;
  uint8_t b;
  const char *name;
} led_color_t;

static const char *TAG = "bno_button_test";

static const led_color_t solid_colors[] = {
    {255, 0, 0, "RED"},
    {0, 255, 0, "GREEN"},
    {0, 0, 255, "BLUE"},
    {255, 255, 255, "WHITE"},
    {255, 180, 0, "AMBER"},
    {255, 0, 255, "MAGENTA"},
    {0, 255, 255, "CYAN"},
};

static bool bno_ready = false;
static uint8_t bno_addr = BNO_ADDR_PRIMARY;
static i2c_master_bus_handle_t i2c_bus = NULL;
static i2c_master_dev_handle_t bno_dev = NULL;

static int last_raw_button = 1;
static int stable_button = 1;
static uint32_t last_button_change_ms = 0;
static uint32_t button_press_start_ms = 0;
static bool button_long_press_fired = false;

static adc_oneshot_unit_handle_t adc_unit = NULL;
static adc_channel_t battery_channel = ADC_CHANNEL_0;
static bool battery_ready = false;
static adc_cali_handle_t adc_cali = NULL;
static bool adc_cali_enabled = false;

static rmt_channel_handle_t led_chan = NULL;
static rmt_encoder_handle_t led_encoder = NULL;
static bool led_ready = false;
static uint8_t led_mode = 0;
static uint8_t rainbow_hue = 0;
static uint8_t led_payload[LED_COUNT * 3] = {0};

static uint32_t last_bno_ms = 0;
static uint32_t last_battery_ms = 0;
static uint32_t last_led_anim_ms = 0;

static const rmt_transmit_config_t led_tx_config = {
    .loop_count = 0,
    .flags = {
        .eot_level = 0,
        .queue_nonblocking = 0,
    },
};

static uint32_t now_ms(void) {
  return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

static float clampf(float value, float lo, float hi) {
  if (value < lo) return lo;
  if (value > hi) return hi;
  return value;
}

static esp_err_t i2c_write_reg(uint8_t reg, uint8_t value) {
  if (bno_dev == NULL) return ESP_ERR_INVALID_STATE;
  uint8_t data[2] = {reg, value};
  return i2c_master_transmit(bno_dev, data, sizeof(data), 100);
}

static esp_err_t i2c_read_reg(uint8_t reg, uint8_t *buf, size_t len) {
  if (bno_dev == NULL) return ESP_ERR_INVALID_STATE;
  return i2c_master_transmit_receive(bno_dev, &reg, 1, buf, len, 100);
}


static int16_t read_i16_le(const uint8_t *buf) {
  return (int16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
}

static void teleplot_emit_float(const char *name, float value) {
  // Teleplot serial parser expects lines prefixed with '>'
  printf(">%s:%.5f\n", name, value);
}

static void teleplot_emit_uint(const char *name, uint32_t value) {
  printf(">%s:%u\n", name, (unsigned int)value);
}

static void emit_bno_teleplot(const uint8_t *linacc, const uint8_t *gyro, const uint8_t *quat, uint8_t calib) {
  const float lin_x = (float)read_i16_le(&linacc[0]) / 100.0f;
  const float lin_y = (float)read_i16_le(&linacc[2]) / 100.0f;
  const float lin_z = (float)read_i16_le(&linacc[4]) / 100.0f;

  const float gyro_x = (float)read_i16_le(&gyro[0]) / 16.0f;
  const float gyro_y = (float)read_i16_le(&gyro[2]) / 16.0f;
  const float gyro_z = (float)read_i16_le(&gyro[4]) / 16.0f;

  const float x = (float)read_i16_le(&quat[0]) / 16384.0f;
  const float y = (float)read_i16_le(&quat[2]) / 16384.0f;
  const float z = (float)read_i16_le(&quat[4]) / 16384.0f;
  const float w = (float)read_i16_le(&quat[6]) / 16384.0f;

  teleplot_emit_float("bno_lin_x", lin_x);
  teleplot_emit_float("bno_lin_y", lin_y);
  teleplot_emit_float("bno_lin_z", lin_z);
  teleplot_emit_float("bno_gyro_x", gyro_x);
  teleplot_emit_float("bno_gyro_y", gyro_y);
  teleplot_emit_float("bno_gyro_z", gyro_z);
  teleplot_emit_float("bno_quat_x", x);
  teleplot_emit_float("bno_quat_y", y);
  teleplot_emit_float("bno_quat_z", z);
  teleplot_emit_float("bno_quat_w", w);
  teleplot_emit_uint("bno_cal_sys", (calib >> 6) & 0x03);
  teleplot_emit_uint("bno_cal_gyro", (calib >> 4) & 0x03);
  teleplot_emit_uint("bno_cal_accel", (calib >> 2) & 0x03);
  teleplot_emit_uint("bno_cal_mag", calib & 0x03);
}

static void print_bno_status(void) {
  uint8_t calib = 0;
  uint8_t linacc[6] = {0};
  uint8_t gyro[6] = {0};
  uint8_t quat[8] = {0};

  if (i2c_read_reg(BNO_REG_CALIB_STAT, &calib, 1) != ESP_OK ||
      i2c_read_reg(BNO_REG_LINACC_DATA, linacc, sizeof(linacc)) != ESP_OK ||
      i2c_read_reg(BNO_REG_GYRO_DATA, gyro, sizeof(gyro)) != ESP_OK ||
      i2c_read_reg(BNO_REG_QUATERNION_DATA, quat, sizeof(quat)) != ESP_OK) {
    ESP_LOGW(TAG, "BNO055 read failed at 0x%02X", bno_addr);
    bno_ready = false;
    return;
  }

  emit_bno_teleplot(linacc, gyro, quat, calib);
}

static bool try_init_bno_on_addr(uint8_t addr) {
  if (i2c_master_probe(i2c_bus, addr, 200) != ESP_OK) {
    ESP_LOGW(TAG, "BNO055 not responding at 0x%02X", addr);
    return false;
  }

  i2c_device_config_t dev_cfg = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = addr,
      .scl_speed_hz = I2C_FREQ_HZ,
      .scl_wait_us = 0,
      .flags.disable_ack_check = 0,
  };

  i2c_master_dev_handle_t candidate = NULL;
  if (i2c_master_bus_add_device(i2c_bus, &dev_cfg, &candidate) != ESP_OK) {
    ESP_LOGW(TAG, "Failed adding I2C device at 0x%02X", addr);
    return false;
  }

  bno_addr = addr;
  bno_dev = candidate;
  uint8_t chip_id = 0;
  if (i2c_read_reg(BNO_REG_CHIP_ID, &chip_id, 1) != ESP_OK) {
    ESP_LOGW(TAG, "BNO055 not responding at 0x%02X", bno_addr);
    i2c_master_bus_rm_device(bno_dev);
    bno_dev = NULL;
    return false;
  }

  if (chip_id != 0xA0) {
    ESP_LOGW(TAG, "Unexpected BNO055 chip id 0x%02X at 0x%02X", chip_id, bno_addr);
    i2c_master_bus_rm_device(bno_dev);
    bno_dev = NULL;
    return false;
  }

  if (i2c_write_reg(BNO_REG_OPR_MODE, BNO_MODE_CONFIG) != ESP_OK) {
    i2c_master_bus_rm_device(bno_dev);
    bno_dev = NULL;
    return false;
  }
  vTaskDelay(pdMS_TO_TICKS(30));
  if (i2c_write_reg(BNO_REG_PAGE_ID, 0x00) != ESP_OK) {
    i2c_master_bus_rm_device(bno_dev);
    bno_dev = NULL;
    return false;
  }
  if (i2c_write_reg(BNO_REG_OPR_MODE, BNO_MODE_NDOF) != ESP_OK) {
    i2c_master_bus_rm_device(bno_dev);
    bno_dev = NULL;
    return false;
  }
  vTaskDelay(pdMS_TO_TICKS(20));

  ESP_LOGI(TAG, "BNO055 ready at 0x%02X", bno_addr);
  return true;
}

static bool init_bno(void) {
  if (bno_dev != NULL) {
    i2c_master_bus_rm_device(bno_dev);
    bno_dev = NULL;
  }
  if (try_init_bno_on_addr(BNO_ADDR_PRIMARY)) return true;
  if (try_init_bno_on_addr(BNO_ADDR_SECONDARY)) return true;
  return false;
}

static void init_i2c(void) {
  const i2c_master_bus_config_t config = {
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .i2c_port = I2C_NUM_0,
      .scl_io_num = I2C_SCL_GPIO,
      .sda_io_num = I2C_SDA_GPIO,
      .glitch_ignore_cnt = 7,
      .flags.enable_internal_pullup = true,
  };

  ESP_ERROR_CHECK(i2c_new_master_bus(&config, &i2c_bus));
  ESP_LOGI(TAG, "I2C initialized SDA=%d SCL=%d", I2C_SDA_GPIO, I2C_SCL_GPIO);
}

static void init_button(void) {
  const gpio_config_t cfg = {
      .pin_bit_mask = 1ULL << BUTTON_GPIO,
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_ENABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  ESP_ERROR_CHECK(gpio_config(&cfg));
  last_raw_button = gpio_get_level(BUTTON_GPIO);
  stable_button = last_raw_button;
  button_press_start_ms = 0;
  button_long_press_fired = false;
  ESP_LOGI(TAG, "Button initialized on GPIO %d", BUTTON_GPIO);
}

// static void enter_deep_sleep(void) {
//   ESP_LOGI(TAG, "Long press detected -> entering deep sleep. Press button to wake.");

//   if (led_ready) led_set_rgb(0, 0, 0);

//   ESP_ERROR_CHECK(esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL));
//   ESP_ERROR_CHECK(esp_sleep_enable_ext1_wakeup_io(1ULL << BUTTON_GPIO, ESP_EXT1_WAKEUP_ANY_HIGH));
//   vTaskDelay(pdMS_TO_TICKS(100));
//   esp_deep_sleep_start();
// }

static bool handle_button(void) {
  bool short_press = false;
  const int raw = gpio_get_level(BUTTON_GPIO);
  const uint32_t now = now_ms();

  if (raw != last_raw_button) {
    last_raw_button = raw;
    last_button_change_ms = now;
  }

  if ((now - last_button_change_ms) < 30) return false;

  if (raw != stable_button) {
    stable_button = raw;
    ESP_LOGI(TAG, "BUTTON: %s", stable_button ? "HIGH" : "LOW");

    if (stable_button) {
      button_press_start_ms = now;
      button_long_press_fired = false;
    } else {
      if (!button_long_press_fired && button_press_start_ms != 0 && (now - button_press_start_ms) < BUTTON_LONG_PRESS_MS) {
        short_press = true;
      }
      button_press_start_ms = 0;
      button_long_press_fired = false;
    }
  }

  // if (stable_button && !button_long_press_fired && button_press_start_ms != 0 && (now - button_press_start_ms) >= BUTTON_LONG_PRESS_MS) {
  //   button_long_press_fired = true;
  //   enter_deep_sleep();
  // }

  return short_press;
}

static void hsv_to_rgb(uint8_t h, uint8_t *r, uint8_t *g, uint8_t *b) {
  const uint8_t region = h / 43;
  const uint8_t rem = (h - (region * 43)) * 6;
  const uint8_t p = 0;
  const uint8_t q = (uint8_t)(255 - rem);
  const uint8_t t = rem;

  switch (region) {
    case 0:
      *r = 255; *g = t; *b = p; break;
    case 1:
      *r = q; *g = 255; *b = p; break;
    case 2:
      *r = p; *g = 255; *b = t; break;
    case 3:
      *r = p; *g = q; *b = 255; break;
    case 4:
      *r = t; *g = p; *b = 255; break;
    default:
      *r = 255; *g = p; *b = q; break;
  }
}

static void led_set_rgb(uint8_t r, uint8_t g, uint8_t b) {
  if (!led_ready) return;

  // SK6812/WS2812 pixel order is usually GRB.
  for (int i = 0; i < LED_COUNT; i++) {
    led_payload[i * 3 + 0] = g;
    led_payload[i * 3 + 1] = r;
    led_payload[i * 3 + 2] = b;
  }

  if (rmt_transmit(led_chan, led_encoder, led_payload, sizeof(led_payload), &led_tx_config) == ESP_OK) {
    rmt_tx_wait_all_done(led_chan, 100);
  }
}

static void apply_led_mode(void) {
  if (!led_ready) return;

  if (led_mode < (uint8_t)(sizeof(solid_colors) / sizeof(solid_colors[0]))) {
    const led_color_t c = solid_colors[led_mode];
    led_set_rgb(c.r, c.g, c.b);
    ESP_LOGI(TAG, "LED mode: %s", c.name);
  } else {
    rainbow_hue = 0;
    ESP_LOGI(TAG, "LED mode: RAINBOW");
  }
}

static void update_led_animation(void) {
  if (!led_ready) return;
  if (led_mode != (uint8_t)(sizeof(solid_colors) / sizeof(solid_colors[0]))) return;

  const uint32_t now = now_ms();
  if ((now - last_led_anim_ms) < 20) return;
  last_led_anim_ms = now;

  for (int i = 0; i < LED_COUNT; i++) {
    uint8_t r, g, b;
    hsv_to_rgb((uint8_t)(rainbow_hue + (i * (256 / LED_COUNT))), &r, &g, &b);
    led_payload[i * 3 + 0] = g;
    led_payload[i * 3 + 1] = r;
    led_payload[i * 3 + 2] = b;
  }
  rainbow_hue++;

  if (rmt_transmit(led_chan, led_encoder, led_payload, sizeof(led_payload), &led_tx_config) == ESP_OK) {
    rmt_tx_wait_all_done(led_chan, 100);
  }
}

static void init_led(void) {
  rmt_tx_channel_config_t tx_chan_config = {
      .gpio_num = LED_GPIO,
      .clk_src = RMT_CLK_SRC_DEFAULT,
      .resolution_hz = 10 * 1000 * 1000,
      .mem_block_symbols = 64,
      .trans_queue_depth = 1,
      .intr_priority = 0,
      .flags = {
          .invert_out = 0,
          .with_dma = 0,
          .allow_pd = 0,
          .init_level = 0,
      },
  };

  rmt_bytes_encoder_config_t bytes_encoder_config = {
      .bit0 = {
          .level0 = 1,
          .duration0 = 3,
          .level1 = 0,
          .duration1 = 9,
      },
      .bit1 = {
          .level0 = 1,
          .duration0 = 6,
          .level1 = 0,
          .duration1 = 6,
      },
      .flags = {
          .msb_first = 1,
      },
  };

  if (rmt_new_tx_channel(&tx_chan_config, &led_chan) != ESP_OK) {
    ESP_LOGW(TAG, "SK6812 init failed on GPIO %d", LED_GPIO);
    return;
  }

  if (rmt_new_bytes_encoder(&bytes_encoder_config, &led_encoder) != ESP_OK) {
    ESP_LOGW(TAG, "SK6812 encoder init failed");
    return;
  }

  if (rmt_enable(led_chan) != ESP_OK) {
    ESP_LOGW(TAG, "SK6812 RMT enable failed");
    return;
  }

  led_ready = true;
  ESP_LOGI(TAG, "SK6812 output initialized on GPIO %d (%d LEDs)", LED_GPIO, LED_COUNT);
  apply_led_mode();
}

static void init_battery_adc(void) {
  adc_unit_t adc_unit_id;
  if (adc_oneshot_io_to_channel(BATTERY_GPIO, &adc_unit_id, &battery_channel) != ESP_OK) {
    ESP_LOGW(TAG, "Battery ADC pin mapping failed on GPIO %d", BATTERY_GPIO);
    return;
  }

  adc_oneshot_unit_init_cfg_t unit_cfg = {
      .unit_id = adc_unit_id,
      .ulp_mode = ADC_ULP_MODE_DISABLE,
  };
  if (adc_oneshot_new_unit(&unit_cfg, &adc_unit) != ESP_OK) {
    ESP_LOGW(TAG, "Battery ADC unit init failed");
    return;
  }

  adc_oneshot_chan_cfg_t channel_cfg = {
      .atten = ADC_ATTEN_DB_12,
      .bitwidth = ADC_BITWIDTH_DEFAULT,
  };
  if (adc_oneshot_config_channel(adc_unit, battery_channel, &channel_cfg) != ESP_OK) {
    ESP_LOGW(TAG, "Battery ADC channel config failed");
    return;
  }

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
  adc_cali_curve_fitting_config_t cali_cfg = {
      .unit_id = adc_unit_id,
      .chan = battery_channel,
      .atten = ADC_ATTEN_DB_12,
      .bitwidth = ADC_BITWIDTH_DEFAULT,
  };
  if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &adc_cali) == ESP_OK) {
    adc_cali_enabled = true;
  }
#endif

  battery_ready = true;
  ESP_LOGI(TAG, "Battery ADC ready on GPIO %d (channel %d)", BATTERY_GPIO, battery_channel);
}

static void print_battery_status(void) {
  if (!battery_ready) return;

  int raw_sum = 0;
  for (int i = 0; i < 8; i++) {
    int raw = 0;
    if (adc_oneshot_read(adc_unit, battery_channel, &raw) != ESP_OK) {
      ESP_LOGW(TAG, "Battery ADC read failed");
      return;
    }
    raw_sum += raw;
  }

  const int raw_avg = raw_sum / 8;
  int mv = 0;
  if (adc_cali_enabled) {
    if (adc_cali_raw_to_voltage(adc_cali, raw_avg, &mv) != ESP_OK) {
      ESP_LOGW(TAG, "Battery calibration conversion failed");
      return;
    }
  } else {
    mv = (raw_avg * 3300) / 4095;
  }

  const float battery_v = ((float)mv / 1000.0f) * BATTERY_DIVIDER;
  const float percent = clampf((battery_v - BATTERY_VMIN) / (BATTERY_VMAX - BATTERY_VMIN) * 100.0f, 0.0f, 100.0f);
  ESP_LOGI(TAG, "BATT: %.3fV (%.0f%%) raw=%d", battery_v, percent, raw_avg);
}

void app_main(void) {
  ESP_LOGI(TAG, "=== BNO055 + Button + Battery + SK6812 Test ===");

  esp_sleep_wakeup_cause_t wake_cause = esp_sleep_get_wakeup_cause();
  if (wake_cause == ESP_SLEEP_WAKEUP_EXT1) {
    ESP_LOGI(TAG, "Wakeup cause: BUTTON (EXT1)");
  } else {
    ESP_LOGI(TAG, "Wakeup cause: %d", (int)wake_cause);
  }

  init_i2c();
  init_button();
  init_battery_adc();
  init_led();

  for (;;) {
    if (handle_button()) {
      led_mode++;
      if (led_mode > (uint8_t)(sizeof(solid_colors) / sizeof(solid_colors[0]))) led_mode = 0;
      apply_led_mode();
    }

    const uint32_t now = now_ms();

    if (!bno_ready) {
      static uint32_t last_retry_ms = 0;
      if (now - last_retry_ms > 200) {
        last_retry_ms = now;
        bno_ready = init_bno();
      }
    } else if (now - last_bno_ms >= BNO_SAMPLE_PERIOD_MS) {
      last_bno_ms = now;
      print_bno_status();
    }

    if (now - last_battery_ms >= 1000) {
      last_battery_ms = now;
      print_battery_status();
    }

    update_led_animation();
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}
