
#include "battery.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

static const char *TAG = "battery";

adc_oneshot_unit_handle_t adc_unit = NULL;
adc_channel_t battery_channel = ADC_CHANNEL_0;
bool battery_ready = false;
adc_cali_handle_t adc_cali = NULL;
bool adc_cali_enabled = false;
uint32_t last_battery_ms = 0;

TaskHandle_t battery_task_handle = NULL;


float clampf(float value, float lo, float hi) {
  if (value < lo) return lo;
  if (value > hi) return hi;
  return value;
}

void init_battery_adc(void) {
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

void print_battery_status(void) {
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

void battery_task(void *arg) {
  while (1) {
    print_battery_status();
    vTaskDelay(pdMS_TO_TICKS(10000));
  }
}

void start_battery_task(void) {
    init_battery_adc();

    xTaskCreatePinnedToCore(battery_task, "battery_task", 2048, NULL, 1, &battery_task_handle, tskNO_AFFINITY);
    
}

