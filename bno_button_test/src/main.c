#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "esp_log.h"
#include "esp_sleep.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "battery.h"
#include "button_task.h"
#include "glowstick_mode.h"
#include "osc.h"
#include "osc_config.h"
#include "wifi_manager.h"
#include "wifi_web_config.h"
#include "swing_mode.h"

static const char *TAG = "bno_button_test";

void app_main(void) {
  ESP_LOGI(TAG, "=== BNO055 + Button + Battery + SK6812 Test ===");

  const uint32_t wake_causes = esp_sleep_get_wakeup_causes();
  if ((wake_causes & BIT(ESP_SLEEP_WAKEUP_EXT1)) != 0U) {
    ESP_LOGI(TAG, "Wakeup cause: BUTTON (EXT1)");
  } else {
    ESP_LOGI(TAG, "Wakeup causes bitmap: 0x%08lx", (unsigned long)wake_causes);
  }

  esp_err_t nvs_err = nvs_flash_init();
  if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    nvs_err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(nvs_err);

  osc_task_init();

  start_battery_task();
  glowstick_mode_init();
  start_button_task();
  swing_mode_init();

  vTaskDelete(NULL);
}
