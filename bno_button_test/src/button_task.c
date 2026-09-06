#include "button_task.h"

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_sleep.h"
#include "bno.h"
#include "led_output.h"

#define BUTTON_GPIO GPIO_NUM_0
#define BUTTON_LONG_PRESS_MS 3000
#define BUTTON_DEBOUNCE_MS 30

static const char *TAG = "button";

static int stable_button = 1;
static int idle_button_level = 1;
static uint32_t button_press_start_ms = 0;
static bool button_long_press_armed = false;
static QueueHandle_t button_edge_queue = NULL;
static QueueHandle_t button_action_queue = NULL;

TaskHandle_t button_task_handle = NULL;

typedef struct {
  uint32_t ts_ms;
  int level;
} button_edge_t;

static uint32_t now_ms(void) {
  return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

static void init_button(void) {
  const gpio_config_t cfg = {
      .pin_bit_mask = 1ULL << BUTTON_GPIO,
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_ENABLE,
      .intr_type = GPIO_INTR_ANYEDGE,
  };
  ESP_ERROR_CHECK(gpio_config(&cfg));

  stable_button = gpio_get_level(BUTTON_GPIO);
  idle_button_level = stable_button;
  button_press_start_ms = 0;
  button_long_press_armed = false;
  ESP_LOGI(TAG, "Button initialized on GPIO %d (idle level=%d)", BUTTON_GPIO, idle_button_level);
}

static void IRAM_ATTR button_isr_handler(void *arg) {
  (void)arg;
  if (button_edge_queue == NULL) return;

  button_edge_t ev = {
      .ts_ms = (uint32_t)(esp_timer_get_time() / 1000ULL),
      .level = gpio_get_level(BUTTON_GPIO),
  };

  BaseType_t woken = pdFALSE;
  xQueueSendFromISR(button_edge_queue, &ev, &woken);
  if (woken == pdTRUE) {
    portYIELD_FROM_ISR();
  }
}

static void publish_button_event(button_event_type_t type, uint32_t ts_ms) {
  if (button_action_queue == NULL) return;
  button_event_t event = {
      .type = type,
      .ts_ms = ts_ms,
  };
  (void)xQueueSend(button_action_queue, &event, 0);
}

static void button_task(void *arg) {
  (void)arg;

  for (;;) {
    const uint32_t now = now_ms();
    button_edge_t ev;
    const BaseType_t got_event = xQueueReceive(button_edge_queue, &ev, pdMS_TO_TICKS(20));

    if (got_event == pdTRUE) {
      (void)ev;
      vTaskDelay(pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS));
      const int debounced_level = gpio_get_level(BUTTON_GPIO);

      if (debounced_level != stable_button) {
        stable_button = debounced_level;
        const bool pressed = (stable_button != idle_button_level);

        if (pressed) {
          button_press_start_ms = now;
          button_long_press_armed = false;
        } else {
          if (button_long_press_armed) {
            publish_button_event(BUTTON_EVENT_LONG_PRESS_RELEASE, now);
            blink_sleep_ready();
            enter_deep_sleep();
          } else if (button_press_start_ms != 0 &&
              (now - button_press_start_ms) < BUTTON_LONG_PRESS_MS) {
            publish_button_event(BUTTON_EVENT_SHORT_PRESS, now);
          }
          button_press_start_ms = 0;
          button_long_press_armed = false;
        }
      }
    }

    const bool still_pressed = (stable_button != idle_button_level);
    if (still_pressed && !button_long_press_armed && button_press_start_ms != 0 &&
        (now - button_press_start_ms) >= BUTTON_LONG_PRESS_MS) {
      button_long_press_armed = true;
      publish_button_event(BUTTON_EVENT_LONG_PRESS_READY, now);
      ESP_LOGI(TAG, "Long press armed: release to sleep");
    }
  }
}

void start_button_task(void) {
  init_button();

  button_edge_queue = xQueueCreate(16, sizeof(button_edge_t));
  button_action_queue = xQueueCreate(8, sizeof(button_event_t));
  configASSERT(button_edge_queue != NULL);
  configASSERT(button_action_queue != NULL);

  const esp_err_t isr_install_err = gpio_install_isr_service(0);
  if (isr_install_err != ESP_OK && isr_install_err != ESP_ERR_INVALID_STATE) {
    ESP_ERROR_CHECK(isr_install_err);
  }
  ESP_ERROR_CHECK(gpio_isr_handler_add(BUTTON_GPIO, button_isr_handler, NULL));

  // Highest priority in the app: a held button must be able to preempt the
  // IMU/OSC/LED tasks (all priority 5) so long-press-to-sleep and mode
  // cycling stay responsive even while they're busy. Those tasks were
  // starving this one at its old priority 4 (always losing to priority-5
  // tasks), which looked like the button "stopped working" - device_state
  // never advanced and deep sleep was never entered, while sensor-driven
  // LED rendering kept running fine (it doesn't depend on button_task).
  xTaskCreatePinnedToCore(button_task, "button_task", 2048, NULL, 10, &button_task_handle, tskNO_AFFINITY);
}

bool button_task_take_event(button_event_t *event, TickType_t wait_ticks) {
  if (event == NULL || button_action_queue == NULL) return false;
  return xQueueReceive(button_action_queue, event, wait_ticks) == pdTRUE;
}

void enter_deep_sleep(void) {
	ESP_LOGI(TAG, "Long press detected -> entering deep sleep. Press button to wake.");
	bno_set_sleeping(true);
	// bno_task only checks the sleeping flag at the top of its loop, so it
	// may still be mid-transaction (an I2C read burst, up to its 100ms
	// per-call timeout) on a different task right when we set that flag.
	// Racing bno_suspend()'s own I2C writes against that in-flight read -
	// or worse, cutting the read off mid-byte via esp_deep_sleep_start()
	// below - can leave the BNO055 holding the bus (SDA stuck low), which
	// then fails every I2C transaction after the next reboot until power
	// is fully removed from the sensor (see i2c_bus_recover() in bno.c,
	// added as a belt-and-suspenders fix for exactly that case). Waiting
	// out one full sample period plus the worst-case transaction timeout
	// here guarantees bno_task has actually parked itself in its paused
	// branch before we touch the bus again or power down.
	vTaskDelay(pdMS_TO_TICKS(BNO_SAMPLE_PERIOD_MS + 120));
	if (bno_ready) {
		(void)bno_suspend();
	}
	led_output_set_all_rgbw(0, 0, 0, 0);

	ESP_ERROR_CHECK(esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL));
	ESP_ERROR_CHECK(esp_sleep_enable_ext1_wakeup_io(1ULL << BUTTON_WAKE_GPIO, ESP_EXT1_WAKEUP_ANY_HIGH));

	gpio_reset_pin(I2C_SDA_GPIO);
	gpio_reset_pin(I2C_SCL_GPIO);
	gpio_set_direction(I2C_SDA_GPIO, GPIO_MODE_INPUT);
	gpio_set_direction(I2C_SCL_GPIO, GPIO_MODE_INPUT);
	gpio_pullup_dis(I2C_SDA_GPIO);
	gpio_pullup_dis(I2C_SCL_GPIO);
	gpio_pulldown_dis(I2C_SDA_GPIO);
	gpio_pulldown_dis(I2C_SCL_GPIO);

	gpio_reset_pin(GPIO_NUM_1);
	gpio_set_direction(GPIO_NUM_1, GPIO_MODE_INPUT);
	gpio_pullup_dis(GPIO_NUM_1);
	gpio_pulldown_dis(GPIO_NUM_1);

	esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
	esp_deep_sleep_start();
}

void blink_sleep_ready(void) {
	for (int i = 0; i < 2; i++) {
		led_output_set_all_rgbw(0, 0, 255, 0); // blue: use the B channel, not W (W drives the pure-white diode)
		vTaskDelay(pdMS_TO_TICKS(80));
		led_output_set_all_rgbw(0, 0, 0, 0);
		vTaskDelay(pdMS_TO_TICKS(70));
	}
}