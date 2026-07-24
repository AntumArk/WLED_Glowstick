#include "button_task.h"

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#define BUTTON_GPIO GPIO_NUM_0
#define BUTTON_LONG_PRESS_MS 2000
#define BUTTON_DEBOUNCE_MS 30

static const char *TAG = "button";

static int stable_button = 1;
static int idle_button_level = 1;
static uint32_t button_press_start_ms = 0;
static bool button_long_press_fired = false;
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
  button_long_press_fired = false;
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
          button_long_press_fired = false;
        } else {
          if (!button_long_press_fired && button_press_start_ms != 0 &&
              (now - button_press_start_ms) < BUTTON_LONG_PRESS_MS) {
            publish_button_event(BUTTON_EVENT_SHORT_PRESS, now);
          }
          button_press_start_ms = 0;
          button_long_press_fired = false;
        }
      }
    }

    const bool still_pressed = (stable_button != idle_button_level);
    if (still_pressed && !button_long_press_fired && button_press_start_ms != 0 &&
        (now - button_press_start_ms) >= BUTTON_LONG_PRESS_MS) {
      button_long_press_fired = true;
      publish_button_event(BUTTON_EVENT_LONG_PRESS, now);
      // Deep sleep intentionally disabled for now.
      ESP_LOGI(TAG, "Long press event published (sleep disabled)");
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

  xTaskCreatePinnedToCore(button_task, "button_task", 2048, NULL, 4, &button_task_handle, tskNO_AFFINITY);
}

bool button_task_take_event(button_event_t *event, TickType_t wait_ticks) {
  if (event == NULL || button_action_queue == NULL) return false;
  return xQueueReceive(button_action_queue, event, wait_ticks) == pdTRUE;
}
