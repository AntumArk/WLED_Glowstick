#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "driver/gpio.h"

#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"

#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "bno.h"
#include "battery.h"


#define BUTTON_GPIO GPIO_NUM_0

#define LED_GPIO GPIO_NUM_18
#define LED_COUNT 9


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


static int last_raw_button = 1;
static int stable_button = 1;
static uint32_t last_button_change_ms = 0;
static uint32_t button_press_start_ms = 0;
static bool button_long_press_fired = false;



static rmt_channel_handle_t led_chan = NULL;
static rmt_encoder_handle_t led_encoder = NULL;
static bool led_ready = false;
static uint8_t led_mode = 0;
static uint8_t rainbow_hue = 0;
static uint8_t led_payload[LED_COUNT * 3] = {0};



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



void app_main(void) {
  ESP_LOGI(TAG, "=== BNO055 + Button + Battery + SK6812 Test ===");

  esp_sleep_wakeup_cause_t wake_cause = esp_sleep_get_wakeup_cause();
  if (wake_cause == ESP_SLEEP_WAKEUP_EXT1) {
    ESP_LOGI(TAG, "Wakeup cause: BUTTON (EXT1)");
  } else {
    ESP_LOGI(TAG, "Wakeup cause: %d", (int)wake_cause);
  }

  init_button();
  start_battery_task();

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

    update_led_animation();
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}
