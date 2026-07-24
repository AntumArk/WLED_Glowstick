#include "led_output.h"

#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"
#include "esp_log.h"

#define LED_GPIO GPIO_NUM_18
#define LED_COUNT 9
#define LED_BYTES_PER_PIXEL 4

static const char *TAG = "bno_button_test";

static rmt_channel_handle_t led_chan = NULL;
static rmt_encoder_handle_t led_encoder = NULL;
static bool led_ready = false;
static uint8_t led_payload[LED_COUNT * LED_BYTES_PER_PIXEL] = {0};

static const rmt_transmit_config_t led_tx_config = {
    .loop_count = 0,
    .flags = {
        .eot_level = 0,
        .queue_nonblocking = 0,
    },
};

bool led_output_init(void) {
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
    return false;
  }

  if (rmt_new_bytes_encoder(&bytes_encoder_config, &led_encoder) != ESP_OK) {
    ESP_LOGW(TAG, "SK6812 encoder init failed");
    return false;
  }

  if (rmt_enable(led_chan) != ESP_OK) {
    ESP_LOGW(TAG, "SK6812 RMT enable failed");
    return false;
  }

  led_ready = true;
  ESP_LOGI(TAG, "SK6812 output initialized on GPIO %d (%d LEDs)", LED_GPIO, LED_COUNT);
  return true;
}

bool led_output_ready(void) {
  return led_ready;
}

void led_output_set_all_rgbw(uint8_t r, uint8_t g, uint8_t b, uint8_t w) {
  if (!led_ready) return;

  // SK6812 RGBW strips usually use GRBW order.
  for (int i = 0; i < LED_COUNT; i++) {
    led_payload[i * LED_BYTES_PER_PIXEL + 0] = g;
    led_payload[i * LED_BYTES_PER_PIXEL + 1] = r;
    led_payload[i * LED_BYTES_PER_PIXEL + 2] = b;
    led_payload[i * LED_BYTES_PER_PIXEL + 3] = w;
  }

  if (rmt_transmit(led_chan, led_encoder, led_payload, sizeof(led_payload), &led_tx_config) == ESP_OK) {
    rmt_tx_wait_all_done(led_chan, 100);
  }
}

void led_output_set_all_rgb(uint8_t r, uint8_t g, uint8_t b) {
  led_output_set_all_rgbw(r, g, b, 0);
}
