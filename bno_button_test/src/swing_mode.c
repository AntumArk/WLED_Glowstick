#include "swing_mode.h"

#include <stdbool.h>
#include <stdint.h>

#include "led_output.h"
#include "zinc_time.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "state_machine.h"

#define PEAK_FLASH_FADE_IN_MS 100U
#define PEAK_FLASH_FADE_OUT_MS 500U
#define PEAK_FLASH_DURATION_MS (PEAK_FLASH_FADE_IN_MS + PEAK_FLASH_FADE_OUT_MS)

static uint32_t flash_started_ms = 0;
static uint32_t random_state = 0x6D2B79F5U;
static uint8_t flash_red = 0;
static uint8_t flash_green = 0;
static uint8_t flash_blue = 0;

TaskHandle_t swing_mode_task_handle = NULL;


void swing_mode_task(void) {
  while (true) {
    if (device_state == DEVICE_STATE_SWING_MODE)
    {
      const uint32_t now = now_ms();
      swing_mode_handle_peak(now);
      swing_mode_render(now);
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void swing_mode_init(void) {
  swing_mode_reset();
  xTaskCreatePinnedToCore((TaskFunction_t)swing_mode_task, "swing_mode_task", 4096, NULL, 5, &swing_mode_task_handle, tskNO_AFFINITY);
}

static uint32_t next_random(void) {
  random_state ^= random_state << 13;
  random_state ^= random_state >> 17;
  random_state ^= random_state << 5;
  return random_state;
}

static void choose_flash_color(void) {
  const uint8_t value = (uint8_t)(next_random() & 0xFFU);
  const uint8_t accent = (uint8_t)(next_random() & 0xFFU);

  switch (next_random() % 6U) {
    case 0:
      flash_red = 255;
      flash_green = value;
      flash_blue = 0;
      break;
    case 1:
      flash_red = value;
      flash_green = 255;
      flash_blue = 0;
      break;
    case 2:
      flash_red = 0;
      flash_green = 255;
      flash_blue = value;
      break;
    case 3:
      flash_red = 0;
      flash_green = value;
      flash_blue = 255;
      break;
    case 4:
      flash_red = value;
      flash_green = 0;
      flash_blue = 255;
      break;
    default:
      flash_red = 255;
      flash_green = 0;
      flash_blue = accent;
      break;
  }
}

void swing_mode_reset(void) {
  flash_started_ms = 0;
}

void swing_mode_handle_peak(uint32_t now_ms) {
  choose_flash_color();
  flash_started_ms = now_ms;
}

void swing_mode_render(uint32_t now_ms) {
  const uint32_t elapsed_ms = now_ms - flash_started_ms;
  if (flash_started_ms == 0 || elapsed_ms >= PEAK_FLASH_DURATION_MS) {
    led_output_set_all_rgbw(0, 0, 0, 0);
    return;
  }

  float brightness = 0.0f;
  if (elapsed_ms < PEAK_FLASH_FADE_IN_MS) {
    brightness = (float)elapsed_ms / (float)PEAK_FLASH_FADE_IN_MS;
  } else {
    brightness = (float)(PEAK_FLASH_DURATION_MS - elapsed_ms) / (float)PEAK_FLASH_FADE_OUT_MS;
  }

  led_output_set_all_rgbw((uint8_t)((float)flash_red * brightness),
      (uint8_t)((float)flash_green * brightness),
      (uint8_t)((float)flash_blue * brightness), 0);
}
