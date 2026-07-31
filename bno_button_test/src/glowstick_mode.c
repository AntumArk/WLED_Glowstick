#include "glowstick_mode.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include "driver/gpio.h"
#include "esp_sleep.h"
#include "esp_timer.h"

#include "esp_log.h"
#include "button_task.h"
#include "led_output.h"
#include "swing_mode.h"

#define GLOW_GREEN_MAX 255U
#define SWING_PEAK_THRESHOLD_MS2 15.0f
#define SWING_PEAK_COOLDOWN_MS 140U
#define BUTTON_WAKE_GPIO GPIO_NUM_0
#define BALL_TUBE_LENGTH_M 0.20f
#define BALL_ACCELERATION_SCALE 0.8f
#define BALL_DAMPING_PER_SECOND 1.5f
#define BALL_BOUNCE_RESTITUTION 0.65f
#define BALL_CHARGE_RATE_PER_SECOND 0.8f

typedef struct {
	uint8_t r;
	uint8_t g;
	uint8_t b;
	uint8_t w;
	const char *name;
} glow_color_t;

static const char *TAG = "bno_button_test";
TaskHandle_t glowstick_task_handle = NULL;

static const glow_color_t glow_colors[] = {
	{0, 255, 0, 0, "GREEN"},
	{180, 0, 255, 0, "PURPLE"},
	{140, 255, 0, 0, "LIME"},
	{255, 0, 0, 0, "RED"},
	{0, 0, 0, 255, "WHITE"},
	{255, 255, 255, 255, "BLAST"},
};
static volatile uint8_t glow_color_index = 0;
static portMUX_TYPE glow_state_lock = portMUX_INITIALIZER_UNLOCKED;

static uint32_t last_mode_update_ms = 0;
static uint32_t last_swing_peak_ms = 0;
static bool swing_armed = true;
static float led_charge[LED_OUTPUT_COUNT] = {0.0f};
static float ball_position = BALL_TUBE_LENGTH_M / 2.0f;
static float ball_velocity = 0.0f;

static void render_charge(void);
static void update_bouncing_ball(float dt_seconds, const float linear_acceleration_ms2[3]);
static void render_bouncing_ball(const glow_color_t *color);

static void blink_sleep_ready(void) {
	for (int i = 0; i < 2; i++) {
		led_output_set_all_rgbw(0, 0, 0, 255);
		vTaskDelay(pdMS_TO_TICKS(80));
		led_output_set_all_rgbw(0, 0, 0, 0);
		vTaskDelay(pdMS_TO_TICKS(70));
	}
	render_charge();
}

static void enter_deep_sleep(void) {
	ESP_LOGI(TAG, "Long press detected -> entering deep sleep. Press button to wake.");
	bno_set_sleeping(true);
	if (bno_ready) {
		(void)bno_suspend();
	}
	led_output_set_all_rgbw(0, 0, 0, 0);

	ESP_ERROR_CHECK(esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL));
	ESP_ERROR_CHECK(esp_sleep_enable_ext1_wakeup_io(1ULL << BUTTON_WAKE_GPIO, ESP_EXT1_WAKEUP_ANY_HIGH));
	esp_deep_sleep_start();
}

static float clamp01(float x) {
	if (x < 0.0f) return 0.0f;
	if (x > 1.0f) return 1.0f;
	return x;
}

static uint32_t now_ms(void) {
  return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

static bool get_current_linear_acceleration(float linear_acceleration_ms2[3]) {
	if (!bno_ready) return false;

	linear_acceleration_ms2[0] = (float)last_bno_teleplot.linacc[0] / 100.0f;
	linear_acceleration_ms2[1] = (float)last_bno_teleplot.linacc[1] / 100.0f;
	linear_acceleration_ms2[2] = (float)last_bno_teleplot.linacc[2] / 100.0f;
	return true;
}

static void render_charge(void) {
	if (!led_output_ready()) return;

	uint8_t color_index_snapshot = 0;
	taskENTER_CRITICAL(&glow_state_lock);
	color_index_snapshot = glow_color_index;
	taskEXIT_CRITICAL(&glow_state_lock);

	const uint8_t color_count = (uint8_t)(sizeof(glow_colors) / sizeof(glow_colors[0]));
	if (color_index_snapshot == color_count) {
		swing_mode_render(now_ms());
		return;
	}

	const glow_color_t c = glow_colors[color_index_snapshot];
	render_bouncing_ball(&c);
}


static void update_bouncing_ball(float dt_seconds, const float linear_acceleration_ms2[3]) {
	ball_velocity += linear_acceleration_ms2[1] * BALL_ACCELERATION_SCALE * dt_seconds;
	ball_velocity /= 1.0f + (BALL_DAMPING_PER_SECOND * dt_seconds);
	ball_position += ball_velocity * dt_seconds;

	if (ball_position < 0.0f) {
		ball_position = -ball_position;
		ball_velocity = -ball_velocity * BALL_BOUNCE_RESTITUTION;
	} else if (ball_position > BALL_TUBE_LENGTH_M) {
		ball_position = 2.0f * BALL_TUBE_LENGTH_M - ball_position;
		ball_velocity = -ball_velocity * BALL_BOUNCE_RESTITUTION;
	}

	for (uint8_t led = 0; led < LED_OUTPUT_COUNT; led++) {
		led_charge[led] = clamp01(led_charge[led] - dt_seconds / ((float)TIME_TO_FADE_MS / 1000.0f));
	}

	const uint8_t ball_led = (uint8_t)lroundf(
		(ball_position / BALL_TUBE_LENGTH_M) * (float)(LED_OUTPUT_COUNT - 1U));
	led_charge[ball_led] = clamp01(led_charge[ball_led] + BALL_CHARGE_RATE_PER_SECOND * dt_seconds);
}

static void render_bouncing_ball(const glow_color_t *color) {
	float charge_snapshot[LED_OUTPUT_COUNT] = {0.0f};
	float ball_position_snapshot = 0.0f;
	taskENTER_CRITICAL(&glow_state_lock);
	for (uint8_t led = 0; led < LED_OUTPUT_COUNT; led++) {
		charge_snapshot[led] = led_charge[led];
	}
	ball_position_snapshot = ball_position;
	taskEXIT_CRITICAL(&glow_state_lock);

	for (uint8_t led = 0; led < LED_OUTPUT_COUNT; led++) {
		const float brightness = charge_snapshot[led];
		led_output_set_pixel_rgbw(led, (uint8_t)((float)color->r * brightness),
			(uint8_t)((float)color->g * brightness), (uint8_t)((float)color->b * brightness),
			(uint8_t)((float)color->w * brightness));
	}

	const uint8_t ball_led = (uint8_t)lroundf(
		(ball_position_snapshot / BALL_TUBE_LENGTH_M) * (float)(LED_OUTPUT_COUNT - 1U));
	led_output_set_pixel_rgbw(ball_led, color->r, color->g, color->b, color->w);
	led_output_show();
}

static void handle_button_events(void) {
	button_event_t event;
	while (button_task_take_event(&event, 0)) {
		if (event.type == BUTTON_EVENT_SHORT_PRESS) {
			glowstick_mode_next_color();
			glowstick_mode_charge_full();
		} else if (event.type == BUTTON_EVENT_LONG_PRESS_READY) {
			blink_sleep_ready();
		} else if (event.type == BUTTON_EVENT_LONG_PRESS_RELEASE) {
			enter_deep_sleep();
		}
	}
}



void glowstick_task() {
    while (1) {
	handle_button_events();

    const uint32_t now = now_ms();
	if (last_mode_update_ms == 0U) {
		last_mode_update_ms = now;
		vTaskDelay(pdMS_TO_TICKS(10));
		continue;
	}

	uint32_t dt_ms = now - last_mode_update_ms;
	if (dt_ms == 0U) {
		vTaskDelay(pdMS_TO_TICKS(1));
		continue;
	}
	if (dt_ms > 100U) dt_ms = 100U;
	last_mode_update_ms = now;

	float linear_acceleration_ms2[3] = {0.0f};
	const bool have_linear_acceleration = get_current_linear_acceleration(linear_acceleration_ms2);
	const float linear_acceleration_magnitude = sqrtf(
		linear_acceleration_ms2[0] * linear_acceleration_ms2[0] +
		linear_acceleration_ms2[1] * linear_acceleration_ms2[1] +
		linear_acceleration_ms2[2] * linear_acceleration_ms2[2]);
	const uint8_t color_count = (uint8_t)(sizeof(glow_colors) / sizeof(glow_colors[0]));
	taskENTER_CRITICAL(&glow_state_lock);
	if (glow_color_index == color_count) {
		if (have_linear_acceleration && linear_acceleration_magnitude < SWING_PEAK_THRESHOLD_MS2) {
			swing_armed = true;
		}
		if (have_linear_acceleration && swing_armed && linear_acceleration_magnitude >= SWING_PEAK_THRESHOLD_MS2 &&
			(now - last_swing_peak_ms) >= SWING_PEAK_COOLDOWN_MS) {
			swing_mode_handle_peak(now);
			last_swing_peak_ms = now;
			swing_armed = false;
		}
	} else if (have_linear_acceleration) {
		update_bouncing_ball((float)dt_ms / 1000.0f, linear_acceleration_ms2);
	}
	taskEXIT_CRITICAL(&glow_state_lock);
	render_charge();
    vTaskDelay(pdMS_TO_TICKS(10));
}
}

void glowstick_mode_init(void) {
	last_mode_update_ms = 0;
	last_swing_peak_ms = 0;
	swing_armed = true;
	ball_position = BALL_TUBE_LENGTH_M / 2.0f;
	ball_velocity = 0.0f;
	for (uint8_t led = 0; led < LED_OUTPUT_COUNT; led++) {
		led_charge[led] = 1.0f;
	}
	swing_mode_reset();
	glow_color_index = 0;
	led_output_init();
	bno_ready = init_bno();
	render_charge();
	ESP_LOGI(TAG, "LED mode: GLOWSTICK (%s)", glow_colors[glow_color_index].name);

    xTaskCreatePinnedToCore((TaskFunction_t)glowstick_task, "glowstick_task", 4096, NULL, 5, &glowstick_task_handle, tskNO_AFFINITY);
}

void glowstick_mode_next_color(void) {
	taskENTER_CRITICAL(&glow_state_lock);
	const uint8_t color_count = (uint8_t)(sizeof(glow_colors) / sizeof(glow_colors[0]));
	glow_color_index = (uint8_t)((glow_color_index + 1U) % (color_count + 1U));
	swing_mode_reset();
	taskEXIT_CRITICAL(&glow_state_lock);
	if (glow_color_index == color_count) {
		ESP_LOGI(TAG, "Glow color: PEAK FLASH");
	} else {
		ESP_LOGI(TAG, "Glow color: %s", glow_colors[glow_color_index].name);
	}
	render_charge();
}

void glowstick_mode_charge_full(void) {
	taskENTER_CRITICAL(&glow_state_lock);
	for (uint8_t led = 0; led < LED_OUTPUT_COUNT; led++) {
		led_charge[led] = 1.0f;
	}
	taskEXIT_CRITICAL(&glow_state_lock);
	render_charge();
}