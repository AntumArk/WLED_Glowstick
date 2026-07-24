#include "glowstick_mode.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include "esp_timer.h"

#include "esp_log.h"
#include "button_task.h"
#include "led_output.h"

#define GLOW_GREEN_MAX 255U
#define SWING_AVG_WINDOW 48
#define CHARGE_TIME_MS 1200
#define LOW_FREQ_ENVELOPE_TAU_MS 350.0f
#define SWING_MAG_SATURATION 0.012f
#define MAX_CHARGE_STEP_PER_UPDATE 0.010f

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
};
static volatile uint8_t glow_color_index = 0;
static portMUX_TYPE glow_state_lock = portMUX_INITIALIZER_UNLOCKED;

static float swing_history[SWING_AVG_WINDOW] = {0.0f};
static uint8_t swing_hist_count = 0;
static uint8_t swing_hist_index = 0;
static float swing_hist_sum = 0.0f;

static float prev_quat[4] = {0.0f};
static bool prev_quat_valid = false;
static uint32_t last_mode_update_ms = 0;
static float charge = 1.0f;
static float low_freq_envelope = 0.0f;
static uint32_t last_bno_retry_ms = 0;
static uint32_t last_bno_sample_ms = 0;

static float clamp01(float x) {
	if (x < 0.0f) return 0.0f;
	if (x > 1.0f) return 1.0f;
	return x;
}

static uint32_t now_ms(void) {
  return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

static bool get_current_quat(float q[4]) {
	if (!bno_ready) return false;

	q[0] = (float)last_bno_teleplot.quat[0] / 16384.0f;
	q[1] = (float)last_bno_teleplot.quat[1] / 16384.0f;
	q[2] = (float)last_bno_teleplot.quat[2] / 16384.0f;
	q[3] = (float)last_bno_teleplot.quat[3] / 16384.0f;

	const float norm = sqrtf((q[0] * q[0]) + (q[1] * q[1]) + (q[2] * q[2]) + (q[3] * q[3]));
	if (norm < 0.0001f) return false;

	q[0] /= norm;
	q[1] /= norm;
	q[2] /= norm;
	q[3] /= norm;
	return true;
}

static void push_swing_sample(float sample) {
	if (swing_hist_count < SWING_AVG_WINDOW) {
		swing_history[swing_hist_count++] = sample;
		swing_hist_sum += sample;
		return;
	}

	swing_hist_sum -= swing_history[swing_hist_index];
	swing_history[swing_hist_index] = sample;
	swing_hist_sum += sample;
	swing_hist_index = (uint8_t)((swing_hist_index + 1U) % SWING_AVG_WINDOW);
}

static float get_swing_average(void) {
	if (swing_hist_count == 0U) return 0.0f;
	return swing_hist_sum / (float)swing_hist_count;
}

static void render_charge(void) {
	if (!led_output_ready()) return;

	uint8_t color_index_snapshot = 0;
	float charge_snapshot = 0.0f;
	taskENTER_CRITICAL(&glow_state_lock);
	color_index_snapshot = glow_color_index;
	charge_snapshot = charge;
	taskEXIT_CRITICAL(&glow_state_lock);

	const glow_color_t c = glow_colors[color_index_snapshot % (uint8_t)(sizeof(glow_colors) / sizeof(glow_colors[0]))];
	const float brightness = clamp01(charge_snapshot);
	const uint8_t r = (uint8_t)((float)c.r * brightness);
	const uint8_t g = (uint8_t)((float)c.g * brightness);
	const uint8_t b = (uint8_t)((float)c.b * brightness);
	const uint8_t w = (uint8_t)((float)c.w * brightness);
	led_output_set_all_rgbw(r, g, b, w);
}

static void handle_button_events(void) {
	button_event_t event;
	while (button_task_take_event(&event, 0)) {
		if (event.type == BUTTON_EVENT_SHORT_PRESS) {
			glowstick_mode_next_color();
			glowstick_mode_charge_full();
		} else if (event.type == BUTTON_EVENT_LONG_PRESS) {
			// Deep sleep intentionally disabled for now.
			ESP_LOGI(TAG, "Long press received (sleep disabled)");
		}
	}
}



void glowstick_task() {
    while (1) {
	handle_button_events();

	if (!bno_ready) {
		const uint32_t now_retry = now_ms();
		if (last_bno_retry_ms == 0 || (now_retry - last_bno_retry_ms) > 200) {
			last_bno_retry_ms = now_retry;
			bno_ready = init_bno();
		}
	} else {
		const uint32_t now_sample = now_ms();
		if ((now_sample - last_bno_sample_ms) >= BNO_SAMPLE_PERIOD_MS) {
			last_bno_sample_ms = now_sample;
			print_bno_status();
		}
	}

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

	float frame_swing_mag = 0.0f;
	float quat[4] = {0.0f};
	if (get_current_quat(quat)) {
		if (prev_quat_valid) {
			float dot = (quat[0] * prev_quat[0]) + (quat[1] * prev_quat[1]) + (quat[2] * prev_quat[2]) + (quat[3] * prev_quat[3]);
			dot = fabsf(dot);
			if (dot > 1.0f) dot = 1.0f;

			const float delta_angle = 2.0f * acosf(dot);
			frame_swing_mag = delta_angle;
		}

		prev_quat[0] = quat[0];
		prev_quat[1] = quat[1];
		prev_quat[2] = quat[2];
		prev_quat[3] = quat[3];
		prev_quat_valid = true;
	} else {
		prev_quat_valid = false;
	}

	// Low-frequency envelope: suppress frame spikes and keep sustained swinging.
	const float alpha = (float)dt_ms / (LOW_FREQ_ENVELOPE_TAU_MS + (float)dt_ms);
	low_freq_envelope += alpha * (frame_swing_mag - low_freq_envelope);
	push_swing_sample(low_freq_envelope);

	// Always decay toward off when idle, then add charge from sustained swing.
	taskENTER_CRITICAL(&glow_state_lock);
	charge -= (float)dt_ms / (float)TIME_TO_FADE_MS;

	const float swing_avg = get_swing_average();
	if (swing_avg > SHAKE_POWER_THRESHOLD) {
		float norm = (swing_avg - SHAKE_POWER_THRESHOLD) / (SWING_MAG_SATURATION - SHAKE_POWER_THRESHOLD);
		if (norm < 0.0f) norm = 0.0f;
		if (norm > 1.0f) norm = 1.0f;
		float add = norm * ((float)dt_ms / (float)CHARGE_TIME_MS);
		if (add > MAX_CHARGE_STEP_PER_UPDATE) add = MAX_CHARGE_STEP_PER_UPDATE;
		charge += add;
	}

	charge = clamp01(charge);
	taskEXIT_CRITICAL(&glow_state_lock);
	render_charge();
    vTaskDelay(pdMS_TO_TICKS(10));
}
}

void glowstick_mode_init(void) {
	swing_hist_count = 0;
	swing_hist_index = 0;
	swing_hist_sum = 0.0f;
	prev_quat_valid = false;
	last_mode_update_ms = 0;
	last_bno_retry_ms = 0;
	last_bno_sample_ms = 0;
	low_freq_envelope = 0.0f;
	charge = 1.0f;
	glow_color_index = 0;
	led_output_init();
	render_charge();
	ESP_LOGI(TAG, "LED mode: GLOWSTICK (%s)", glow_colors[glow_color_index].name);

    xTaskCreatePinnedToCore((TaskFunction_t)glowstick_task, "glowstick_task", 4096, NULL, 5, &glowstick_task_handle, tskNO_AFFINITY);
}

void glowstick_mode_next_color(void) {
	taskENTER_CRITICAL(&glow_state_lock);
	glow_color_index = (uint8_t)((glow_color_index + 1U) % (uint8_t)(sizeof(glow_colors) / sizeof(glow_colors[0])));
	taskEXIT_CRITICAL(&glow_state_lock);
	ESP_LOGI(TAG, "Glow color: %s", glow_colors[glow_color_index].name);
	render_charge();
}

void glowstick_mode_charge_full(void) {
	taskENTER_CRITICAL(&glow_state_lock);
	charge = 1.0f;
	taskEXIT_CRITICAL(&glow_state_lock);
	render_charge();
}