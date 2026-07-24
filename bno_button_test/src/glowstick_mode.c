#include "glowstick_mode.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include "driver/gpio.h"
#include "esp_random.h"
#include "esp_sleep.h"
#include "esp_timer.h"

#include "esp_log.h"
#include "button_task.h"
#include "led_output.h"

#define GLOW_GREEN_MAX 255U
#define SWING_AVG_WINDOW 48
#define CHARGE_TIME_MS 1200
#define QUAT_ENVELOPE_TAU_MS 350.0f
#define LINACC_ENVELOPE_TAU_MS 250.0f
#define SWING_MAG_SATURATION 0.95f
#define MAX_CHARGE_STEP_PER_UPDATE 0.010f
#define BUTTON_WAKE_GPIO GPIO_NUM_0
#define OVERCHARGE_TRIGGER_MS 10000U
#define OVERCHARGE_INTENSE_THRESHOLD 0.70f
#define OVERCHARGE_STOP_THRESHOLD 0.25f
#define OVERCHARGE_STOP_HYSTERESIS_MS 600U
#define OVERCHARGE_STROBE_PERIOD_MS 200U
#define OVERCHARGE_STROBE_ON_MS 30U

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
static float quat_envelope = 0.0f;
static float linacc_envelope = 0.0f;
static uint32_t last_bno_retry_ms = 0;
static uint32_t last_bno_sample_ms = 0;
static uint32_t sustained_shake_ms = 0;
static uint32_t calm_ms = 0;
static bool overcharge_active = false;
static uint32_t last_overcharge_blink_ms = 0;
static uint16_t overcharge_mask = 0;

static void render_charge(void);

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
	led_output_set_all_rgbw(0, 0, 0, 0);

	ESP_ERROR_CHECK(esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL));
	ESP_ERROR_CHECK(esp_sleep_enable_ext1_wakeup_io(1ULL << BUTTON_WAKE_GPIO, ESP_EXT1_WAKEUP_ANY_HIGH));
	vTaskDelay(pdMS_TO_TICKS(100));
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

static float get_current_linacc_mag(void) {
	if (!bno_ready) return 0.0f;
	const float x = (float)last_bno_teleplot.linacc[0] / 100.0f;
	const float y = (float)last_bno_teleplot.linacc[1] / 100.0f;
	const float z = (float)last_bno_teleplot.linacc[2] / 100.0f;
	return sqrtf((x * x) + (y * y) + (z * z));
}

static void render_overcharge(uint32_t now) {
	const uint32_t elapsed = now - last_overcharge_blink_ms;
	if (elapsed >= OVERCHARGE_STROBE_PERIOD_MS) {
		last_overcharge_blink_ms = now;
		overcharge_mask = 0;
		for (int i = 0; i < 9; i++) {
			if ((esp_random() & 0x1U) != 0U) {
				overcharge_mask |= (uint16_t)(1U << i);
			}
		}
		if (overcharge_mask == 0U) {
			overcharge_mask = (uint16_t)(1U << (esp_random() % 9U));
		}
	}

	if ((now - last_overcharge_blink_ms) <= OVERCHARGE_STROBE_ON_MS) {
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
		const uint8_t w = (uint8_t)(120.0f * brightness);
		led_output_set_base_rgb_with_white_mask(r, g, b, overcharge_mask, w);
	} else {
		render_charge();
	}
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

	// Blend orientation swing with linear acceleration so only stronger shaking charges.
	const float quat_alpha = (float)dt_ms / (QUAT_ENVELOPE_TAU_MS + (float)dt_ms);
	quat_envelope += quat_alpha * (frame_swing_mag - quat_envelope);
	const float linacc_mag = get_current_linacc_mag();
	const float linacc_alpha = (float)dt_ms / (LINACC_ENVELOPE_TAU_MS + (float)dt_ms);
	linacc_envelope += linacc_alpha * (linacc_mag - linacc_envelope);

	const float quat_norm = clamp01((quat_envelope - 0.008f) / (0.060f - 0.008f));
	const float linacc_norm = clamp01((linacc_envelope - 0.40f) / (4.00f - 0.40f));
	const float shake_intensity = clamp01((quat_norm * 0.55f) + (linacc_norm * 0.45f));
	push_swing_sample(shake_intensity);

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

	if (swing_avg >= OVERCHARGE_INTENSE_THRESHOLD) {
		sustained_shake_ms += dt_ms;
		calm_ms = 0;
		if (sustained_shake_ms >= OVERCHARGE_TRIGGER_MS) {
			overcharge_active = true;
		}
	} else {
		sustained_shake_ms = 0;
		if (overcharge_active) {
			if (swing_avg <= OVERCHARGE_STOP_THRESHOLD) {
				calm_ms += dt_ms;
				if (calm_ms >= OVERCHARGE_STOP_HYSTERESIS_MS) {
					overcharge_active = false;
					calm_ms = 0;
				}
			} else {
				calm_ms = 0;
			}
		}
	}

	if (overcharge_active) {
		render_overcharge(now);
	} else {
		render_charge();
	}
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
	quat_envelope = 0.0f;
	linacc_envelope = 0.0f;
	sustained_shake_ms = 0;
	calm_ms = 0;
	overcharge_active = false;
	last_overcharge_blink_ms = 0;
	overcharge_mask = 0;
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