#include "glowstick_mode.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include "driver/gpio.h"

#include "esp_timer.h"

#include "battery.h"
#include "bno.h"
#include "button_task.h"
#include "led_output.h"
#include "osc.h"
#include "osc_config.h"
#include "swing_mode.h"
#include "wifi_manager.h"
#include "state_machine.h"
#include "zinc_time.h"

typedef struct {
	uint8_t r;
	uint8_t g;
	uint8_t b;
	uint8_t w;
} glow_color_t;

TaskHandle_t glowstick_task_handle = NULL;

static const glow_color_t glow_colors[] = {
	{0, 255, 0, 0},
	{180, 0, 255, 0},
	{140, 255, 0, 0},
	{0, 0, 255, 0},
	{255, 0, 0, 0},
	{0, 0, 0, 255},
	{255, 255, 255, 255},
};

static portMUX_TYPE glow_state_lock = portMUX_INITIALIZER_UNLOCKED;

static uint32_t last_mode_update_ms = 0;
static uint32_t last_swing_peak_ms = 0;


static bool swing_armed = true;

static void render_charge(void);
static void render_glow_ball(const glow_color_t *color);

static float glow_charge = 0.0f; // overall brightness envelope; ramps up while shaking, fades to black when still
static float row_level[TOTAL_ROWS] = {0.0f}; // gravity-driven shape of where the glow concentrates
static float smoothed_settle = 0.5f; // 0 = MIN end down, 1 = MAX end down

// y position of each of the 4 LED rows along the tube axis.
static const float row_y[TOTAL_ROWS] = {
	BALL_TUBE_MIN_POS_M, 0.0542f, 0.1058f, BALL_TUBE_MAX_POS_M,
};

static float clamp01(float value) {
	if (value < 0.0f) return 0.0f;
	if (value > 1.0f) return 1.0f;
	return value;
}

static float clampf(float value, float lo, float hi) {
	if (value < lo) return lo;
	if (value > hi) return hi;
	return value;
}

static void update_glow_levels(const float gravity_ms2[3], const float linear_acceleration_ms2[3],
	float dt_seconds) {
	const float g_y_norm = clampf(gravity_ms2[1] / 9.8f, -1.0f, 1.0f);
	const float target_settle = 0.5f + 0.5f * g_y_norm;
	smoothed_settle += (target_settle - smoothed_settle) * clamp01(dt_seconds / GRAVITY_SETTLE_TIME_CONSTANT_S);

	const float tube_length_m = BALL_TUBE_MAX_POS_M - BALL_TUBE_MIN_POS_M;
	for (int row = 0; row < TOTAL_ROWS; row++) {
		const float normalized_pos = (row_y[row] - BALL_TUBE_MIN_POS_M) / tube_length_m;
		const float distance = fabsf(normalized_pos - smoothed_settle);
		row_level[row] = clamp01(1.0f - distance / GRAVITY_GLOW_SPREAD);
	}

	const float linear_acceleration_magnitude = sqrtf(
		linear_acceleration_ms2[0] * linear_acceleration_ms2[0] +
		linear_acceleration_ms2[1] * linear_acceleration_ms2[1] +
		linear_acceleration_ms2[2] * linear_acceleration_ms2[2]);
	const float shake_gain = clamp01((linear_acceleration_magnitude - SWING_PEAK_THRESHOLD_MS2) / SHAKE_ENERGY_RANGE_MS2);
	glow_charge = clamp01(glow_charge + shake_gain * SHAKE_CHARGE_GAIN_PER_SECOND * dt_seconds - CHARGE_FADE_PER_SECOND * dt_seconds);
}


// Gravity vector reflects tube orientation (unlike linacc, which excludes gravity)
static bool get_current_gravity(float gravity_ms2[3]) {
	if (!bno_ready) return false;

	gravity_ms2[0] = (float)last_bno_teleplot.gravity[0] / 100.0f;
	gravity_ms2[1] = (float)last_bno_teleplot.gravity[1] / 100.0f;
	gravity_ms2[2] = (float)last_bno_teleplot.gravity[2] / 100.0f;
	return true;
}

static void render_charge(void) {
	if (!led_output_ready()) return;

	uint8_t color_index_snapshot = 0;
	taskENTER_CRITICAL(&glow_state_lock);
	color_index_snapshot = device_state;
	taskEXIT_CRITICAL(&glow_state_lock);

	if (color_index_snapshot >= DEVICE_STATE_SWING_MODE) {
		swing_mode_render(now_ms());
		return;
	}

	const glow_color_t c = glow_colors[color_index_snapshot];
	render_glow_ball(&c);
}


static void render_glow_ball(const glow_color_t *color) {
	float charge_snapshot;
	float row_level_snapshot[TOTAL_ROWS];
	taskENTER_CRITICAL(&glow_state_lock);
	charge_snapshot = glow_charge;
	for (uint8_t row = 0; row < TOTAL_ROWS; row++) {
		row_level_snapshot[row] = row_level[row];
	}
	taskEXIT_CRITICAL(&glow_state_lock);

	for (uint8_t led = 0; led < NUM_LEDS; led++) {
		const uint8_t row = (led <= 3U) ? led : (uint8_t)(7U - led);
		const float brightness = row_level_snapshot[row] * charge_snapshot;
		const float shade = 0.82f + 0.06f * (float)(led % 4U);
		float red = (float)color->r * shade;
		float green = (float)color->g * shade;
		float blue = (float)color->b * shade;
		if (color->g > color->r && color->g > color->b) {
			red += (float)color->g * (0.03f * (float)(led % 3U));
			blue += (float)color->g * (0.025f * (float)((led + 1U) % 3U));
		}
		led_output_set_pixel_rgbw(led, (uint8_t)(red * brightness), (uint8_t)(green * brightness),
			(uint8_t)(blue * brightness), (uint8_t)((float)color->w * shade * brightness));
	}
	led_output_set_pixel_rgbw(NUM_LEDS, 0, 0, 0, 0);
	led_output_show();
}

static void handle_button_events(void) {
	button_event_t event;
	while (button_task_take_event(&event, 0)) {
		if (event.type != BUTTON_EVENT_SHORT_PRESS) return;
		next_device_state();
		if(device_state<=DEVICE_STATE_GLOWSTICK_BLAST)
		{
			glowstick_mode_charge_full();
		} 
	}
}

void glowstick_task() {
	TickType_t next_update = xTaskGetTickCount();
    while (1) {
	handle_button_events();

    const uint32_t now = now_ms();
	if (last_mode_update_ms == 0U) {
		last_mode_update_ms = now;
		xTaskDelayUntil(&next_update, pdMS_TO_TICKS(10));
		continue;
	}

	uint32_t dt_ms = now - last_mode_update_ms;
	if (dt_ms == 0U) continue;
	if (dt_ms > 100U) dt_ms = 100U;
	last_mode_update_ms = now;

	float linear_acceleration_ms2[3] = {0.0f};
	const bool have_linear_acceleration = get_current_linear_acceleration(linear_acceleration_ms2);
	const float linear_acceleration_magnitude = sqrtf(
		linear_acceleration_ms2[0] * linear_acceleration_ms2[0] +
		linear_acceleration_ms2[1] * linear_acceleration_ms2[1] +
		linear_acceleration_ms2[2] * linear_acceleration_ms2[2]);
	float gravity_ms2[3] = {0.0f};
	const bool have_gravity = get_current_gravity(gravity_ms2);
	taskENTER_CRITICAL(&glow_state_lock);
	if (device_state >= DEVICE_STATE_SWING_MODE) {
		if (have_linear_acceleration && linear_acceleration_magnitude < SWING_PEAK_THRESHOLD_MS2) {
			swing_armed = true;
		}
		if (have_linear_acceleration && swing_armed && linear_acceleration_magnitude >= SWING_PEAK_THRESHOLD_MS2 &&
			(now - last_swing_peak_ms) >= SWING_PEAK_COOLDOWN_MS) {
			swing_mode_handle_peak(now);
			last_swing_peak_ms = now;
			swing_armed = false;
		}
	} else if (have_gravity) {
		update_glow_levels(gravity_ms2, linear_acceleration_ms2, (float)dt_ms / 1000.0f);
	}
	taskEXIT_CRITICAL(&glow_state_lock);

	render_charge();
	xTaskDelayUntil(&next_update, pdMS_TO_TICKS(10));
}
}

void glowstick_mode_init(void) {
	last_mode_update_ms = 0;
	last_swing_peak_ms = 0;
	swing_armed = true;
	smoothed_settle = 0.5f;
	glow_charge = 0.0f;
	for (int row = 0; row < TOTAL_ROWS; row++) {
		row_level[row] = 0.0f;
	}
	swing_mode_reset();
	device_state = 0;
	led_output_init();
	bno_ready = init_bno();
	render_charge();

    xTaskCreatePinnedToCore((TaskFunction_t)glowstick_task, "glowstick_task", 4096, NULL, 5, &glowstick_task_handle, tskNO_AFFINITY);
}

void glowstick_mode_charge_full(void) {
	taskENTER_CRITICAL(&glow_state_lock);
	glow_charge = 1.0f;
	for (int row = 0; row < TOTAL_ROWS; row++) {
		row_level[row] = 1.0f;
	}
	taskEXIT_CRITICAL(&glow_state_lock);
	render_charge();
}