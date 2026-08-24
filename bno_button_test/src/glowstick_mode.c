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
#include "battery.h"
#include "bno.h"
#include "button_task.h"
#include "led_output.h"
#include "osc.h"
#include "osc_config.h"
#include "swing_mode.h"
#include "wifi_manager.h"

#define GLOW_GREEN_MAX 255U
#define SWING_PEAK_THRESHOLD_MS2 15.0f
#define SWING_PEAK_COOLDOWN_MS 100U
#define STATUS_STREAM_PERIOD_MS 2000U
#define BUTTON_WAKE_GPIO GPIO_NUM_0
#define BALL_TUBE_MIN_POS_M 0.0025f
#define BALL_TUBE_MAX_POS_M 0.1575f
#define CHARGE_FADE_PER_SECOND 0.01f
#define GRAVITY_SETTLE_TIME_CONSTANT_S 30.35f
#define GRAVITY_GLOW_SPREAD 0.8f
#define SHAKE_CHARGE_GAIN_PER_SECOND 5.5f
#define SHAKE_ENERGY_RANGE_MS2 15.0f

#define POWER_LIMIT 1.0f // keep this. crucial for battery life

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
	{0, 0, 255, 0, "BLUE"},
	{255, 0, 0, 0, "RED"},
	{0, 0, 0, 255, "WHITE"},
	{255, 255, 255, 255, "BLAST"},
};
static volatile uint8_t glow_color_index = 0;
static portMUX_TYPE glow_state_lock = portMUX_INITIALIZER_UNLOCKED;

static uint32_t last_mode_update_ms = 0;
static uint32_t last_swing_peak_ms = 0;
static uint32_t last_imu_stream_ms = 0;
static uint32_t last_status_stream_ms = 0;
static bool swing_armed = true;

static void render_charge(void);
static void render_glow_ball(const glow_color_t *color);

#define NUM_LEDS 4
#define TOTAL_ROWS 4  // 4 rows of 2 parallel LEDs

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

static uint32_t now_ms(void) {
  return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

/* Streams raw linear acceleration plus the rest of the BNO055 pose data
 * (orientation quaternion, gyro, magnetometer, gravity vector, and sensor
 * calibration status) as OSC messages, gated by the live
 * (OSC-configurable) enable flag and rate limit - see osc_config.h for
 * the full list of addresses/argument layouts. */
static void stream_imu_over_osc(uint32_t now, const float linear_acceleration_ms2[3]) {
  const osc_config_t *cfg = osc_config_get();
  if (!cfg->stream_enabled) return;
  if ((now - last_imu_stream_ms) < cfg->stream_period_ms) return;
  last_imu_stream_ms = now;
  if (!osc_has_target()) return;

  osc_bundle_t bundle;
  osc_bundle_init(&bundle);
  if (!osc_bundle_add_floats(&bundle, "/glowstick/accel", linear_acceleration_ms2, 3)) return;

  const float quat[4] = {
      (float)last_bno_teleplot.quat[0] / 16384.0f,
      (float)last_bno_teleplot.quat[1] / 16384.0f,
      (float)last_bno_teleplot.quat[2] / 16384.0f,
      (float)last_bno_teleplot.quat[3] / 16384.0f,
  };
  if (!osc_bundle_add_floats(&bundle, "/glowstick/orientation", quat, 4)) return;

  const float gyro[3] = {
      (float)last_bno_teleplot.gyro[0] / 16.0f,
      (float)last_bno_teleplot.gyro[1] / 16.0f,
      (float)last_bno_teleplot.gyro[2] / 16.0f,
  };
  if (!osc_bundle_add_floats(&bundle, "/glowstick/gyro", gyro, 3)) return;

  const float mag[3] = {
      (float)last_bno_teleplot.mag[0] / 16.0f,
      (float)last_bno_teleplot.mag[1] / 16.0f,
      (float)last_bno_teleplot.mag[2] / 16.0f,
  };
  if (!osc_bundle_add_floats(&bundle, "/glowstick/mag", mag, 3)) return;

  const float gravity[3] = {
      (float)last_bno_teleplot.gravity[0] / 100.0f,
      (float)last_bno_teleplot.gravity[1] / 100.0f,
      (float)last_bno_teleplot.gravity[2] / 100.0f,
  };
  if (!osc_bundle_add_floats(&bundle, "/glowstick/gravity", gravity, 3)) return;

  const int32_t calib[4] = {
      (last_bno_teleplot.calib >> 6) & 0x03, /* system */
      (last_bno_teleplot.calib >> 4) & 0x03, /* gyro */
      (last_bno_teleplot.calib >> 2) & 0x03, /* accel */
      last_bno_teleplot.calib & 0x03,        /* mag */
  };
  if (!osc_bundle_add_ints(&bundle, "/glowstick/calib", calib, 4)) return;

	const int32_t ndof[3] = {
			last_bno_teleplot.op_mode,
			last_bno_teleplot.sys_status,
			last_bno_teleplot.sys_error,
	};
	if (!osc_bundle_add_ints(&bundle, "/glowstick/ndof", ndof, 3)) return;
	osc_bundle_send(&bundle);
}

/* Streams device housekeeping data (battery, BNO055 die temperature,
 * Wi-Fi RSSI) at a fixed slow rate, independent of the IMU stream
 * enable/period settings - this is cheap and useful to have on hand for
 * any receiver UI even without a full accel stream running. */
static void stream_status_over_osc(uint32_t now) {
  if ((now - last_status_stream_ms) < STATUS_STREAM_PERIOD_MS) return;
  last_status_stream_ms = now;
  if (!osc_has_target()) return;

  osc_send_floats("/glowstick/battery", (const float[]){battery_get_voltage(), battery_get_percent()}, 2);
  osc_send_float1("/glowstick/temp", (float)last_bno_teleplot.temp_c);
  const int32_t rssi = wifi_manager_get_rssi();
  osc_send_ints("/glowstick/rssi", &rssi, 1);
  /* Power-on self-test result (set once at boot, resent here so it's
   * visible without a serial connection): bit0=MCU bit1=gyro bit2=accel
   * bit3=mag, 1=pass. If bit3 (mag) is 0, the board's magnetometer
   * hardware failed self-test and /glowstick/mag will always read zero
   * regardless of firmware - a common issue on some BNO055 clone modules. */
  const int32_t selftest[4] = {
      last_bno_teleplot.selftest & 0x01,
      (last_bno_teleplot.selftest >> 1) & 0x01,
      (last_bno_teleplot.selftest >> 2) & 0x01,
      (last_bno_teleplot.selftest >> 3) & 0x01,
  };
  osc_send_ints("/glowstick/selftest", selftest, 4);

	const float mag_probe[3] = {
			(float)last_bno_teleplot.mag_probe[0] / 16.0f,
			(float)last_bno_teleplot.mag_probe[1] / 16.0f,
			(float)last_bno_teleplot.mag_probe[2] / 16.0f,
	};
	osc_send_floats("/glowstick/magprobe", mag_probe, 3);
}

static bool get_current_linear_acceleration(float linear_acceleration_ms2[3]) {
	if (!bno_ready) return false;

	linear_acceleration_ms2[0] = (float)last_bno_teleplot.linacc[0] / 100.0f;
	linear_acceleration_ms2[1] = (float)last_bno_teleplot.linacc[1] / 100.0f;
	linear_acceleration_ms2[2] = (float)last_bno_teleplot.linacc[2] / 100.0f;
	return true;
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
	color_index_snapshot = glow_color_index;
	taskEXIT_CRITICAL(&glow_state_lock);

	const uint8_t color_count = (uint8_t)(sizeof(glow_colors) / sizeof(glow_colors[0]));
	if (color_index_snapshot == color_count) {
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
	const uint8_t color_count = (uint8_t)(sizeof(glow_colors) / sizeof(glow_colors[0]));
		if (have_linear_acceleration) {
		stream_imu_over_osc(now, linear_acceleration_ms2);
	}
	stream_status_over_osc(now);
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
	last_imu_stream_ms = 0;
	swing_armed = true;
	smoothed_settle = 0.5f;
	glow_charge = 0.0f;
	for (int row = 0; row < TOTAL_ROWS; row++) {
		row_level[row] = 0.0f;
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
	glow_charge = 1.0f;
	taskEXIT_CRITICAL(&glow_state_lock);
	render_charge();
}