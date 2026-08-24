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
#define SWING_PEAK_THRESHOLD_MS2 1.2f
#define SWING_PEAK_COOLDOWN_MS 140U
#define STATUS_STREAM_PERIOD_MS 2000U
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
static uint32_t last_imu_stream_ms = 0;
static uint32_t last_status_stream_ms = 0;
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
	const uint8_t color_count = (uint8_t)(sizeof(glow_colors) / sizeof(glow_colors[0]));
	if (have_linear_acceleration) {
		stream_imu_over_osc(now, linear_acceleration_ms2);
	}
	stream_status_over_osc(now);

	/* Swing/hit detection (and the resulting /glowstick/hit OSC event) runs
	 * unconditionally, regardless of which LED animation mode is currently
	 * selected. Previously this was gated behind the last "PEAK FLASH" mode
	 * in the color cycle, meaning hits were never sent unless the user had
	 * pressed the button enough times to reach that hidden mode. The LED
	 * flash itself is still only rendered while in PEAK FLASH mode (see
	 * render_charge()); other modes simply ignore the resulting flash state.
	 *
	 * swing_armed/last_swing_peak_ms are only ever touched by this task (the
	 * button-triggered glowstick_mode_* setters run from within this same
	 * task via handle_button_events()), so no locking is needed here at all.
	 * Importantly, swing_mode_handle_peak() must NOT be called from inside a
	 * taskENTER_CRITICAL/EXIT_CRITICAL section: it logs (ESP_LOGI) and sends
	 * a UDP/OSC packet through lwIP, both of which can block/yield - doing so
	 * with interrupts disabled hung the task as soon as a real hit fired. */
	bool trigger_peak = false;
	if (have_linear_acceleration && linear_acceleration_magnitude < SWING_PEAK_THRESHOLD_MS2) {
		swing_armed = true;
	}
	if (have_linear_acceleration && swing_armed && linear_acceleration_magnitude >= SWING_PEAK_THRESHOLD_MS2 &&
		(now - last_swing_peak_ms) >= SWING_PEAK_COOLDOWN_MS) {
		trigger_peak = true;
		last_swing_peak_ms = now;
		swing_armed = false;
	}

	taskENTER_CRITICAL(&glow_state_lock);
	if (glow_color_index != color_count && have_linear_acceleration) {
		update_bouncing_ball((float)dt_ms / 1000.0f, linear_acceleration_ms2);
	}
	taskEXIT_CRITICAL(&glow_state_lock);

	if (trigger_peak) {
		swing_mode_handle_peak(now);
	}
	render_charge();
	xTaskDelayUntil(&next_update, pdMS_TO_TICKS(10));
}
}

void glowstick_mode_init(void) {
	last_mode_update_ms = 0;
	last_swing_peak_ms = 0;
	last_imu_stream_ms = 0;
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