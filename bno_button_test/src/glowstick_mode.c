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
#define FLUID_ACCELERATION_SCALE 1.20f
#define BALL_TUBE_MIN_POS_M 0.0025f
#define BALL_TUBE_MAX_POS_M 0.1575f
#define FLUID_MASS_KG 0.025f
#define TUBE_INNER_RADIUS_M 0.014f
#define FLUID_PARTICLE_COUNT 24
#define FLUID_PARTICLE_RADIUS_M 0.0025f
#define FLUID_REST_DISTANCE_M 0.0065f
#define FLUID_INTERACTION_RADIUS_M 0.014f
#define FLUID_PRESSURE_STIFFNESS 0.45f
#define FLUID_VISCOSITY_PER_SECOND 3.5f
#define FLUID_WALL_RESTITUTION 0.22f
#define FLUID_WALL_FRICTION 0.88f
#define FLUID_CONTACT_SPEED_MPS 0.035f
#define FLUID_BOUNCE_SPEED_MPS 0.12f
#define FLUID_CHARGE_SPEED_MPS 0.08f
#define FLUID_SHEAR_CHARGE_SPEED_MPS 0.06f
#define FLUID_SHEAR_CHARGE_SCALE 0.45f
#define LED_INFLUENCE_RADIUS_M 0.040f
#define IMPACT_ENERGY_REFERENCE_J 0.00020f
#define CHARGE_FADE_PER_SECOND 0.16f

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

static void render_charge(void);
static void render_glow_ball(const glow_color_t *color);

#define NUM_LEDS 8
#define TOTAL_ROWS 4  // 4 rows of 2 parallel LEDs

typedef struct {
    float x;
    float y;
    float z;
} vec3_t;

typedef struct {
	vec3_t position;
	vec3_t velocity;
} fluid_particle_t;

static fluid_particle_t fluid[FLUID_PARTICLE_COUNT];
static float led_charge[NUM_LEDS] = {0.0f};

// Physical locations of the two LED columns on the tube wall.
static const vec3_t led_positions[NUM_LEDS] = {
	{-TUBE_INNER_RADIUS_M, BALL_TUBE_MIN_POS_M, 0.0f},
	{-TUBE_INNER_RADIUS_M, 0.0542f, 0.0f},
	{-TUBE_INNER_RADIUS_M, 0.1058f, 0.0f},
	{-TUBE_INNER_RADIUS_M, BALL_TUBE_MAX_POS_M, 0.0f},
	{ TUBE_INNER_RADIUS_M, BALL_TUBE_MAX_POS_M, 0.0f},
	{ TUBE_INNER_RADIUS_M, 0.1058f, 0.0f},
	{ TUBE_INNER_RADIUS_M, 0.0542f, 0.0f},
	{ TUBE_INNER_RADIUS_M, BALL_TUBE_MIN_POS_M, 0.0f},
};

static float clamp01(float value) {
	if (value < 0.0f) return 0.0f;
	if (value > 1.0f) return 1.0f;
	return value;
}

static void deposit_impact_charge(const vec3_t *impact_position, float normal_speed) {
	const float particle_mass = FLUID_MASS_KG / (float)FLUID_PARTICLE_COUNT;
	const float energy = 0.5f * particle_mass * normal_speed * normal_speed;
	const float impact_strength = clamp01(energy / IMPACT_ENERGY_REFERENCE_J);

	if (impact_strength == 0.0f) return;
	for (int led = 0; led < NUM_LEDS; led++) {
		const float dx = impact_position->x - led_positions[led].x;
		const float dy = impact_position->y - led_positions[led].y;
		const float dz = impact_position->z - led_positions[led].z;
		const float distance = sqrtf(dx * dx + dy * dy + dz * dz);
		const float influence = clamp01(1.0f - distance / LED_INFLUENCE_RADIUS_M);
		led_charge[led] = clamp01(led_charge[led] + impact_strength * influence * influence);
	}
}

static void resolve_particle_collision(fluid_particle_t *particle) {
	const float end_min = BALL_TUBE_MIN_POS_M + FLUID_PARTICLE_RADIUS_M;
	const float end_max = BALL_TUBE_MAX_POS_M - FLUID_PARTICLE_RADIUS_M;
	if (particle->position.y < end_min || particle->position.y > end_max) {
		const float normal_y = particle->position.y < end_min ? -1.0f : 1.0f;
		const float normal_speed = particle->velocity.y * normal_y;
		particle->position.y = normal_y < 0.0f ? end_min : end_max;
		if (normal_speed > FLUID_BOUNCE_SPEED_MPS) {
			if (normal_speed > FLUID_CHARGE_SPEED_MPS) {
				deposit_impact_charge(&particle->position, normal_speed);
			}
			particle->velocity.y -= (1.0f + FLUID_WALL_RESTITUTION) * normal_speed * normal_y;
		} else if (normal_speed > 0.0f) {
			particle->velocity.y = 0.0f;
		}
		particle->velocity.x *= FLUID_WALL_FRICTION;
		particle->velocity.z *= FLUID_WALL_FRICTION;
	}

	const float max_radius = TUBE_INNER_RADIUS_M - FLUID_PARTICLE_RADIUS_M;
	const float radial_distance = sqrtf(particle->position.x * particle->position.x + particle->position.z * particle->position.z);
	if (radial_distance > max_radius) {
		const float normal_x = particle->position.x / radial_distance;
		const float normal_z = particle->position.z / radial_distance;
		const float normal_speed = particle->velocity.x * normal_x + particle->velocity.z * normal_z;
		const float tangent_x = particle->velocity.x - normal_speed * normal_x;
		const float tangent_z = particle->velocity.z - normal_speed * normal_z;
		const float tangential_speed = sqrtf(tangent_x * tangent_x + tangent_z * tangent_z +
			particle->velocity.y * particle->velocity.y);
		particle->position.x = normal_x * max_radius;
		particle->position.z = normal_z * max_radius;
		if (normal_speed > FLUID_BOUNCE_SPEED_MPS) {
			if (normal_speed > FLUID_CHARGE_SPEED_MPS) {
				deposit_impact_charge(&particle->position, normal_speed);
			}
			particle->velocity.x -= (1.0f + FLUID_WALL_RESTITUTION) * normal_speed * normal_x;
			particle->velocity.z -= (1.0f + FLUID_WALL_RESTITUTION) * normal_speed * normal_z;
		} else if (normal_speed > 0.0f) {
			particle->velocity.x -= normal_speed * normal_x;
			particle->velocity.z -= normal_speed * normal_z;
		}
		if (tangential_speed > FLUID_SHEAR_CHARGE_SPEED_MPS && normal_speed > FLUID_CONTACT_SPEED_MPS) {
			deposit_impact_charge(&particle->position, tangential_speed * FLUID_SHEAR_CHARGE_SCALE);
		}
		particle->velocity.y *= FLUID_WALL_FRICTION;
	}
}

static void solve_fluid_constraints(float dt_seconds) {
	for (int first = 0; first < FLUID_PARTICLE_COUNT; first++) {
		for (int second = first + 1; second < FLUID_PARTICLE_COUNT; second++) {
			float dx = fluid[second].position.x - fluid[first].position.x;
			float dy = fluid[second].position.y - fluid[first].position.y;
			float dz = fluid[second].position.z - fluid[first].position.z;
			const float distance = sqrtf(dx * dx + dy * dy + dz * dz);
			if (distance <= 0.0001f || distance >= FLUID_INTERACTION_RADIUS_M) continue;

			dx /= distance;
			dy /= distance;
			dz /= distance;
			if (distance < FLUID_REST_DISTANCE_M) {
				const float correction = (FLUID_REST_DISTANCE_M - distance) * FLUID_PRESSURE_STIFFNESS * 0.5f;
				fluid[first].position.x -= dx * correction;
				fluid[first].position.y -= dy * correction;
				fluid[first].position.z -= dz * correction;
				fluid[second].position.x += dx * correction;
				fluid[second].position.y += dy * correction;
				fluid[second].position.z += dz * correction;
			}

			const float velocity_mix = (1.0f - expf(-FLUID_VISCOSITY_PER_SECOND * dt_seconds)) *
				(1.0f - distance / FLUID_INTERACTION_RADIUS_M);
			const float relative_x = fluid[second].velocity.x - fluid[first].velocity.x;
			const float relative_y = fluid[second].velocity.y - fluid[first].velocity.y;
			const float relative_z = fluid[second].velocity.z - fluid[first].velocity.z;
			fluid[first].velocity.x += relative_x * velocity_mix;
			fluid[first].velocity.y += relative_y * velocity_mix;
			fluid[first].velocity.z += relative_z * velocity_mix;
			fluid[second].velocity.x -= relative_x * velocity_mix;
			fluid[second].velocity.y -= relative_y * velocity_mix;
			fluid[second].velocity.z -= relative_z * velocity_mix;
		}
	}
}

static void update_fluid(const float gravity_ms2[3], const float linear_acceleration_ms2[3], float dt_seconds) {
	const vec3_t acceleration = {
		.x = (gravity_ms2[0] - linear_acceleration_ms2[0]) * FLUID_ACCELERATION_SCALE,
		.y = (gravity_ms2[1] - linear_acceleration_ms2[1]) * FLUID_ACCELERATION_SCALE,
		.z = (gravity_ms2[2] - linear_acceleration_ms2[2]) * FLUID_ACCELERATION_SCALE,
	};

	for (int particle = 0; particle < FLUID_PARTICLE_COUNT; particle++) {
		fluid[particle].velocity.x += acceleration.x * dt_seconds;
		fluid[particle].velocity.y += acceleration.y * dt_seconds;
		fluid[particle].velocity.z += acceleration.z * dt_seconds;
		fluid[particle].position.x += fluid[particle].velocity.x * dt_seconds;
		fluid[particle].position.y += fluid[particle].velocity.y * dt_seconds;
		fluid[particle].position.z += fluid[particle].velocity.z * dt_seconds;
		resolve_particle_collision(&fluid[particle]);
	}

	solve_fluid_constraints(dt_seconds);
	for (int particle = 0; particle < FLUID_PARTICLE_COUNT; particle++) {
		resolve_particle_collision(&fluid[particle]);
	}
	for (int led = 0; led < NUM_LEDS; led++) {
		led_charge[led] = clamp01(led_charge[led] - CHARGE_FADE_PER_SECOND * dt_seconds);
	}
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
	float charge_snapshot[NUM_LEDS];
	taskENTER_CRITICAL(&glow_state_lock);
	for (uint8_t led = 0; led < NUM_LEDS; led++) {
		charge_snapshot[led] = led_charge[led];
	}
	taskEXIT_CRITICAL(&glow_state_lock);

	for (uint8_t led = 0; led < NUM_LEDS; led++) {
		const float brightness = charge_snapshot[led];
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
	float gravity_ms2[3] = {0.0f};
	const bool have_gravity = get_current_gravity(gravity_ms2);
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
	} else if (have_gravity) {
		update_fluid(gravity_ms2, linear_acceleration_ms2, (float)dt_ms / 1000.0f);
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
	const float fluid_center_y = (BALL_TUBE_MIN_POS_M + BALL_TUBE_MAX_POS_M) / 2.0f;
	for (int particle = 0; particle < FLUID_PARTICLE_COUNT; particle++) {
		const int cross_section_slot = particle % 4;
		fluid[particle].position.x = (cross_section_slot & 1) ? 0.0015f : -0.0015f;
		fluid[particle].position.y = fluid_center_y + ((float)particle - (float)(FLUID_PARTICLE_COUNT - 1) / 2.0f) * 0.0025f;
		fluid[particle].position.z = (cross_section_slot & 2) ? 0.0015f : -0.0015f;
		fluid[particle].velocity.x = 0.0f;
		fluid[particle].velocity.y = 0.0f;
		fluid[particle].velocity.z = 0.0f;
	}
	for (int i = 0; i < NUM_LEDS; i++) {
		led_charge[i] = 0.0f;
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
	for (int i = 0; i < NUM_LEDS; i++) {
		led_charge[i] = 1.0f;
	}
	taskEXIT_CRITICAL(&glow_state_lock);
	render_charge();
}