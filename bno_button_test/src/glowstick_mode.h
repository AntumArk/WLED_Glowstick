#pragma once

#include "bno.h"
#define NUM_LEDS 4
#define TOTAL_ROWS 4  // 4 rows of 2 parallel LEDs
#define GLOW_GREEN_MAX 255U
#define SWING_PEAK_THRESHOLD_MS2 15.0f
#define SWING_PEAK_COOLDOWN_MS 100U


#define BALL_TUBE_MIN_POS_M 0.0025f
#define BALL_TUBE_MAX_POS_M 0.1575f
#define CHARGE_FADE_PER_SECOND 0.01f
#define GRAVITY_SETTLE_TIME_CONSTANT_S 30.35f
#define GRAVITY_GLOW_SPREAD 0.8f
#define SHAKE_CHARGE_GAIN_PER_SECOND 5.5f
#define SHAKE_ENERGY_RANGE_MS2 15.0f

#define POWER_LIMIT 1.0f // keep this. crucial for battery life

void glowstick_mode_init(void);
void glowstick_mode_next_color(void);
void glowstick_mode_charge_full(void);