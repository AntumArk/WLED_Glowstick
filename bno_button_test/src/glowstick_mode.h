#pragma once

#include "bno.h"


#define TIME_TO_FADE_MS 3000
#define SHAKE_POWER_THRESHOLD 0.015f

void glowstick_mode_init(void);
void glowstick_mode_next_color(void);
void glowstick_mode_charge_full(void);