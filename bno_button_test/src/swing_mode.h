#pragma once

#include <stdint.h>

void swing_mode_reset(void);
void swing_mode_handle_peak(uint32_t now_ms);
void swing_mode_render(uint32_t now_ms);
void swing_mode_task(void);
void swing_mode_init(void);
