#pragma once

#include <stdbool.h>
#include <stdint.h>

bool led_output_init(void);
bool led_output_ready(void);
void led_output_set_all_rgbw(uint8_t r, uint8_t g, uint8_t b, uint8_t w);
void led_output_set_all_rgb(uint8_t r, uint8_t g, uint8_t b);
void led_output_set_mask_rgbw(uint16_t mask, uint8_t r, uint8_t g, uint8_t b, uint8_t w);
void led_output_set_base_rgb_with_white_mask(uint8_t r, uint8_t g, uint8_t b, uint16_t white_mask, uint8_t w);
