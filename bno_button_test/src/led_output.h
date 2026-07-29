#pragma once

#include <stdbool.h>
#include <stdint.h>

#define LED_OUTPUT_COUNT 9

bool led_output_init(void);
bool led_output_ready(void);
void led_output_set_pixel_rgbw(uint8_t pixel, uint8_t r, uint8_t g, uint8_t b, uint8_t w);
void led_output_show(void);
void led_output_set_all_rgbw(uint8_t r, uint8_t g, uint8_t b, uint8_t w);
void led_output_set_all_rgb(uint8_t r, uint8_t g, uint8_t b);
