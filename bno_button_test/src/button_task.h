#pragma once

#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#define BUTTON_WAKE_GPIO GPIO_NUM_0

typedef enum {
	BUTTON_EVENT_SHORT_PRESS = 0,
	BUTTON_EVENT_LONG_PRESS_READY = 1,
	BUTTON_EVENT_LONG_PRESS_RELEASE = 2,
} button_event_type_t;

typedef struct {
	button_event_type_t type;
	uint32_t ts_ms;
} button_event_t;

void start_button_task(void);
bool button_task_take_event(button_event_t *event, TickType_t wait_ticks);
void enter_deep_sleep(void);
void blink_sleep_ready(void);