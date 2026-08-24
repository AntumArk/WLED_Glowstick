#pragma once

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"


#define BATTERY_GPIO GPIO_NUM_1
#define BATTERY_DIVIDER 2.0f
#define BATTERY_VMIN 3.40f
#define BATTERY_VMAX 4.20f


void start_battery_task(void);

/* Last-measured pack voltage (volts) and estimated charge percent (0-100),
 * updated every ~10s by battery_task(); 0 before the first reading or if
 * the ADC never initialized. Safe to call from any task - these are plain
 * floats updated atomically enough for a slow-changing display value. */
float battery_get_voltage(void);
float battery_get_percent(void);

