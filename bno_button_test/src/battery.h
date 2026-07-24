#pragma once

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"


#define BATTERY_GPIO GPIO_NUM_1
#define BATTERY_DIVIDER 2.0f
#define BATTERY_VMIN 3.40f
#define BATTERY_VMAX 4.20f


void start_battery_task(void);

