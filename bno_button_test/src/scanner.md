#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"

#define I2C_SDA_IO           22    // Custom SDA Pin
#define I2C_SCL_IO           23    // Custom SCL Pin
#define I2C_FREQ_HZ          400000 // 100kHz Standard Mode

static const char *TAG = "i2c_scanner";

void app_main(void)
{
    ESP_LOGI(TAG, "Initializing I2C Master on ESP32-C6...");

    // 1. Configure the I2C Master Bus
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = I2C_SCL_IO,
        .sda_io_num = I2C_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true, // Enables weak internal pull-ups
    };

    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));

    ESP_LOGI(TAG, "Starting I2C bus scan...");

    while (1) {
        int devices_found = 0;
        printf("\n     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");
        printf("00:          ");

        // Loop through all valid 7-bit I2C addresses (0x03 to 0x77)
        for (uint8_t address = 0x03; address < 0x78; address++) {
            if (address % 16 == 0) {
                printf("\n%02x: ", address);
            }

            // Probe the address by checking if it responds to a basic contact test
            esp_err_t ret = i2c_master_probe(bus_handle, address, 200); // 200ms timeout

            if (ret == ESP_OK) {
                printf("%02x ", address);
                devices_found++;
            } else {
                printf("-- ");
            }
        }
        
        printf("\n\nScan finished. Found %d device(s).\n", devices_found);
        vTaskDelay(pdMS_TO_TICKS(5000)); // Repeat scan every 5 seconds
    }
}