#pragma once

#include <stdint.h>
#include <stdbool.h>

#define I2C_SDA_GPIO 22
#define I2C_SCL_GPIO 23
#define I2C_FREQ_HZ          400000 // 100kHz Standard Mode

#define BNO_ADDR_PRIMARY 0x29
#define BNO_ADDR_SECONDARY 0x28

#define BNO_REG_CHIP_ID 0x00
#define BNO_REG_OPR_MODE 0x3D
#define BNO_REG_PAGE_ID 0x07
#define BNO_REG_CALIB_STAT 0x35
#define BNO_REG_LINACC_DATA 0x28
#define BNO_REG_GYRO_DATA 0x14
#define BNO_REG_QUATERNION_DATA 0x20

#define BNO_MODE_CONFIG 0x00
#define BNO_MODE_NDOF 0x0C
#define BNO_SAMPLE_PERIOD_MS 10


extern uint32_t last_bno_ms;
extern bool bno_ready;

void print_bno_status(void);
bool init_bno(void);

typedef struct bno_readings_t {
  int16_t linacc[3];
  int16_t gyro[3];
  int16_t quat[4];
  uint8_t calib;
} bno_readings_t;

extern bno_readings_t last_bno_teleplot;