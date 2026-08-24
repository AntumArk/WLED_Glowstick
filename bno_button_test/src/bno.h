#pragma once

#include <stdint.h>
#include <stdbool.h>

#define I2C_SDA_GPIO 22
#define I2C_SCL_GPIO 23
#define I2C_FREQ_HZ          400000 // 100kHz Standard Mode

#define BNO_ADDR_PRIMARY 0x29
#define BNO_ADDR_SECONDARY 0x28

#define BNO_REG_CHIP_ID 0x00
#define BNO_REG_SYS_CLK_STATUS 0x38
#define BNO_REG_SYS_STATUS 0x39
#define BNO_REG_SYS_ERR 0x3A
#define BNO_REG_OPR_MODE 0x3D
#define BNO_REG_PWR_MODE 0x3E
#define BNO_REG_SYS_TRIGGER 0x3F
#define BNO_REG_PAGE_ID 0x07
#define BNO_REG_CALIB_STAT 0x35
#define BNO_REG_ST_RESULT 0x36 /* self-test result, set once at power-on: bit0=MCU bit1=gyro bit2=accel bit3=mag, 1=pass */
#define BNO_REG_MAG_DATA 0x0E
#define BNO_REG_LINACC_DATA 0x28
#define BNO_REG_GRAVITY_DATA 0x2E
#define BNO_REG_GYRO_DATA 0x14
#define BNO_REG_QUATERNION_DATA 0x20
#define BNO_REG_TEMP 0x34

#define BNO_MODE_CONFIG 0x00
#define BNO_MODE_MAGONLY 0x02
#define BNO_PWR_MODE_NORMAL 0x00
#define BNO_PWR_MODE_LOW 0x01
#define BNO_MODE_SUSPEND 0x02
#define BNO_MODE_NDOF 0x0B /* NDOF_FMC_OFF: 9-axis fusion without fast magnetometer calibration */
#define BNO_SYS_TRIGGER_RST_SYS 0x20
#define BNO_SYS_TRIGGER_CLK_SEL 0x80
#define BNO_USE_EXTERNAL_CRYSTAL 0
#define BNO_SAMPLE_PERIOD_MS 10


extern uint32_t last_bno_ms;
extern bool bno_ready;

bool print_bno_status(void);
bool init_bno(void);
void bno_set_sleeping(bool sleeping);
bool bno_suspend(void);
bool bno_resume(void);

typedef struct bno_readings_t {
  int16_t linacc[3];
  int16_t mag[3];
  int16_t gravity[3];
  int16_t gyro[3];
  int16_t quat[4];
  uint8_t calib;
  uint8_t op_mode;
  uint8_t sys_status;
  uint8_t sys_error;
  int16_t mag_probe[3];
  uint8_t selftest; /* BNO_REG_ST_RESULT snapshot: bit0=MCU bit1=gyro bit2=accel bit3=mag, 1=pass */
  int8_t temp_c; /* onboard BNO055 die temperature, degrees Celsius (register 0x34) */
} bno_readings_t;

extern bno_readings_t last_bno_teleplot;