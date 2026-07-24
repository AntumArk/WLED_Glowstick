#include "bno.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


static uint8_t bno_addr = BNO_ADDR_PRIMARY;
static i2c_master_bus_handle_t i2c_bus = NULL;
static i2c_master_dev_handle_t bno_dev = NULL;

uint32_t last_bno_ms = 0;
bool bno_ready = false;
bno_readings_t last_bno_teleplot = {0};

static const char *TAG = "bno_button_test";


esp_err_t i2c_write_reg(uint8_t reg, uint8_t value) {
  if (bno_dev == NULL) return ESP_ERR_INVALID_STATE;
  uint8_t data[2] = {reg, value};
  return i2c_master_transmit(bno_dev, data, sizeof(data), 100);
}

esp_err_t i2c_read_reg(uint8_t reg, uint8_t *buf, size_t len) {
  if (bno_dev == NULL) return ESP_ERR_INVALID_STATE;
  return i2c_master_transmit_receive(bno_dev, &reg, 1, buf, len, 100);
}


int16_t read_i16_le(const uint8_t *buf) {
  return (int16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
}

void teleplot_emit_float(const char *name, float value) {
  // Teleplot serial parser expects lines prefixed with '>'
  printf(">%s:%.5f\n", name, value);
}

void teleplot_emit_uint(const char *name, uint32_t value) {
  printf(">%s:%u\n", name, (unsigned int)value);
}

void emit_bno_teleplot(const uint8_t *linacc, const uint8_t *gyro, const uint8_t *quat, uint8_t calib) {
  const float lin_x = (float)read_i16_le(&linacc[0]) / 100.0f;
  const float lin_y = (float)read_i16_le(&linacc[2]) / 100.0f;
  const float lin_z = (float)read_i16_le(&linacc[4]) / 100.0f;

  const float gyro_x = (float)read_i16_le(&gyro[0]) / 16.0f;
  const float gyro_y = (float)read_i16_le(&gyro[2]) / 16.0f;
  const float gyro_z = (float)read_i16_le(&gyro[4]) / 16.0f;

  const float x = (float)read_i16_le(&quat[0]) / 16384.0f;
  const float y = (float)read_i16_le(&quat[2]) / 16384.0f;
  const float z = (float)read_i16_le(&quat[4]) / 16384.0f;
  const float w = (float)read_i16_le(&quat[6]) / 16384.0f;

  teleplot_emit_float("bno_lin_x", lin_x);
  teleplot_emit_float("bno_lin_y", lin_y);
  teleplot_emit_float("bno_lin_z", lin_z);
  teleplot_emit_float("bno_gyro_x", gyro_x);
  teleplot_emit_float("bno_gyro_y", gyro_y);
  teleplot_emit_float("bno_gyro_z", gyro_z);
  teleplot_emit_float("bno_quat_x", x);
  teleplot_emit_float("bno_quat_y", y);
  teleplot_emit_float("bno_quat_z", z);
  teleplot_emit_float("bno_quat_w", w);
  teleplot_emit_uint("bno_cal_sys", (calib >> 6) & 0x03);
  teleplot_emit_uint("bno_cal_gyro", (calib >> 4) & 0x03);
  teleplot_emit_uint("bno_cal_accel", (calib >> 2) & 0x03);
  teleplot_emit_uint("bno_cal_mag", calib & 0x03);
}


void print_bno_status(void) {
  uint8_t calib = 0;
  uint8_t linacc[6] = {0};
  uint8_t gyro[6] = {0};
  uint8_t quat[8] = {0};

  if (i2c_read_reg(BNO_REG_CALIB_STAT, &calib, 1) != ESP_OK ||
      i2c_read_reg(BNO_REG_LINACC_DATA, linacc, sizeof(linacc)) != ESP_OK ||
      i2c_read_reg(BNO_REG_GYRO_DATA, gyro, sizeof(gyro)) != ESP_OK ||
      i2c_read_reg(BNO_REG_QUATERNION_DATA, quat, sizeof(quat)) != ESP_OK) {
    ESP_LOGW(TAG, "BNO055 read failed at 0x%02X", bno_addr);
    bno_ready = false;
    return;
  }

   last_bno_teleplot.linacc[0] = read_i16_le(&linacc[0]);
    last_bno_teleplot.linacc[1] = read_i16_le(&linacc[2]);
    last_bno_teleplot.linacc[2] = read_i16_le(&linacc[4]);
    last_bno_teleplot.gyro[0] = read_i16_le(&gyro[0]);
    last_bno_teleplot.gyro[1] = read_i16_le(&gyro[2]);
    last_bno_teleplot.gyro[2] = read_i16_le(&gyro[4]);
    last_bno_teleplot.quat[0] = read_i16_le(&quat[0]);
    last_bno_teleplot.quat[1] = read_i16_le(&quat[2]);
    last_bno_teleplot.quat[2] = read_i16_le(&quat[4]);
    last_bno_teleplot.quat[3] = read_i16_le(&quat[6]);

  emit_bno_teleplot(linacc, gyro, quat, calib);
}

bool try_init_bno_on_addr(uint8_t addr) {
  if (i2c_master_probe(i2c_bus, addr, 200) != ESP_OK) {
    ESP_LOGW(TAG, "BNO055 not responding at 0x%02X", addr);
    return false;
  }

  i2c_device_config_t dev_cfg = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = addr,
      .scl_speed_hz = I2C_FREQ_HZ,
      .scl_wait_us = 0,
      .flags.disable_ack_check = 0,
  };

  i2c_master_dev_handle_t candidate = NULL;
  if (i2c_master_bus_add_device(i2c_bus, &dev_cfg, &candidate) != ESP_OK) {
    ESP_LOGW(TAG, "Failed adding I2C device at 0x%02X", addr);
    return false;
  }

  bno_addr = addr;
  bno_dev = candidate;
  uint8_t chip_id = 0;
  if (i2c_read_reg(BNO_REG_CHIP_ID, &chip_id, 1) != ESP_OK) {
    ESP_LOGW(TAG, "BNO055 not responding at 0x%02X", bno_addr);
    i2c_master_bus_rm_device(bno_dev);
    bno_dev = NULL;
    return false;
  }

  if (chip_id != 0xA0) {
    ESP_LOGW(TAG, "Unexpected BNO055 chip id 0x%02X at 0x%02X", chip_id, bno_addr);
    i2c_master_bus_rm_device(bno_dev);
    bno_dev = NULL;
    return false;
  }

  if (i2c_write_reg(BNO_REG_OPR_MODE, BNO_MODE_CONFIG) != ESP_OK) {
    i2c_master_bus_rm_device(bno_dev);
    bno_dev = NULL;
    return false;
  }
  vTaskDelay(pdMS_TO_TICKS(30));
  if (i2c_write_reg(BNO_REG_PAGE_ID, 0x00) != ESP_OK) {
    i2c_master_bus_rm_device(bno_dev);
    bno_dev = NULL;
    return false;
  }
  if (i2c_write_reg(BNO_REG_OPR_MODE, BNO_MODE_NDOF) != ESP_OK) {
    i2c_master_bus_rm_device(bno_dev);
    bno_dev = NULL;
    return false;
  }
  vTaskDelay(pdMS_TO_TICKS(20));

  ESP_LOGI(TAG, "BNO055 ready at 0x%02X", bno_addr);
  return true;
}

void init_i2c(void) {
  const i2c_master_bus_config_t config = {
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .i2c_port = I2C_NUM_0,
      .scl_io_num = I2C_SCL_GPIO,
      .sda_io_num = I2C_SDA_GPIO,
      .glitch_ignore_cnt = 7,
      .flags.enable_internal_pullup = true,
  };

  ESP_ERROR_CHECK(i2c_new_master_bus(&config, &i2c_bus));
  ESP_LOGI(TAG, "I2C initialized SDA=%d SCL=%d", I2C_SDA_GPIO, I2C_SCL_GPIO);
}

bool init_bno(void) {
  
    init_i2c();

  if (bno_dev != NULL) {
    i2c_master_bus_rm_device(bno_dev);
    bno_dev = NULL;
  }
  if (try_init_bno_on_addr(BNO_ADDR_PRIMARY)) return true;
  if (try_init_bno_on_addr(BNO_ADDR_SECONDARY)) return true;
  return false;
}

