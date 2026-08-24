#include "bno.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


TaskHandle_t bno_task_handle = NULL;

static uint8_t bno_addr = BNO_ADDR_PRIMARY;
static i2c_master_bus_handle_t i2c_bus = NULL;
static i2c_master_dev_handle_t bno_dev = NULL;
static volatile bool bno_sleeping = false;

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


bool print_bno_status(void) {
  enum {
    TELEMETRY_LENGTH = BNO_REG_OPR_MODE - BNO_REG_MAG_DATA + 1,
    MAG_OFFSET = BNO_REG_MAG_DATA - BNO_REG_MAG_DATA,
    GYRO_OFFSET = BNO_REG_GYRO_DATA - BNO_REG_MAG_DATA,
    QUAT_OFFSET = BNO_REG_QUATERNION_DATA - BNO_REG_MAG_DATA,
    LINACC_OFFSET = BNO_REG_LINACC_DATA - BNO_REG_MAG_DATA,
    GRAVITY_OFFSET = BNO_REG_GRAVITY_DATA - BNO_REG_MAG_DATA,
    TEMP_OFFSET = BNO_REG_TEMP - BNO_REG_MAG_DATA,
    CALIB_OFFSET = BNO_REG_CALIB_STAT - BNO_REG_MAG_DATA,
    SYS_STATUS_OFFSET = BNO_REG_SYS_STATUS - BNO_REG_MAG_DATA,
    SYS_ERROR_OFFSET = BNO_REG_SYS_ERR - BNO_REG_MAG_DATA,
    OP_MODE_OFFSET = BNO_REG_OPR_MODE - BNO_REG_MAG_DATA,
  };
  uint8_t telemetry[TELEMETRY_LENGTH] = {0};

  if (i2c_read_reg(BNO_REG_MAG_DATA, telemetry, sizeof(telemetry)) != ESP_OK) {
    ESP_LOGW(TAG, "BNO055 read failed at 0x%02X", bno_addr);
    bno_ready = false;
    return false;
  }

  last_bno_teleplot.linacc[0] = read_i16_le(&telemetry[LINACC_OFFSET]);
  last_bno_teleplot.linacc[1] = read_i16_le(&telemetry[LINACC_OFFSET + 2]);
  last_bno_teleplot.linacc[2] = read_i16_le(&telemetry[LINACC_OFFSET + 4]);
  last_bno_teleplot.mag[0] = read_i16_le(&telemetry[MAG_OFFSET]);
  last_bno_teleplot.mag[1] = read_i16_le(&telemetry[MAG_OFFSET + 2]);
  last_bno_teleplot.mag[2] = read_i16_le(&telemetry[MAG_OFFSET + 4]);
  last_bno_teleplot.gravity[0] = read_i16_le(&telemetry[GRAVITY_OFFSET]);
  last_bno_teleplot.gravity[1] = read_i16_le(&telemetry[GRAVITY_OFFSET + 2]);
  last_bno_teleplot.gravity[2] = read_i16_le(&telemetry[GRAVITY_OFFSET + 4]);
  last_bno_teleplot.gyro[0] = read_i16_le(&telemetry[GYRO_OFFSET]);
  last_bno_teleplot.gyro[1] = read_i16_le(&telemetry[GYRO_OFFSET + 2]);
  last_bno_teleplot.gyro[2] = read_i16_le(&telemetry[GYRO_OFFSET + 4]);
  last_bno_teleplot.quat[0] = read_i16_le(&telemetry[QUAT_OFFSET]);
  last_bno_teleplot.quat[1] = read_i16_le(&telemetry[QUAT_OFFSET + 2]);
  last_bno_teleplot.quat[2] = read_i16_le(&telemetry[QUAT_OFFSET + 4]);
  last_bno_teleplot.quat[3] = read_i16_le(&telemetry[QUAT_OFFSET + 6]);
  last_bno_teleplot.calib = telemetry[CALIB_OFFSET];
  last_bno_teleplot.op_mode = telemetry[OP_MODE_OFFSET];
  last_bno_teleplot.sys_status = telemetry[SYS_STATUS_OFFSET];
  last_bno_teleplot.sys_error = telemetry[SYS_ERROR_OFFSET];
  last_bno_teleplot.temp_c = (int8_t)telemetry[TEMP_OFFSET];
  last_bno_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

  return true;
}

void bno_set_sleeping(bool sleeping) {
  bno_sleeping = sleeping;
}

bool bno_suspend(void) {
  if (bno_dev == NULL) return false;

  if (i2c_write_reg(BNO_REG_OPR_MODE, BNO_MODE_CONFIG) != ESP_OK) {
    ESP_LOGW(TAG, "Failed to switch BNO055 to config mode before suspend");
    return false;
  }
  vTaskDelay(pdMS_TO_TICKS(30));

  if (i2c_write_reg(BNO_REG_PWR_MODE, BNO_MODE_SUSPEND) != ESP_OK) {
    ESP_LOGW(TAG, "Failed to put BNO055 into suspend mode");
    return false;
  }

  vTaskDelay(pdMS_TO_TICKS(20));
  return true;
}

bool bno_resume(void) {
  if (bno_dev == NULL) return false;

  if (i2c_write_reg(BNO_REG_PWR_MODE, BNO_PWR_MODE_NORMAL) != ESP_OK) {
    ESP_LOGW(TAG, "Failed to restore BNO055 normal power mode");
    return false;
  }
  vTaskDelay(pdMS_TO_TICKS(30));

  if (i2c_write_reg(BNO_REG_OPR_MODE, BNO_MODE_CONFIG) != ESP_OK) {
    ESP_LOGW(TAG, "Failed to switch BNO055 to config mode");
    return false;
  }
  vTaskDelay(pdMS_TO_TICKS(30));

#if BNO_USE_EXTERNAL_CRYSTAL
  if (i2c_write_reg(BNO_REG_SYS_TRIGGER, BNO_SYS_TRIGGER_CLK_SEL) != ESP_OK) {
    ESP_LOGW(TAG, "Failed to select external BNO055 clock source");
    return false;
  }

  for (int retry = 0; retry < 10; retry++) {
    uint8_t clk_status = 0;
    if (i2c_read_reg(BNO_REG_SYS_CLK_STATUS, &clk_status, 1) != ESP_OK) {
      ESP_LOGW(TAG, "Failed to read BNO055 clock status");
      return false;
    }

    if ((clk_status & 0x01U) == 0U) {
      break;
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
#endif

  if (i2c_write_reg(BNO_REG_PAGE_ID, 0x00) != ESP_OK) {
    ESP_LOGW(TAG, "Failed to select BNO055 page 0");
    return false;
  }
  if (i2c_write_reg(BNO_REG_OPR_MODE, BNO_MODE_NDOF) != ESP_OK) {
    ESP_LOGW(TAG, "Failed to restore BNO055 NDOF mode");
    return false;
  }

  vTaskDelay(pdMS_TO_TICKS(20));
  return true;
}

// AI: below section was generated by an AI
/* Resets sensor state that can survive an MCU-only reset, then waits for BNO055 startup. */
static bool reset_bno(void) {
  if (i2c_write_reg(BNO_REG_OPR_MODE, BNO_MODE_CONFIG) != ESP_OK) return false;
  vTaskDelay(pdMS_TO_TICKS(30));
  if (i2c_write_reg(BNO_REG_SYS_TRIGGER, BNO_SYS_TRIGGER_RST_SYS) != ESP_OK) return false;
  vTaskDelay(pdMS_TO_TICKS(700));

  for (int retry = 0; retry < 20; retry++) {
    uint8_t chip_id = 0;
    if (i2c_read_reg(BNO_REG_CHIP_ID, &chip_id, 1) == ESP_OK && chip_id == 0xA0) return true;
    vTaskDelay(pdMS_TO_TICKS(50));
  }
  return false;
}
// AI: end

// AI: below section was generated by an AI
/* Samples the raw magnetometer outside fusion mode, then restores NDOF. */
static bool probe_magonly(void) {
  uint8_t raw[6] = {0};

  if (i2c_write_reg(BNO_REG_OPR_MODE, BNO_MODE_CONFIG) != ESP_OK) return false;
  vTaskDelay(pdMS_TO_TICKS(30));
  if (i2c_write_reg(BNO_REG_OPR_MODE, BNO_MODE_MAGONLY) != ESP_OK) return false;
  vTaskDelay(pdMS_TO_TICKS(100));

  const bool read_ok = i2c_read_reg(BNO_REG_MAG_DATA, raw, sizeof(raw)) == ESP_OK;
  if (read_ok) {
    last_bno_teleplot.mag_probe[0] = read_i16_le(&raw[0]);
    last_bno_teleplot.mag_probe[1] = read_i16_le(&raw[2]);
    last_bno_teleplot.mag_probe[2] = read_i16_le(&raw[4]);
  }

  if (i2c_write_reg(BNO_REG_OPR_MODE, BNO_MODE_CONFIG) != ESP_OK) return false;
  vTaskDelay(pdMS_TO_TICKS(30));
  if (i2c_write_reg(BNO_REG_OPR_MODE, BNO_MODE_NDOF) != ESP_OK) return false;
  vTaskDelay(pdMS_TO_TICKS(100));
  return read_ok;
}
// AI: end

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

  if (!reset_bno()) {
    ESP_LOGW(TAG, "BNO055 reset failed at 0x%02X", bno_addr);
    i2c_master_bus_rm_device(bno_dev);
    bno_dev = NULL;
    return false;
  }

  if (!bno_resume()) {
    i2c_master_bus_rm_device(bno_dev);
    bno_dev = NULL;
    return false;
  }

  if (probe_magonly()) {
    ESP_LOGI(TAG, "BNO055 MAGONLY probe raw X=%d Y=%d Z=%d",
             last_bno_teleplot.mag_probe[0], last_bno_teleplot.mag_probe[1],
             last_bno_teleplot.mag_probe[2]);
  } else {
    ESP_LOGW(TAG, "BNO055 MAGONLY probe failed");
  }

  uint8_t st_result = 0;
  if (i2c_read_reg(BNO_REG_ST_RESULT, &st_result, 1) == ESP_OK) {
    last_bno_teleplot.selftest = st_result;
    ESP_LOGI(TAG, "BNO055 self-test result 0x%02X: MCU=%d GYR=%d ACC=%d MAG=%d",
             st_result, st_result & 0x01, (st_result >> 1) & 0x01,
             (st_result >> 2) & 0x01, (st_result >> 3) & 0x01);
    if (((st_result >> 3) & 0x01) == 0) {
      ESP_LOGW(TAG, "BNO055 magnetometer failed power-on self-test - this board's "
                    "magnetometer hardware is likely non-functional (common on some "
                    "BNO055 clone/counterfeit modules); /glowstick/mag will read all "
                    "zeros regardless of firmware and calib_stat's mag bits may be "
                    "misleadingly stuck at 3");
    }
  } else {
    ESP_LOGW(TAG, "Failed to read BNO055 self-test result register");
  }

  ESP_LOGI(TAG, "BNO055 ready at 0x%02X", bno_addr);
  return true;
}

void init_i2c(void) {
  if (i2c_bus != NULL) {
    return;
  }

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

void bno_task(void *arg) {
  ESP_LOGI(TAG, "BNO task started");
  TickType_t next_sample = xTaskGetTickCount();
  while (1) {
    if (bno_sleeping) {
      vTaskDelay(pdMS_TO_TICKS(50));
      next_sample = xTaskGetTickCount();
      continue;
    }

    bno_ready = print_bno_status();
    xTaskDelayUntil(&next_sample, pdMS_TO_TICKS(BNO_SAMPLE_PERIOD_MS));
  }
}

bool init_bno(void) {
  bno_set_sleeping(false);
  init_i2c();

  if (bno_dev != NULL) {
    i2c_master_bus_rm_device(bno_dev);
    bno_dev = NULL;
  }
  bool initGood = try_init_bno_on_addr(BNO_ADDR_PRIMARY);
  if (!initGood) {
    initGood = try_init_bno_on_addr(BNO_ADDR_SECONDARY);
  }

  if (bno_task_handle == NULL) {
    xTaskCreatePinnedToCore(bno_task, "bno_task", 4096, NULL, 5, &bno_task_handle, tskNO_AFFINITY);
  }
  return initGood;
}


