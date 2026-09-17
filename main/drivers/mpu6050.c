#include "mpu6050.h"

#include "esp_log.h"

static const char *TAG = "MPU-6050";

#define MPU6050_ADDR 0x68

#define PWR_MGMT_1 0x6B
#define ACCEL_CONFIG 0x1C

static esp_err_t reg_read(mpu6050_t *device, uint8_t reg, uint8_t *value) {
  if (device == NULL || device->i2c_device == NULL || value == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  return i2c_master_transmit_receive(device->i2c_device, &reg, 1, value, 1,
                                     100);
}

static esp_err_t reg_write(mpu6050_t *device, uint8_t reg, uint8_t val) {
  if (device == NULL || device->i2c_device == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  uint8_t data[2] = {reg, val};

  return i2c_master_transmit(device->i2c_device, data, sizeof(data), 100);
}

static esp_err_t mpu6050_config(mpu6050_t *device) {
  // 4.28 Register 107 – Power Management 1
  esp_err_t err = reg_write(device, PWR_MGMT_1, 0x00);
  if (err != ESP_OK) {
    return err;
  }

  // ± 8g
  err = reg_write(device, ACCEL_CONFIG, 0x10);
  if (err != ESP_OK) {
    return err;
  }

  return ESP_OK;
}

esp_err_t mpu6050_init(i2c_master_bus_handle_t bus_handle, mpu6050_t *device) {
  if (bus_handle == NULL || device == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  const i2c_device_config_t config = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = MPU6050_ADDR,
      .scl_speed_hz = 400000,

  };

  esp_err_t err =
      i2c_master_bus_add_device(bus_handle, &config, &device->i2c_device);
  if (err != ESP_OK) {
    return err;
  }

  err = mpu6050_config(device);
  if (err != ESP_OK) {
    return err;
  }

  return ESP_OK;
}
