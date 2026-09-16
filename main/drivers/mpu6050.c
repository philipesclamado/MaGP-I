#include "mpu6050.h"

#define MPU6050_ADDR 0x68

#define PWR_MGMT_1 0x6B

static esp_err_t reg_write(mpu6050_t *device, uint8_t reg, uint8_t val) {
  if (device == NULL || device->i2c_device == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  uint8_t data[2] = {reg, val};

  return i2c_master_transmit(device->i2c_device, data, sizeof(data), 100);
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

  err = reg_write(device, PWR_MGMT_1, 0x00);
  if (err != ESP_OK) {
    return err;
  }

  return ESP_OK;
}
