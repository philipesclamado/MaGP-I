#include "mlx90393.h"

#define MLX90393_ADDR                                                          \
  0x0C // unmodified - both a_0 and a_1 are shorted to ground

static esp_err_t reg_read(mlx90393_t *device, uint8_t reg, uint8_t *value) {
  if (device == NULL || device->i2c_device == NULL || value == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  return i2c_master_transmit_receive(device->i2c_device, &reg, 1, value, 1,
                                     100);
}

static esp_err_t reg_write(mlx90393_t *device, uint8_t reg, uint8_t val) {
  if (device == NULL || device->i2c_device == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  uint8_t data[2] = {reg, val};

  return i2c_master_transmit(device->i2c_device, data, sizeof(data), 100);
}

esp_err_t mlx90393_init(i2c_master_bus_handle_t bus_handle,
                        mlx90393_t *device) {
  if (bus_handle == NULL || device == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  const i2c_device_config_t config = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = MLX90393_ADDR,
      .scl_speed_hz = 400000,

  };

  esp_err_t err =
      i2c_master_bus_add_device(bus_handle, &config, &device->i2c_device);
  if (err != ESP_OK) {
    return err;
  }

  return ESP_OK;
}
