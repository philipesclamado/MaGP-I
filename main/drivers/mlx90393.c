#include "mlx90393.h"

#include "esp_log.h"

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

static esp_err_t mlx90393_command(mlx90393_t *sensor, uint8_t command,
                                  uint8_t *response, size_t response_len) {
  esp_err_t ret;

  ret = i2c_master_transmit(sensor->i2c_device, &command, 1, 100);

  if (ret != ESP_OK) {
    return ret;
  }

  if (response && response_len > 0) {
    ret = i2c_master_receive(sensor->i2c_device, response, response_len, 100);
  }

  return ret;
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

esp_err_t mlx90393_measure(mlx90393_t *sensor, mag_t *data) {
  if (sensor == NULL || sensor->i2c_device == NULL || data == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  uint8_t cmd = CMD_SM;
  uint8_t status;
  uint8_t rx[7];

  // Start single measurement: X + Y + Z
  esp_err_t ret =
      i2c_master_transmit_receive(sensor->i2c_device, &cmd, 1, &status, 1, 100);

  if (ret != ESP_OK) {
    return ret;
  }

  // Give the conversion time to complete.
  vTaskDelay(pdMS_TO_TICKS(10));

  // Read measurement: X + Y + Z
  cmd = CMD_RM;

  ret = i2c_master_transmit_receive(sensor->i2c_device, &cmd, 1, rx, sizeof(rx),
                                    100);

  if (ret != ESP_OK) {
    return ret;
  }

  data->x = (int16_t)((rx[1] << 8) | rx[2]);
  data->y = (int16_t)((rx[3] << 8) | rx[4]);
  data->z = (int16_t)((rx[5] << 8) | rx[6]);

  return ESP_OK;
}
