#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"

#define MLX90393_I2C_ADDR 0x0C

typedef struct {
  i2c_master_dev_handle_t i2c_device;
} mlx90393_t;

esp_err_t mlx90393_init(i2c_master_bus_handle_t bus_handle, mlx90393_t *device);
