#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#define MLX90393_ADDR                                                          \
  0x0C // unmodified - both a_0 and a_1 are shorted to ground

#define CMD_SM 0x3E
#define CMD_RM 0x4E

typedef struct {
  int16_t x;
  int16_t y;
  int16_t z;
} mag_t;

typedef struct {
  i2c_master_dev_handle_t i2c_device;
} mlx90393_t;

esp_err_t mlx90393_init(i2c_master_bus_handle_t bus_handle, mlx90393_t *device);

esp_err_t mlx90393_measure(mlx90393_t *sensor, mag_t *data);
