#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"

typedef struct {
  int16_t x;
  int16_t y;
  int16_t z;
} accel_t;

typedef struct {
  i2c_master_dev_handle_t i2c_device;
} mpu6050_t;

esp_err_t mpu6050_init(i2c_master_bus_handle_t bus_handle, mpu6050_t *device);

esp_err_t mpu6050_deinit(mpu6050_t *device);

esp_err_t mpu6050_measure(mpu6050_t *device, accel_t *accel);
