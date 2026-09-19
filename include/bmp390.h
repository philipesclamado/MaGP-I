#pragma once

#include <stdbool.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

typedef struct {
  i2c_master_dev_handle_t i2c_dev;

  /* Bosch BMP390 calibration coefficients. */
  double par_t1;
  double par_t2;
  double par_t3;

  double par_p1;
  double par_p2;
  double par_p3;
  double par_p4;
  double par_p5;
  double par_p6;
  double par_p7;
  double par_p8;
  double par_p9;
  double par_p10;
  double par_p11;

  double t_lin;

  bool initialized;
} bmp390_t;

esp_err_t bmp390_init(i2c_master_bus_handle_t bus_handle, bmp390_t *dev);

esp_err_t bmp390_deinit(bmp390_t *dev);

esp_err_t bmp390_measure(bmp390_t *dev, float *pressure_hpa,
                         float *temperature_c);
