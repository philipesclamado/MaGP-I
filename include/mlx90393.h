#pragma once

#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

#define MLX90393_ADDR 0x0C

/*
 * With OSR=3 and DIG_FILT=2, XYZ conversion takes up to approximately
 * 17.7 ms including oscillator tolerance.  Keep a 20 ms interval between
 * starting a conversion and reading it.
 */
#define MLX90393_XYZ_CONVERSION_TIME_MS 20U

/*
 * MLX90393 command base values.
 *
 * RT is special:
 *   - Generic/SPI command byte: 0xF0
 *   - I2C wire command byte:    0x80
 */
#define MLX90393_CMD_SB 0x10
#define MLX90393_CMD_SW 0x20
#define MLX90393_CMD_SM 0x30
#define MLX90393_CMD_RM 0x40
#define MLX90393_CMD_RR 0x50
#define MLX90393_CMD_WR 0x60
#define MLX90393_CMD_EX 0x80
#define MLX90393_CMD_RT 0xF0

#define MLX90393_CMD_RT_I2C 0x80

/* XYZ measurement flags */
#define MLX90393_AXIS_XYZ 0x0E

#define MLX90393_CMD_SM_XYZ (MLX90393_CMD_SM | MLX90393_AXIS_XYZ)

#define MLX90393_CMD_RM_XYZ (MLX90393_CMD_RM | MLX90393_AXIS_XYZ)

/* Registers */
#define MLX90393_REG_CONF1 0x00
#define MLX90393_REG_CONF2 0x01
#define MLX90393_REG_CONF3 0x02

/*
 * Magnetic field in microtesla.
 */
typedef struct {
  float x;
  float y;
  float z;
} mag_t;

typedef struct {
  i2c_master_dev_handle_t i2c_device;

  /*
   * Calibration offsets in microtesla.
   */
  float offset_x;
  float offset_y;
  float offset_z;

} mlx90393_t;

esp_err_t mlx90393_init(i2c_master_bus_handle_t bus_handle, mlx90393_t *device);

esp_err_t mlx90393_deinit(mlx90393_t *device);

/*
 * Non-blocking single-conversion interface.  Start a conversion, wait at
 * least MLX90393_XYZ_CONVERSION_TIME_MS, then read it.  This lets the flight
 * task keep its IMU/control deadline while magnetic data is converting.
 */
esp_err_t mlx90393_start_measurement(mlx90393_t *sensor);

esp_err_t mlx90393_read_measurement(mlx90393_t *sensor, mag_t *data);

esp_err_t mlx90393_measure(mlx90393_t *sensor, mag_t *data);
