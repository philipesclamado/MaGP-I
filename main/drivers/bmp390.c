#include "bmp390.h"

#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "bmp390";

/* I2C */
#define BMP390_I2C_ADDR 0x77U

/* Identification */
#define BMP390_CHIP_ID 0x60U

/* Registers */
#define BMP390_REG_CHIP_ID 0x00U
#define BMP390_REG_ERR 0x02U
#define BMP390_REG_STATUS 0x03U
#define BMP390_REG_DATA 0x04U
#define BMP390_REG_PWR_CTRL 0x1BU
#define BMP390_REG_OSR 0x1CU
#define BMP390_REG_ODR 0x1DU
#define BMP390_REG_CONFIG 0x1FU
#define BMP390_REG_CALIB 0x31U
#define BMP390_REG_CMD 0x7EU

/* Commands */
#define BMP390_CMD_SOFT_RESET 0xB6U

/* STATUS */
#define BMP390_STATUS_DRDY_PRESS 0x20U
#define BMP390_STATUS_DRDY_TEMP 0x40U

/* ERR_REG */
#define BMP390_ERR_FATAL 0x01U
#define BMP390_ERR_CMD 0x02U
#define BMP390_ERR_CONF 0x04U

/*
 * PWR_CTRL:
 *
 * mode = normal/forced
 * temp_en = 1
 * press_en = 1
 */
#define BMP390_PWR_PRESS_TEMP 0x03U
#define BMP390_MODE_FORCED 0x01U
#define BMP390_MODE_NORMAL 0x30U

/*
 * OSR:
 *
 * pressure x4  = 0b010
 * temperature x1 = 0b000
 */
#define BMP390_OSR_PRESS_X4_TEMP_X1 0x02U

/*
 * 25 Hz.
 */
#define BMP390_ODR_25HZ 0x03U

/*
 * No IIR filter.
 */
#define BMP390_CONFIG_NO_FILTER 0x00U

#define BMP390_CALIB_LENGTH 21U
#define BMP390_DATA_LENGTH 6U

#define BMP390_READ_TIMEOUT_MS 20U

/* -------------------------------------------------------------------------- */
/* I2C                                                                        */
/* -------------------------------------------------------------------------- */

static esp_err_t read_reg(bmp390_t *dev, uint8_t reg, uint8_t *data,
                          size_t length) {
  if (dev == NULL || dev->i2c_dev == NULL || data == NULL || length == 0) {
    return ESP_ERR_INVALID_ARG;
  }

  return i2c_master_transmit_receive(dev->i2c_dev, &reg, 1, data, length, 100);
}

static esp_err_t write_reg(bmp390_t *dev, uint8_t reg, uint8_t value) {
  if (dev == NULL || dev->i2c_dev == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  uint8_t data[2] = {reg, value};

  return i2c_master_transmit(dev->i2c_dev, data, sizeof(data), 100);
}

static esp_err_t read_u8(bmp390_t *dev, uint8_t reg, uint8_t *value) {
  return read_reg(dev, reg, value, 1);
}

/* -------------------------------------------------------------------------- */
/* Integer decoding                                                           */
/* -------------------------------------------------------------------------- */

static uint16_t u16_le(const uint8_t *p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static int16_t s16_le(const uint8_t *p) { return (int16_t)u16_le(p); }

static uint32_t u24_le(const uint8_t *p) {
  return ((uint32_t)p[0]) | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16);
}

/* -------------------------------------------------------------------------- */
/* Calibration                                                                */
/* -------------------------------------------------------------------------- */

static void read_calibration(bmp390_t *dev, const uint8_t *data) {
  /*
   * These are the exact BMP390 NVM coefficient types.
   *
   * T1, T2, P5 and P6 are unsigned.
   * T3, P3, P4, P7, P8, P10 and P11 are signed 8-bit.
   * P1, P2 and P9 are signed 16-bit.
   */

  const uint16_t nvm_t1 = u16_le(&data[0]);
  const uint16_t nvm_t2 = u16_le(&data[2]);
  const int8_t nvm_t3 = (int8_t)data[4];

  const int16_t nvm_p1 = s16_le(&data[5]);
  const int16_t nvm_p2 = s16_le(&data[7]);
  const int8_t nvm_p3 = (int8_t)data[9];
  const int8_t nvm_p4 = (int8_t)data[10];
  const uint16_t nvm_p5 = u16_le(&data[11]);
  const uint16_t nvm_p6 = u16_le(&data[13]);
  const int8_t nvm_p7 = (int8_t)data[15];
  const int8_t nvm_p8 = (int8_t)data[16];
  const int16_t nvm_p9 = s16_le(&data[17]);
  const int8_t nvm_p10 = (int8_t)data[19];
  const int8_t nvm_p11 = (int8_t)data[20];

  /*
   * Bosch floating-point coefficient conversion.
   *
   * PAR_T1 = NVM_PAR_T1 / 2^-8
   * PAR_T2 = NVM_PAR_T2 / 2^30
   * PAR_T3 = NVM_PAR_T3 / 2^48
   */
  dev->par_t1 = (double)nvm_t1 / 0.00390625;
  dev->par_t2 = (double)nvm_t2 / 1073741824.0;
  dev->par_t3 = (double)nvm_t3 / 281474976710656.0;

  dev->par_p1 = ((double)nvm_p1 - 16384.0) / 1048576.0;

  dev->par_p2 = ((double)nvm_p2 - 16384.0) / 536870912.0;

  dev->par_p3 = (double)nvm_p3 / 4294967296.0;

  dev->par_p4 = (double)nvm_p4 / 137438953472.0;

  dev->par_p5 = (double)nvm_p5 / 0.125;

  dev->par_p6 = (double)nvm_p6 / 64.0;

  dev->par_p7 = (double)nvm_p7 / 256.0;

  dev->par_p8 = (double)nvm_p8 / 32768.0;

  dev->par_p9 = (double)nvm_p9 / 281474976710656.0;

  dev->par_p10 = (double)nvm_p10 / 281474976710656.0;

  dev->par_p11 = (double)nvm_p11 / 36893488147419103232.0;
}

/* -------------------------------------------------------------------------- */
/* Bosch compensation                                                         */
/* -------------------------------------------------------------------------- */

static double compensate_temperature(bmp390_t *dev, uint32_t uncomp_temp) {
  /*
   * This follows Bosch's reference implementation directly.
   */
  const double partial_data1 = (double)uncomp_temp - dev->par_t1;

  const double partial_data2 = partial_data1 * dev->par_t2;

  dev->t_lin = partial_data2 + (partial_data1 * partial_data1) * dev->par_t3;

  return dev->t_lin;
}

static double compensate_pressure(const bmp390_t *dev, uint32_t uncomp_press) {
  /*
   * This follows Bosch's reference implementation directly.
   */

  const double t = dev->t_lin;

  const double t2 = t * t;
  const double t3 = t2 * t;

  const double partial_out1 =
      dev->par_p5 + dev->par_p6 * t + dev->par_p7 * t2 + dev->par_p8 * t3;

  const double partial_out2 =
      (double)uncomp_press *
      (dev->par_p1 + dev->par_p2 * t + dev->par_p3 * t2 + dev->par_p4 * t3);

  const double press2 = (double)uncomp_press * (double)uncomp_press;

  const double partial_data = press2 * (dev->par_p9 + dev->par_p10 * t);

  const double press3 = press2 * (double)uncomp_press;

  const double partial_data4 = partial_data + press3 * dev->par_p11;

  return partial_out1 + partial_out2 + partial_data4;
}

/* -------------------------------------------------------------------------- */
/* Initialization                                                             */
/* -------------------------------------------------------------------------- */

esp_err_t bmp390_init(i2c_master_bus_handle_t bus_handle, bmp390_t *dev) {
  if (bus_handle == NULL || dev == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  memset(dev, 0, sizeof(*dev));

  const i2c_device_config_t config = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = BMP390_I2C_ADDR,
      .scl_speed_hz = 400000,
  };

  esp_err_t err = i2c_master_bus_add_device(bus_handle, &config, &dev->i2c_dev);

  if (err != ESP_OK) {
    return err;
  }

  /* Verify chip. */
  uint8_t chip_id = 0;

  err = read_u8(dev, BMP390_REG_CHIP_ID, &chip_id);

  if (err != ESP_OK) {
    goto fail;
  }

  if (chip_id != BMP390_CHIP_ID) {
    ESP_LOGE(TAG, "Unexpected chip ID: 0x%02X", chip_id);

    err = ESP_ERR_NOT_FOUND;
    goto fail;
  }

  /*
   * Reset.
   *
   * Give the sensor enough time to finish the reset before reading
   * calibration data.
   */
  err = write_reg(dev, BMP390_REG_CMD, BMP390_CMD_SOFT_RESET);

  if (err != ESP_OK) {
    goto fail;
  }

  vTaskDelay(pdMS_TO_TICKS(10));

  /*
   * Read factory calibration.
   */
  uint8_t calib[BMP390_CALIB_LENGTH];

  err = read_reg(dev, BMP390_REG_CALIB, calib, sizeof(calib));

  if (err != ESP_OK) {
    goto fail;
  }

  read_calibration(dev, calib);

  /*
   * Pressure x4, temperature x1.
   */
  err = write_reg(dev, BMP390_REG_OSR, BMP390_OSR_PRESS_X4_TEMP_X1);

  if (err != ESP_OK) {
    goto fail;
  }

  /*
   * 25 Hz output data rate.
   */
  err = write_reg(dev, BMP390_REG_ODR, BMP390_ODR_25HZ);

  if (err != ESP_OK) {
    goto fail;
  }

  /*
   * No sensor-side IIR filter.
   */
  err = write_reg(dev, BMP390_REG_CONFIG, BMP390_CONFIG_NO_FILTER);

  if (err != ESP_OK) {
    goto fail;
  }

  /*
   * Enable pressure and temperature.
   *
   * Leave the sensor in sleep mode here.
   * bmp390_read() starts each measurement explicitly.
   */
  err = write_reg(dev, BMP390_REG_PWR_CTRL, BMP390_PWR_PRESS_TEMP);

  if (err != ESP_OK) {
    goto fail;
  }

  dev->initialized = true;

  return ESP_OK;

fail:
  if (dev->i2c_dev != NULL) {
    i2c_master_bus_rm_device(dev->i2c_dev);
    dev->i2c_dev = NULL;
  }

  return err;
}

/* -------------------------------------------------------------------------- */
/* Read                                                                       */
/* -------------------------------------------------------------------------- */

esp_err_t bmp390_read(bmp390_t *dev, float *pressure_hpa,
                      float *temperature_c) {
  if (dev == NULL || pressure_hpa == NULL || temperature_c == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  if (!dev->initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  /*
   * Start one forced measurement.
   *
   * PWR_CTRL:
   *
   * 0x03 = pressure + temperature enabled
   * 0x04 = forced mode
   *
   * Therefore 0x07.
   */
  esp_err_t err = write_reg(dev, BMP390_REG_PWR_CTRL,
                            BMP390_PWR_PRESS_TEMP | (BMP390_MODE_FORCED << 4));

  /*
   * NOTE:
   *
   * The mode field occupies bits 5:4.
   * Forced mode is value 01, hence 0x10.
   *
   * So the actual value written above is 0x13.
   */
  if (err != ESP_OK) {
    return err;
  }

  /*
   * Wait for both pressure and temperature.
   */
  uint8_t status = 0;

  bool ready = false;

  for (uint32_t i = 0; i < BMP390_READ_TIMEOUT_MS; ++i) {
    err = read_u8(dev, BMP390_REG_STATUS, &status);

    if (err != ESP_OK) {
      return err;
    }

    if ((status & BMP390_STATUS_DRDY_PRESS) &&
        (status & BMP390_STATUS_DRDY_TEMP)) {
      ready = true;
      break;
    }

    vTaskDelay(pdMS_TO_TICKS(1));
  }

  if (!ready) {
    return ESP_ERR_TIMEOUT;
  }

  /*
   * Check sensor error register.
   */
  uint8_t sensor_error = 0;

  err = read_u8(dev, BMP390_REG_ERR, &sensor_error);

  if (err != ESP_OK) {
    return err;
  }

  if (sensor_error & (BMP390_ERR_FATAL | BMP390_ERR_CMD | BMP390_ERR_CONF)) {
    ESP_LOGE(TAG, "Sensor error: 0x%02X", sensor_error);

    return ESP_FAIL;
  }

  /*
   * Read pressure and temperature together.
   */
  uint8_t data[BMP390_DATA_LENGTH];

  err = read_reg(dev, BMP390_REG_DATA, data, sizeof(data));

  if (err != ESP_OK) {
    return err;
  }

  const uint32_t raw_pressure = u24_le(&data[0]);

  const uint32_t raw_temperature = u24_le(&data[3]);

  /*
   * Bosch compensation.
   */
  const double temperature = compensate_temperature(dev, raw_temperature);

  const double pressure = compensate_pressure(dev, raw_pressure);

  /*
   * BMP390 returns pressure in Pa.
   */
  *temperature_c = (float)temperature;
  *pressure_hpa = (float)(pressure / 100.0);

  return ESP_OK;
}

/* -------------------------------------------------------------------------- */
/* Deinitialization                                                           */
/* -------------------------------------------------------------------------- */

esp_err_t bmp390_deinit(bmp390_t *dev) {
  if (dev == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  if (dev->i2c_dev != NULL) {
    esp_err_t err = i2c_master_bus_rm_device(dev->i2c_dev);

    if (err != ESP_OK) {
      return err;
    }
  }

  memset(dev, 0, sizeof(*dev));

  return ESP_OK;
}
