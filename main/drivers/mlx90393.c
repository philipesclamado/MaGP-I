#include "mlx90393.h"

#include <stdint.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "MLX90393"

/* -------------------------------------------------------------------------- */
/* Commands                                                                   */
/* -------------------------------------------------------------------------- */

#define MLX90393_CMD_NOP 0x00
#define MLX90393_CMD_SM 0x30
#define MLX90393_CMD_RM 0x40
#define MLX90393_CMD_RR 0x50
#define MLX90393_CMD_WR 0x60

#define MLX90393_AXIS_XYZ 0x0E

#define MLX90393_CMD_SM_XYZ (MLX90393_CMD_SM | MLX90393_AXIS_XYZ)

#define MLX90393_CMD_RM_XYZ (MLX90393_CMD_RM | MLX90393_AXIS_XYZ)

/* -------------------------------------------------------------------------- */
/* Registers                                                                  */
/* -------------------------------------------------------------------------- */

#define MLX90393_REG_CONF1 0x00
#define MLX90393_REG_CONF2 0x01
#define MLX90393_REG_CONF3 0x02

/* -------------------------------------------------------------------------- */
/* Configuration                                                              */
/* -------------------------------------------------------------------------- */

/*
 * CONF1 / REG00
 *
 * GAIN_SEL = 7
 * HALLCONF = 0xC
 *
 * Low byte = 0x7C
 */

#define MLX90393_GAIN_SEL 7
#define MLX90393_HALLCONF 0x0C

#define MLX90393_CONF1_VALUE ((MLX90393_GAIN_SEL << 4) | MLX90393_HALLCONF)

/*
 * CONF3 / REG02
 *
 * OSR      = 3
 * DIG_FILT = 2
 * RES_X    = 0
 * RES_Y    = 0
 * RES_Z    = 0
 */

#define MLX90393_OSR 3
#define MLX90393_DIG_FILT 2

#define MLX90393_CONF3_VALUE (MLX90393_OSR | (MLX90393_DIG_FILT << 2))

/* -------------------------------------------------------------------------- */
/* Sensitivity                                                                */
/* -------------------------------------------------------------------------- */

#define MLX90393_XY_UT_PER_LSB 0.150f
#define MLX90393_Z_UT_PER_LSB 0.242f

/* -------------------------------------------------------------------------- */
/* Status                                                                     */
/* -------------------------------------------------------------------------- */

#define MLX90393_STATUS_ERROR 0x10

/* -------------------------------------------------------------------------- */
/* Status check                                                               */
/* -------------------------------------------------------------------------- */

static esp_err_t mlx90393_check_status(uint8_t status)
{
    if (status & MLX90393_STATUS_ERROR)
    {
        ESP_LOGE(TAG, "Sensor error status=0x%02X", status);

        return ESP_FAIL;
    }

    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
/* Probe                                                                      */
/* -------------------------------------------------------------------------- */

static esp_err_t mlx90393_probe(i2c_master_bus_handle_t bus_handle)
{
    if (bus_handle == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Probing I2C address 0x%02X", MLX90393_ADDR);

    esp_err_t err = i2c_master_probe(bus_handle, MLX90393_ADDR, 100);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C probe failed: %s", esp_err_to_name(err));

        return err;
    }

    ESP_LOGI(TAG, "I2C address 0x%02X ACKed", MLX90393_ADDR);

    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
/* NOP                                                                        */
/* -------------------------------------------------------------------------- */

static esp_err_t mlx90393_nop(mlx90393_t *sensor)
{
    if (sensor == NULL || sensor->i2c_device == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t cmd = MLX90393_CMD_NOP;
    uint8_t status = 0;

    esp_err_t err =
        i2c_master_transmit_receive(sensor->i2c_device, &cmd, 1, &status, 1, 100);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "NOP failed: %s", esp_err_to_name(err));

        return err;
    }

    ESP_LOGI(TAG, "NOP status=0x%02X", status);

    return mlx90393_check_status(status);
}

/* -------------------------------------------------------------------------- */
/* Read register                                                              */
/* -------------------------------------------------------------------------- */

static esp_err_t mlx90393_read_register(mlx90393_t *sensor, uint8_t reg,
                                        uint16_t *value)
{
    if (sensor == NULL || sensor->i2c_device == NULL || value == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t tx[2] = {MLX90393_CMD_RR, (uint8_t)(reg << 2)};

    uint8_t rx[3] = {0};

    esp_err_t err = i2c_master_transmit_receive(sensor->i2c_device, tx,
                                                sizeof(tx), rx, sizeof(rx), 100);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "RR REG0x%02X failed: %s", reg, esp_err_to_name(err));

        return err;
    }

    ESP_LOGD(TAG, "RR REG0x%02X STATUS=0x%02X DATA=0x%02X%02X", reg, rx[0], rx[1],
             rx[2]);

    err = mlx90393_check_status(rx[0]);

    if (err != ESP_OK)
    {
        return err;
    }

    *value = ((uint16_t)rx[1] << 8) | (uint16_t)rx[2];

    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
/* Write register                                                             */
/* -------------------------------------------------------------------------- */

static esp_err_t mlx90393_write_register(mlx90393_t *sensor, uint8_t reg,
                                         uint16_t value)
{
    if (sensor == NULL || sensor->i2c_device == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t tx[4] = {MLX90393_CMD_WR, (uint8_t)(value >> 8),
                     (uint8_t)(value & 0xFF), (uint8_t)(reg << 2)};

    uint8_t status = 0;

    esp_err_t err = i2c_master_transmit_receive(sensor->i2c_device, tx,
                                                sizeof(tx), &status, 1, 100);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "WR REG0x%02X failed: %s", reg, esp_err_to_name(err));

        return err;
    }

    ESP_LOGD(TAG, "WR REG0x%02X value=0x%04X status=0x%02X", reg, value, status);

    return mlx90393_check_status(status);
}

/* -------------------------------------------------------------------------- */
/* Start measurement                                                          */
/* -------------------------------------------------------------------------- */

esp_err_t mlx90393_start_measurement(mlx90393_t *sensor)
{
    if (sensor == NULL || sensor->i2c_device == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t cmd = MLX90393_CMD_SM_XYZ;
    uint8_t status = 0;

    esp_err_t err =
        i2c_master_transmit_receive(sensor->i2c_device, &cmd, 1, &status, 1, 100);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "SM XYZ failed: %s", esp_err_to_name(err));

        return err;
    }

    ESP_LOGD(TAG, "SM XYZ status=0x%02X", status);

    return mlx90393_check_status(status);
}

/* -------------------------------------------------------------------------- */
/* Read XYZ                                                                   */
/* -------------------------------------------------------------------------- */

static esp_err_t mlx90393_read_xyz(mlx90393_t *sensor, int16_t *x, int16_t *y,
                                   int16_t *z)
{
    if (sensor == NULL || sensor->i2c_device == NULL || x == NULL || y == NULL ||
        z == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t cmd = MLX90393_CMD_RM_XYZ;
    uint8_t rx[7] = {0};

    esp_err_t err = i2c_master_transmit_receive(sensor->i2c_device, &cmd, 1, rx,
                                                sizeof(rx), 100);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "RM XYZ failed: %s", esp_err_to_name(err));

        return err;
    }

    ESP_LOGD(TAG, "RM XYZ status=0x%02X", rx[0]);

    err = mlx90393_check_status(rx[0]);

    if (err != ESP_OK)
    {
        return err;
    }

    *x = (int16_t)(((uint16_t)rx[1] << 8) | (uint16_t)rx[2]);

    *y = (int16_t)(((uint16_t)rx[3] << 8) | (uint16_t)rx[4]);

    *z = (int16_t)(((uint16_t)rx[5] << 8) | (uint16_t)rx[6]);

    ESP_LOGD(TAG, "RAW XYZ = %d %d %d", *x, *y, *z);

    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
/* Measure                                                                    */
/* -------------------------------------------------------------------------- */

esp_err_t mlx90393_measure(mlx90393_t *sensor, mag_t *data)
{
    if (sensor == NULL || sensor->i2c_device == NULL || data == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = mlx90393_start_measurement(sensor);

    if (err != ESP_OK)
    {
        return err;
    }

    /*
     * OSR=3, DIG_FILT=2.
     *
     * Keep a 20 ms margin for conversion and oscillator tolerance.
     */
    vTaskDelay(pdMS_TO_TICKS(MLX90393_XYZ_CONVERSION_TIME_MS));

    return mlx90393_read_measurement(sensor, data);
}

esp_err_t mlx90393_read_measurement(mlx90393_t *sensor, mag_t *data)
{
    if (sensor == NULL || sensor->i2c_device == NULL || data == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    int16_t raw_x;
    int16_t raw_y;
    int16_t raw_z;

    esp_err_t err = mlx90393_read_xyz(sensor, &raw_x, &raw_y, &raw_z);

    if (err != ESP_OK)
    {
        return err;
    }

    /*
     * Convert raw ADC counts to microtesla.
     *
     * Calibration offsets are stored in microtesla.
     */
    data->x = ((float)raw_x * MLX90393_XY_UT_PER_LSB) + sensor->offset_x;

    data->y = ((float)raw_y * MLX90393_XY_UT_PER_LSB) + sensor->offset_y;

    data->z = ((float)raw_z * MLX90393_Z_UT_PER_LSB) + sensor->offset_z;

    ESP_LOGD(TAG, "MAG: X=%.3f uT Y=%.3f uT Z=%.3f uT", data->x, data->y,
             data->z);

    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
/* Initialization                                                             */
/* -------------------------------------------------------------------------- */

esp_err_t mlx90393_init(i2c_master_bus_handle_t bus_handle,
                        mlx90393_t *device)
{
    if (bus_handle == NULL || device == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    /*
     * Reset software state.
     */
    *device = (mlx90393_t){0};

    /*
     * ----------------------------------------------------------------------
     * Probe address
     * ----------------------------------------------------------------------
     */

    esp_err_t err = mlx90393_probe(bus_handle);

    if (err != ESP_OK)
    {
        return err;
    }

    /*
     * ----------------------------------------------------------------------
     * Add device to existing I2C bus
     * ----------------------------------------------------------------------
     */

    const i2c_device_config_t config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MLX90393_ADDR,
        .scl_speed_hz = 100000,
    };

    err = i2c_master_bus_add_device(bus_handle, &config, &device->i2c_device);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to add MLX90393: %s", esp_err_to_name(err));

        return err;
    }

    ESP_LOGI(TAG, "MLX90393 added at I2C address 0x%02X", MLX90393_ADDR);

    /*
     * ----------------------------------------------------------------------
     * Communication test
     * ----------------------------------------------------------------------
     */

    ESP_LOGI(TAG, "Testing communication");

    err = mlx90393_nop(device);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "MLX90393 communication test failed: %s",
                 esp_err_to_name(err));

        goto fail;
    }

    /*
     * ----------------------------------------------------------------------
     * Read configuration
     * ----------------------------------------------------------------------
     *
     * IMPORTANT:
     *
     * No reset is performed here.
     *
     * The sensor has already demonstrated that it is alive and responding.
     */

    uint16_t conf1 = 0;
    uint16_t conf3 = 0;

    err = mlx90393_read_register(device, MLX90393_REG_CONF1, &conf1);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read CONF1: %s", esp_err_to_name(err));

        goto fail;
    }

    err = mlx90393_read_register(device, MLX90393_REG_CONF3, &conf3);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read CONF3: %s", esp_err_to_name(err));

        goto fail;
    }

    ESP_LOGI(TAG, "Initial configuration: CONF1=0x%04X CONF3=0x%04X", conf1,
             conf3);

    /*
     * ----------------------------------------------------------------------
     * Configure CONF1
     * ----------------------------------------------------------------------
     *
     * Preserve the upper byte.
     */
    uint16_t new_conf1 = (conf1 & 0xFF00) | MLX90393_CONF1_VALUE;

    if (new_conf1 != conf1)
    {
        ESP_LOGI(TAG, "Writing CONF1: 0x%04X -> 0x%04X", conf1, new_conf1);

        err = mlx90393_write_register(device, MLX90393_REG_CONF1, new_conf1);

        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to write CONF1: %s", esp_err_to_name(err));

            goto fail;
        }
    }
    else
    {
        ESP_LOGI(TAG, "CONF1 already configured");
    }

    /*
     * ----------------------------------------------------------------------
     * Configure CONF3
     * ----------------------------------------------------------------------
     */

    uint16_t new_conf3 = (conf3 & 0xF800) | MLX90393_CONF3_VALUE;

    if (new_conf3 != conf3)
    {
        ESP_LOGI(TAG, "Writing CONF3: 0x%04X -> 0x%04X", conf3, new_conf3);

        err = mlx90393_write_register(device, MLX90393_REG_CONF3, new_conf3);

        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to write CONF3: %s", esp_err_to_name(err));

            goto fail;
        }
    }
    else
    {
        ESP_LOGI(TAG, "CONF3 already configured");
    }

    /*
     * ----------------------------------------------------------------------
     * Verify configuration
     * ----------------------------------------------------------------------
     */

    err = mlx90393_read_register(device, MLX90393_REG_CONF1, &conf1);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to verify CONF1: %s", esp_err_to_name(err));

        goto fail;
    }

    err = mlx90393_read_register(device, MLX90393_REG_CONF3, &conf3);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to verify CONF3: %s", esp_err_to_name(err));

        goto fail;
    }

    ESP_LOGI(TAG, "Final configuration: CONF1=0x%04X CONF3=0x%04X", conf1, conf3);

    if ((conf1 & 0x00FF) != MLX90393_CONF1_VALUE)
    {
        ESP_LOGE(TAG, "CONF1 mismatch: expected 0x%02X, got 0x%02X",
                 MLX90393_CONF1_VALUE, conf1 & 0xFF);

        err = ESP_FAIL;
        goto fail;
    }

    if ((conf3 & 0x001F) != MLX90393_CONF3_VALUE)
    {
        ESP_LOGE(TAG, "CONF3 mismatch: expected 0x%04X, got 0x%04X",
                 MLX90393_CONF3_VALUE, conf3 & 0x001F);

        err = ESP_FAIL;
        goto fail;
    }

    /*
     * ----------------------------------------------------------------------
     * Final communication test
     * ----------------------------------------------------------------------
     */

    err = mlx90393_nop(device);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Final NOP failed: %s", esp_err_to_name(err));

        goto fail;
    }

    /*
     * ----------------------------------------------------------------------
     * Done
     * ----------------------------------------------------------------------
     */

    ESP_LOGI(TAG, "MLX90393 initialized successfully");

    ESP_LOGI(TAG, "Sensitivity XY=%.3f uT/LSB Z=%.3f uT/LSB",
             MLX90393_XY_UT_PER_LSB, MLX90393_Z_UT_PER_LSB);

    return ESP_OK;

fail:

    if (device->i2c_device != NULL)
    {
        i2c_master_bus_rm_device(device->i2c_device);

        device->i2c_device = NULL;
    }

    return err;
}

esp_err_t mlx90393_deinit(mlx90393_t *device)
{
    if (device == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (device->i2c_device != NULL)
    {
        esp_err_t err = i2c_master_bus_rm_device(device->i2c_device);

        if (err != ESP_OK)
        {
            return err;
        }
    }

    *device = (mlx90393_t){0};

    return ESP_OK;
}
