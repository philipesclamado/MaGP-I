#include "rtos.h"

#include <stdbool.h>

#include "bmp390.h"
#include "mlx90393.h"
#include "mpu6050.h"

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "rtos";

/* -------------------------------------------------------------------------- */
/* I2C                                                                        */
/* -------------------------------------------------------------------------- */

#define I2C_PORT I2C_NUM_0
#define I2C_SDA_GPIO 21
#define I2C_SCL_GPIO 22

/* -------------------------------------------------------------------------- */
/* System rates                                                               */
/* -------------------------------------------------------------------------- */

/*
 * Flight/control loop:
 * 100 Hz = one iteration every 10 ms.
 */
#define FLIGHT_RATE_HZ 100U
#define FLIGHT_PERIOD_MS (1000U / FLIGHT_RATE_HZ)

/*
 * The magnetometer and barometer each supply a fresh sample at 50 Hz.  Their
 * latest verified value is held between updates for the 100 Hz estimator.
 */
#define BMP390_RATE_HZ 50U
#define BMP390_PERIOD_LOOPS (FLIGHT_RATE_HZ / BMP390_RATE_HZ)

#define IMU_RATE_HZ FLIGHT_RATE_HZ
#define MAG_RATE_HZ 50U
#define MAG_PERIOD_LOOPS (FLIGHT_RATE_HZ / MAG_RATE_HZ)

/* -------------------------------------------------------------------------- */
/* Tasks                                                                      */
/* -------------------------------------------------------------------------- */

#define FLIGHT_TASK_NAME "flight_core"
#define FLIGHT_TASK_STACK_SIZE 4096
#define FLIGHT_TASK_PRIORITY 15

static TaskHandle_t s_flight_task = NULL;

static i2c_master_bus_handle_t s_i2c_bus = NULL;

static bmp390_t s_bmp390;
static mpu6050_t s_mpu6050;
static mlx90393_t s_mlx90393;

/* -------------------------------------------------------------------------- */
/* I2C                                                                        */
/* -------------------------------------------------------------------------- */

static esp_err_t i2c_init(void)
{
  const i2c_master_bus_config_t config = {
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .i2c_port = I2C_PORT,
      .sda_io_num = I2C_SDA_GPIO,
      .scl_io_num = I2C_SCL_GPIO,
      .glitch_ignore_cnt = 7,
      .flags.enable_internal_pullup = true,
  };

  return i2c_new_master_bus(&config, &s_i2c_bus);
}

static void i2c_deinit(void)
{
  if (s_i2c_bus != NULL)
  {
    i2c_del_master_bus(s_i2c_bus);
    s_i2c_bus = NULL;
  }
}

/* -------------------------------------------------------------------------- */
/* Flight task                                                                */
/* -------------------------------------------------------------------------- */

static void flight_task(void *arg)
{
  (void)arg;

  esp_err_t err;

  /* ---------------------------------------------------------------------- */
  /* Sensor initialization                                                  */
  /* ---------------------------------------------------------------------- */

  err = bmp390_init(s_i2c_bus, &s_bmp390);

  if (err != ESP_OK)
  {
    ESP_LOGE(TAG, "BMP390 initialization failed: %s", esp_err_to_name(err));

    goto cleanup;
  }

  err = mpu6050_init(s_i2c_bus, &s_mpu6050);

  if (err != ESP_OK)
  {
    ESP_LOGE(TAG, "MPU-6050 initialization failed: %s", esp_err_to_name(err));

    goto cleanup;
  }

  err = mlx90393_init(s_i2c_bus, &s_mlx90393);

  if (err != ESP_OK)
  {
    ESP_LOGE(TAG, "MLX90393 initialization failed: %s", esp_err_to_name(err));

    goto cleanup;
  }

  ESP_LOGI(TAG, "Flight task started");

  ESP_LOGI(TAG, "Rates: IMU=%u Hz, MAG=%u Hz, BMP390=%u Hz", IMU_RATE_HZ,
           MAG_RATE_HZ, BMP390_RATE_HZ);

  /* ---------------------------------------------------------------------- */
  /* Timing                                                                 */
  /* ---------------------------------------------------------------------- */

  TickType_t last_wake = xTaskGetTickCount();

  const TickType_t loop_period = pdMS_TO_TICKS(FLIGHT_PERIOD_MS);

  uint32_t loop_count = 0;

  /* ---------------------------------------------------------------------- */
  /* Sensor data                                                             */
  /* ---------------------------------------------------------------------- */

  float pressure = 0.0f;
  float temperature = 0.0f;

  accel_t accel = {0};
  gyro_t gyro = {0};
  mag_t mag = {0};
  bool mag_valid = false;
  bool mag_conversion_pending = false;
  bool baro_valid = false;

  /* ---------------------------------------------------------------------- */
  /* Main 100 Hz flight loop                                                */
  /* ---------------------------------------------------------------------- */

  while (true)
  {
    /*
     * Maintain a precise 100 Hz loop.
     */
    vTaskDelayUntil(&last_wake, loop_period);

    /* ------------------------------------------------------------------ */
    /* IMU: 100 Hz                                                        */
    /* ------------------------------------------------------------------ */

    err = mpu6050_measure(&s_mpu6050, &accel, &gyro);

    if (err != ESP_OK)
    {
      ESP_LOGW(TAG, "MPU-6050 read failed: %s", esp_err_to_name(err));

      /*
       * IMU failure is currently treated as a failed flight-loop
       * iteration because the control/estimation system depends on it.
       */
      continue;
    }

    /* ------------------------------------------------------------------ */
    /* Magnetometer: 50 Hz, non-blocking single-conversion pipeline      */
    /* ------------------------------------------------------------------ */

    if ((loop_count % MAG_PERIOD_LOOPS) == 0U)
    {
      if (mag_conversion_pending)
      {
        err = mlx90393_read_measurement(&s_mlx90393, &mag);

        if (err == ESP_OK)
        {
          mag_valid = true;
        }
        else
        {
          ESP_LOGW(TAG, "MLX90393 read failed: %s", esp_err_to_name(err));
        }
      }

      err = mlx90393_start_measurement(&s_mlx90393);

      if (err == ESP_OK)
      {
        mag_conversion_pending = true;
      }
      else
      {
        mag_conversion_pending = false;
        ESP_LOGW(TAG, "MLX90393 start failed: %s", esp_err_to_name(err));
      }
    }

    /* ------------------------------------------------------------------ */
    /* BMP390: 50 Hz                                                      */
    /* ------------------------------------------------------------------ */

    if ((loop_count % BMP390_PERIOD_LOOPS) == 0)
    {
      err = bmp390_measure(&s_bmp390, &pressure, &temperature);

      if (err == ESP_OK)
      {
        baro_valid = true;
      }
      else if (err != ESP_ERR_NOT_FINISHED)
      {
        ESP_LOGW(TAG, "BMP390 read failed: %s", esp_err_to_name(err));
      }
    }

    /* ------------------------------------------------------------------ */
    /* Guidance / estimation / control                                    */
    /* ------------------------------------------------------------------ */

    /*
     * Your state estimation and guidance logic goes here.
     *
     * At this point:
     *
     *   accel / gyro       -> newest 100 Hz IMU sample
     *   mag                -> newest verified 50 Hz magnetometer sample
     *   pressure           -> newest verified 50 Hz BMP390 sample
     *   temperature        -> newest verified 50 Hz BMP390 sample
     *
     * The BMP390 values intentionally remain unchanged on the
     * intermediate 100 Hz iteration.
     */

    /*
     * Example:
     *
     * state_estimator_update(
     *     &accel,
     *     &gyro,
     *     &mag,
     *     pressure,
     *     temperature);
     *
     * guidance_update(...);
     * actuator_update(...);
     */

    /* ------------------------------------------------------------------ */
    /* Debug output                                                       */
    /* ------------------------------------------------------------------ */

    /*
     * Print at 5 Hz rather than every loop.
     */
    if ((loop_count % 20U) == 0U)
    {
      ESP_LOGI(TAG,
               "A [%.3f %.3f %.3f] g | "
               "G [%.2f %.2f %.2f] dps | "
               "M [%.1f %.1f %.1f] uT (%s) | "
               "P %.2f hPa | "
               "T %.2f C (%s)",

               accel.x / 4096.0f, accel.y / 4096.0f, accel.z / 4096.0f,

               gyro.x / 16.4f, gyro.y / 16.4f, gyro.z / 16.4f,

               mag.x, mag.y, mag.z,

               mag_valid ? "valid" : "waiting",

               pressure, temperature, baro_valid ? "valid" : "waiting");
    }

    loop_count++;
  }

cleanup:

  /*
   * Deinitialize sensors before deleting the I2C bus.
   */
  bmp390_deinit(&s_bmp390);
  mpu6050_deinit(&s_mpu6050);
  mlx90393_deinit(&s_mlx90393);

  i2c_deinit();

  s_flight_task = NULL;

  vTaskDelete(NULL);
}

/* -------------------------------------------------------------------------- */
/* Initialization                                                             */
/* -------------------------------------------------------------------------- */

esp_err_t rtos_init(void)
{
  /*
   * Prevent starting the flight task twice.
   */
  if (s_flight_task != NULL)
  {
    return ESP_ERR_INVALID_STATE;
  }

  /* ---------------------------------------------------------------------- */
  /* I2C                                                                     */
  /* ---------------------------------------------------------------------- */

  esp_err_t err = i2c_init();

  if (err != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to initialize global I2C bus: %s",
             esp_err_to_name(err));

    return err;
  }

  /* ---------------------------------------------------------------------- */
  /* Flight task                                                             */
  /* ---------------------------------------------------------------------- */

  BaseType_t result = xTaskCreatePinnedToCore(
      flight_task, FLIGHT_TASK_NAME, FLIGHT_TASK_STACK_SIZE, NULL,
      FLIGHT_TASK_PRIORITY, &s_flight_task, 1);

  if (result != pdPASS)
  {
    i2c_deinit();

    s_flight_task = NULL;

    return ESP_ERR_NO_MEM;
  }

  return ESP_OK;
}

/* -------------------------------------------------------------------------- */
/* Deinitialization                                                           */
/* -------------------------------------------------------------------------- */

esp_err_t rtos_deinit(void)
{
  if (s_flight_task != NULL)
  {
    vTaskDelete(s_flight_task);
    s_flight_task = NULL;
  }

  /*
   * The task normally owns sensor cleanup, but if rtos_deinit()
   * is called externally while the task is still alive, delete the
   * bus after deleting the task.
   */
  i2c_deinit();

  return ESP_OK;
}
