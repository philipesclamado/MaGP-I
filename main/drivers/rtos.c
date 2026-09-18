#include "rtos.h"

#include "bmp390.h"
#include "mlx90393.h"

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
/* Sensor task                                                                */
/* -------------------------------------------------------------------------- */

#define SENSOR_TASK_NAME "bmp390"
#define SENSOR_TASK_STACK_SIZE 4096
#define SENSOR_TASK_PRIORITY 1

#define SENSOR_RATE_HZ 25
#define SENSOR_PERIOD_MS (1000 / SENSOR_RATE_HZ)

#define FLIGHT_TASK_NAME "flight_core"
#define FLIGHT_TASK_STACK_SIZE 4096
#define FLIGHT_TASK_PRIORITY 15 // High priority to prevent preemptive drops

static TaskHandle_t s_flight_task = NULL;
static TaskHandle_t s_sensor_task = NULL;
static i2c_master_bus_handle_t s_i2c_bus = NULL;
static bmp390_t s_bmp390;
static mlx90393_t s_mlx90393;

/* -------------------------------------------------------------------------- */
/* I2C                                                                        */
/* -------------------------------------------------------------------------- */

static esp_err_t i2c_init(void) {
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

static void i2c_deinit(void) {
  if (s_i2c_bus != NULL) {
    i2c_del_master_bus(s_i2c_bus);
    s_i2c_bus = NULL;
  }
}

static void flight_task(void *arg) {
  (void)arg;

  esp_err_t err = mlx90393_init(&s_mlx90393, s_i2c_bus);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "BMP390 initialization failed: %s", esp_err_to_name(err));
    goto cleanup;
  }

  ESP_LOGI(TAG, "Master flight control core successfully spawned on Core 1.");

  while (true) {
    // Dynamic Multi-rate polling loop
    vTaskDelay(pdMS_TO_TICKS(10)); // Temporary placeholder delay
  }

cleanup:
  i2c_deinit();
  s_flight_task = NULL;
  vTaskDelete(NULL);
}

/* -------------------------------------------------------------------------- */
/* Sensor task                                                                */
/* -------------------------------------------------------------------------- */

// static void sensor_task(void *arg)
// {
//   (void)arg;

//   esp_err_t err = i2c_init();

//   if (err != ESP_OK)
//   {
//     ESP_LOGE(TAG, "I2C initialization failed: %s", esp_err_to_name(err));

//     goto cleanup;
//   }

//   err = bmp390_init(s_i2c_bus, &s_bmp390);

//   if (err != ESP_OK)
//   {
//     ESP_LOGE(TAG, "BMP390 initialization failed: %s", esp_err_to_name(err));

//     goto cleanup;
//   }

//   ESP_LOGI(TAG, "BMP390 task started at %d Hz", SENSOR_RATE_HZ);

//   /*
//    * Keep the task on a fixed 25 Hz schedule.
//    *
//    * vTaskDelayUntil() avoids accumulating the execution time of
//    * bmp390_read() and logging into the next period.
//    */
//   TickType_t last_wake = xTaskGetTickCount();

//   while (true)
//   {
//     float pressure_hpa = 0.0f;
//     float temperature_c = 0.0f;

//     err = bmp390_read(&s_bmp390, &pressure_hpa, &temperature_c);

//     if (err == ESP_OK)
//     {
//       ESP_LOGI(TAG, "Pressure: %.2f hPa | Temperature: %.2f C", pressure_hpa,
//                temperature_c);
//     }
//     else
//     {
//       ESP_LOGW(TAG, "BMP390 read failed: %s", esp_err_to_name(err));
//     }

//     vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SENSOR_PERIOD_MS));
//   }

// cleanup:

//   bmp390_deinit(&s_bmp390);
//   dei2c_init();

//   s_sensor_task = NULL;

//   vTaskDelete(NULL);
// }

esp_err_t rtos_init(void) {
  if (s_sensor_task != NULL) {
    return ESP_ERR_INVALID_STATE;
  }

  esp_err_t err = i2c_init();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize global I2C layout: %s",
             esp_err_to_name(err));
    return err;
  }

  // Pin the mission critical loop straight to Core 1 (APP_CPU)
  BaseType_t result = xTaskCreatePinnedToCore(
      flight_task, FLIGHT_TASK_NAME, FLIGHT_TASK_STACK_SIZE, NULL,
      FLIGHT_TASK_PRIORITY, &s_flight_task,
      1 // <--- STRICT CORE 1 PINNING
  );

  if (result != pdPASS) {
    i2c_deinit();
    s_flight_task = NULL;
    return ESP_ERR_NO_MEM;
  }

  return ESP_OK;
}

esp_err_t rtos_deinit(void) {
  if (s_flight_task != NULL) {
    vTaskDelete(s_flight_task);
    s_flight_task = NULL;
  }
  i2c_deinit();
  return ESP_OK;
}
