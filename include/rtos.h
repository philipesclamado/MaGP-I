#pragma once

#include "esp_err.h"

/*
 * @brief Initialize application RTOS tasks.
 *
 * Creates the sensor task responsible for initializing and polling
 * the BMP390.
 *
 * @return
 *   - ESP_OK on success
 *   - ESP_ERR_INVALID_STATE if already initialized
 *   - ESP_ERR_NO_MEM if the task could not be created
 */
esp_err_t rtos_init(void);

/*
 * @brief Stop application RTOS tasks and release owned resources.
 *
 * Safe to call if RTOS initialization failed or was never performed.
 *
 * @return ESP_OK on success.
 */
esp_err_t rtos_deinit(void);
