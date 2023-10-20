#pragma once

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nmea_parser.h"
#include <stdio.h>
#include <stdlib.h>

static const char *TAG = "gps_data";

// For gps data
#define TIME_ZONE (-8)   // Canada Time
#define YEAR_BASE (2000) // date in GPS starts from 2000

static void gps_event_handler(void *event_handler_arg,
                              esp_event_base_t event_base, int32_t event_id,
                              void *event_data);
void app_main(void);
