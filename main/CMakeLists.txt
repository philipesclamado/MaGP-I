idf_component_register(
    SRCS
        "MaGP-I.c"
        "drivers/rtos.c"
        "drivers/nmea_parser.c"
        "drivers/bmp390.c"
        "drivers/mpu6050.c"

    INCLUDE_DIRS
        "../include"

    REQUIRES
        esp_driver_i2c
        esp_driver_uart
        esp_event
        freertos
)
