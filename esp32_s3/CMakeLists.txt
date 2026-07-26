idf_component_register(
    SRCS
        "board_pins.c"
        "mpu6050.c"
        "drv8833.c"
        "servo_pwm.c"
        "flash_storage.c"
        "wifi_control.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_timer nvs_flash
             esp_wifi esp_event esp_netif esp_http_server
)
