idf_component_register(
    SRCS
        "app_main.c"
    INCLUDE_DIRS "."
    REQUIRES nvs_flash esp_driver_uart esp_driver_gpio esp_driver_ledc driver esp_timer esp32_s3
)
