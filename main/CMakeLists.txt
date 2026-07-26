idf_component_register(
    SRCS
        "app_main.c"
    INCLUDE_DIRS "."
    REQUIRES nvs_flash driver esp_timer esp32_s3
)
