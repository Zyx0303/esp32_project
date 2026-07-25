idf_component_register(
    SRCS
        "board_pins.c"
        "mpu6050.c"
        "drv8833.c"
        "servo_pwm.c"
        "flash_storage.c"
        "ble_control.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_gpio esp_driver_i2c esp_driver_ledc esp_timer nvs_flash bt
)
