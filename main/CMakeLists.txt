idf_component_register(
    SRCS
        "app_main.c"
        "app_status.c"
        "app_actuators.c"
        "app_control.c"
        "app_imu.c"
        "app_console.c"
    INCLUDE_DIRS "."
    REQUIRES nvs_flash driver esp_timer robot_controller
)
