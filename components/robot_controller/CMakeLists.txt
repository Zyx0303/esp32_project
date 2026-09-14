idf_component_register(
    SRCS
        "mpu6050.c"
        "imu_log.c"
        "drv8833.c"
        "servo_pwm.c"
        "power_control.c"
        "robot_config.c"
        "robot_state_machine.c"
        "control_manager.c"
        "wifi_control.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_timer nvs_flash json
             esp_wifi esp_event esp_netif esp_http_server fatfs wear_levelling esp_partition
)
