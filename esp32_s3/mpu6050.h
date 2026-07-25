#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

typedef struct {
    i2c_master_dev_handle_t dev;
} mpu6050_t;

typedef struct {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t temp_raw;
} mpu6050_raw_t;

esp_err_t mpu6050_init(mpu6050_t *imu, i2c_master_bus_handle_t bus, uint8_t i2c_addr);
esp_err_t mpu6050_config_data_ready_int(mpu6050_t *imu, bool enable_latched);
esp_err_t mpu6050_read_int_status(mpu6050_t *imu, uint8_t *int_status);
esp_err_t mpu6050_read_raw(mpu6050_t *imu, mpu6050_raw_t *out);

