#pragma once
#include "mpu6050.h"
#include "esp_http_server.h"

esp_err_t imu_log_init(void);
/* Nonblocking: only queues raw samples, never accesses flash from the sensor task. */
void imu_log_record(const mpu6050_raw_t *sample, int64_t time_us,
                    uint32_t sequence, uint32_t ready_count, esp_err_t error);
esp_err_t imu_log_register_http(httpd_handle_t server);
