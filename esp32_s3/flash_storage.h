#pragma once

#include <stdint.h>
#include "esp_err.h"

#define IMU_DATA_MAGIC 0xDEADBEEF

typedef struct {
    uint32_t magic;
    uint32_t timestamp_ms;
    uint32_t seq;
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t temp_raw;
} imu_data_entry_t;

typedef struct {
    uint32_t seq;
    uint32_t count;
    uint32_t start_time_ms;
    uint32_t end_time_ms;
} imu_data_header_t;

esp_err_t flash_storage_init(void);
esp_err_t flash_storage_write_entry(const imu_data_entry_t *entry);
esp_err_t flash_storage_flush(void);
esp_err_t flash_storage_read_all(void (*callback)(const imu_data_entry_t *, size_t));
size_t flash_storage_get_count(void);
