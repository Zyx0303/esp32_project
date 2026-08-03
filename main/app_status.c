 #include "app_status.h"

#include <string.h>

#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"

/*
 * control_task、sensor_task 会更新快照，HTTP/UART 会读取快照。临界区只做定长
 * 内存复制；取系统时间、堆大小、I2C 和日志等操作必须放在锁外。
 */
static portMUX_TYPE s_status_lock = portMUX_INITIALIZER_UNLOCKED;
static robot_status_t s_status_snapshot;

void app_status_reset(void)
{
    portENTER_CRITICAL(&s_status_lock);
    memset(&s_status_snapshot, 0, sizeof(s_status_snapshot));
    portEXIT_CRITICAL(&s_status_lock);
}

void app_status_publish_control(const robot_status_t *control_status,
                                bool drv8833_fault_active,
                                uint32_t queue_overflow_count)
{
    if (control_status == NULL) {
        return;
    }

    robot_status_t merged = *control_status;
    merged.uptime_ms = (uint32_t)(esp_timer_get_time() / 1000);
    merged.free_heap_bytes = esp_get_free_heap_size();
    merged.drv8833_fault_active = drv8833_fault_active;
    merged.queue_overflow_count = queue_overflow_count;

    portENTER_CRITICAL(&s_status_lock);
    // 传感器字段由 sensor_task 所有，发布控制状态时必须原样保留。
    merged.imu_valid = s_status_snapshot.imu_valid;
    merged.imu_ax = s_status_snapshot.imu_ax;
    merged.imu_ay = s_status_snapshot.imu_ay;
    merged.imu_az = s_status_snapshot.imu_az;
    merged.imu_gx = s_status_snapshot.imu_gx;
    merged.imu_gy = s_status_snapshot.imu_gy;
    merged.imu_gz = s_status_snapshot.imu_gz;
    merged.imu_temp_raw = s_status_snapshot.imu_temp_raw;
    merged.imu_last_update_us = s_status_snapshot.imu_last_update_us;
    merged.imu_sample_count = s_status_snapshot.imu_sample_count;
    merged.imu_error_count = s_status_snapshot.imu_error_count;
    s_status_snapshot = merged;
    portEXIT_CRITICAL(&s_status_lock);
}

void app_status_record_imu(const mpu6050_raw_t *sample, int64_t sample_time_us)
{
    portENTER_CRITICAL(&s_status_lock);
    if (sample != NULL) {
        s_status_snapshot.imu_valid = true;
        s_status_snapshot.imu_ax = sample->ax;
        s_status_snapshot.imu_ay = sample->ay;
        s_status_snapshot.imu_az = sample->az;
        s_status_snapshot.imu_gx = sample->gx;
        s_status_snapshot.imu_gy = sample->gy;
        s_status_snapshot.imu_gz = sample->gz;
        s_status_snapshot.imu_temp_raw = sample->temp_raw;
        s_status_snapshot.imu_last_update_us = sample_time_us;
        s_status_snapshot.imu_sample_count++;
    } else {
        s_status_snapshot.imu_valid = false;
        s_status_snapshot.imu_error_count++;
    }
    portEXIT_CRITICAL(&s_status_lock);
}

bool app_status_get(robot_status_t *status)
{
    if (status == NULL) {
        return false;
    }
    portENTER_CRITICAL(&s_status_lock);
    *status = s_status_snapshot;
    portEXIT_CRITICAL(&s_status_lock);
    return true;
}
