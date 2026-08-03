#include "app_imu.h"

#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "app_status.h"
#include "board_pins.h"
#include "mpu6050.h"
#include "robot_types.h"

static const char *TAG = "app_imu";

static i2c_master_bus_handle_t s_i2c_bus;
static mpu6050_t s_imu;

/**
 * 创建 MPU6050 使用的 I2C 主总线。
 *
 * 芯片内部上拉仅用于辅助；正式 PCB 仍应使用满足 400 kHz 时序的外部上拉。
 * 引脚全部来自 board_pins.h，避免硬件版本变更时出现散落的 GPIO 常量。
 */
static esp_err_t init_i2c_bus(void)
{
    const i2c_master_bus_config_t config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = PIN_I2C_SDA,
        .scl_io_num = PIN_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    return i2c_new_master_bus(&config, &s_i2c_bus);
}

/**
 * IMU 的唯一 I2C 访问者。
 *
 * 采样完成后只把结果写入共享状态快照；HTTP 和串口读取快照，不会与本任务争用总线。
 * 读取失败时保留历史计数，同时把 imu_valid 清零，避免上位机误用旧数据。
 */
static void sensor_task(void *argument)
{
    (void)argument;
    TickType_t last_wake = xTaskGetTickCount();

    while (true) {
        mpu6050_raw_t sample;
        const esp_err_t err = mpu6050_read_raw(&s_imu, &sample);
        const int64_t sample_time_us = esp_timer_get_time();
        app_status_record_imu(err == ESP_OK ? &sample : NULL, sample_time_us);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));
    }
}

esp_err_t app_imu_start(void)
{
    esp_err_t err = init_i2c_bus();
    if (err != ESP_OK) {
        return err;
    }

    err = mpu6050_init(&s_imu, s_i2c_bus, MPU6050_I2C_ADDR);
    if (err != ESP_OK) {
        return err;
    }

    const BaseType_t created = xTaskCreatePinnedToCore(
        sensor_task, "sensor_task", 4096, NULL, 5, NULL, 0);
    return created == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}

void app_imu_log_latest(void)
{
    robot_status_t status;
    if (!app_status_get(&status) || !status.imu_valid) {
        ESP_LOGW(TAG, "MPU6050 unavailable or stale");
        return;
    }

    ESP_LOGI(TAG, "IMU accel=(%d,%d,%d) gyro=(%d,%d,%d) temp=%d samples=%" PRIu32,
             status.imu_ax, status.imu_ay, status.imu_az,
             status.imu_gx, status.imu_gy, status.imu_gz,
             status.imu_temp_raw, status.imu_sample_count);
}
