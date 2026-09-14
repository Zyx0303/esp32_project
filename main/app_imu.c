#include "app_imu.h"

#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "app_status.h"
#include "imu_log.h"
#include "driver/gpio.h"
#include "board_pins.h"
#include "mpu6050.h"
#include "robot_types.h"

static const char *TAG = "app_imu";

static i2c_master_bus_handle_t s_i2c_bus;
static mpu6050_t s_imu;
static TaskHandle_t s_sensor_task;

static void imu_ready_isr(void *argument)
{
    (void)argument;
    BaseType_t wake = pdFALSE;
    vTaskNotifyGiveFromISR(s_sensor_task, &wake);
    if (wake) portYIELD_FROM_ISR();
}

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
 * DATA_READY 通知后采样，更新共享快照并非阻塞地入日志队列。
 * HTTP 和串口读取快照，不会与本任务争用总线。
 * 读取失败时保留历史计数，同时把 imu_valid 清零，避免上位机误用旧数据。
 */
static void sensor_task(void *argument)
{
    (void)argument;
    uint32_t sequence = 0;
    while (true) {
        uint32_t ready = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));
        int64_t sample_time_us = esp_timer_get_time();
        mpu6050_raw_t sample = {0};
        esp_err_t err = ready ? mpu6050_read_raw(&s_imu, &sample) : ESP_ERR_TIMEOUT;
        sequence += ready ? ready : 1;
        app_status_record_imu(err == ESP_OK ? &sample : NULL, sample_time_us);
        imu_log_record(&sample, sample_time_us, sequence, ready, err);
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

    gpio_config_t interrupt_config = {
        .pin_bit_mask = 1ULL << PIN_MPU_INT,
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    err = gpio_config(&interrupt_config);
    if (err != ESP_OK) return err;
    err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;

    const BaseType_t created = xTaskCreatePinnedToCore(
        sensor_task, "sensor_task", 4096, NULL, 5, &s_sensor_task, 0);
    if (created != pdPASS) return ESP_ERR_NO_MEM;
    err = gpio_isr_handler_add(PIN_MPU_INT, imu_ready_isr, NULL);
    if (err == ESP_OK) err = mpu6050_config_data_ready_int(&s_imu, false);
    if (err != ESP_OK) {
        gpio_isr_handler_remove(PIN_MPU_INT);
        vTaskDelete(s_sensor_task);
        s_sensor_task = NULL;
    }
    return err;
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
