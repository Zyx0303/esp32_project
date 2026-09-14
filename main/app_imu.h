#pragma once

#include "esp_err.h"

/**
 * 初始化 I2C/MPU6050 和 GPIO8 DATA_READY 中断，启动 100 Hz 采样任务。
 *
 * I2C 或传感器不存在时返回对应错误，调用者可以让系统降级运行；
 * 任务内存不足则返回 ESP_ERR_NO_MEM，应视为系统启动失败。
 */
esp_err_t app_imu_start(void);

/** 打印状态快照中的最近一帧 IMU 数据，不在调用线程中直接访问 I2C。 */
void app_imu_log_latest(void);
