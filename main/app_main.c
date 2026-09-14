/*
 * ESP32-S3 机器人控制器应用入口
 *
 * 本文件只负责编排各模块的启动顺序，不包含协议解析、控制算法或底层 GPIO 操作。
 * 数据与控制方向如下：
 *
 *   HTTP / UART -> app_control 命令队列 -> control_manager -> app_actuators
 *        |                                      |
 *        +------------- app_status <------------+
 *                           ^
 *                           |
 *                        app_imu
 *
 * 这种边界保证所有执行器只有 control_task 一个运行时写入者；网络和串口只负责
 * 产生命令及读取状态快照，从结构上避免多个 FreeRTOS 任务同时操作电机或舵机。
 */
#include "app_actuators.h"
#include "app_console.h"
#include "app_control.h"
#include "app_imu.h"
#include "imu_log.h"
#include "app_status.h"
#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "robot_config.h"
#include "wifi_control.h"

static const char *TAG = "app";

/**
 * 初始化 NVS。
 *
 * 仅在分区页耗尽或版本不兼容时擦除并重建；普通重启不会清空已保存的机器人参数。
 */
static void init_nvs(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

void app_main(void)
{
    ESP_LOGI(TAG, "=== ESP32-S3 Robot Controller ===");
    ESP_LOGI(TAG, "Build: %s %s", __DATE__, __TIME__);

    /*
     * 阶段 1：建立共享状态，并尽早把外部电源控制脚置于安全电平。
     * 这一步必须早于网络、IMU 等非关键服务，防止较慢的启动过程误触发外部负载。
     */
    app_status_reset();
    ESP_ERROR_CHECK(app_actuators_init_safe_power());

    /* 阶段 2：建立本地诊断通道和持久化配置。 */
    ESP_ERROR_CHECK(app_console_init());
    init_nvs();

    robot_config_t stored_config;
    bool used_defaults = false;
    ESP_ERROR_CHECK(robot_config_load(&stored_config, &used_defaults));
    if (used_defaults) {
        ESP_LOGW(TAG, "using validated safe default configuration");
    }

    /*
     * 阶段 3：初始化电机和舵机驱动，但保持禁止输出。
     * 单个执行器初始化失败时仍允许系统启动用于诊断；模块会阻止访问不可用设备。
     */
    app_actuators_init_motion(&stored_config);

    /*
     * 阶段 4：创建安全状态机、命令队列和唯一的执行器所有者 control_task。
     * boot_complete 只把 BOOT 转为 SAFE，不会自动 ARM 或恢复上次运动状态。
     */
    ESP_ERROR_CHECK(app_control_start(&stored_config));

    /*
     * 阶段 5：启动 IMU 采样。I2C 或器件不可用属于可诊断降级；
     * 但传感器已经就绪却无法创建任务，说明系统内存不足，必须终止启动。
     */
    const esp_err_t log_err = imu_log_init();
    if (log_err != ESP_OK) ESP_LOGW(TAG, "IMU logging unavailable: %s", esp_err_to_name(log_err));
    const esp_err_t imu_err = app_imu_start();
    if (imu_err == ESP_ERR_NO_MEM) {
        ESP_ERROR_CHECK(imu_err);
    } else if (imu_err != ESP_OK) {
        ESP_LOGW(TAG, "IMU unavailable: %s", esp_err_to_name(imu_err));
    }

    /*
     * 阶段 6：最后开放 Wi-Fi/HTTP 控制入口。
     * 网络层只获得“命令入队”和“状态快照”两个回调，无法绕过控制状态机操作硬件。
     */
    ESP_ERROR_CHECK(wifi_control_init(app_control_enqueue, app_status_get));
    ESP_LOGI(TAG, "SAFE: connect to %s, open http://%s, then ARM explicitly",
             WIFI_CONTROL_AP_SSID, WIFI_CONTROL_AP_IP);
    ESP_LOGI(TAG, "type 'help' for serial commands");

    /* app_main 的剩余生命周期只用于低开销串口行解析，其他工作均由独立任务承担。 */
    app_console_run();
}
