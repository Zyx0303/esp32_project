#include "app_control.h"

#include <inttypes.h>

#include "app_actuators.h"
#include "app_status.h"
#include "control_manager.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

static const char *TAG = "app_control";

#define COMMAND_QUEUE_LENGTH 16
#define CONTROL_PERIOD_MS 10

static control_manager_t s_control_manager;
static QueueHandle_t s_command_queue;

/*
 * 命令生产者可能来自不同 HTTP/UART 任务，因此溢出标志和计数使用独立短临界区。
 * 故障状态本身仍只由 control_task 修改。
 */
static portMUX_TYPE s_queue_lock = portMUX_INITIALIZER_UNLOCKED;
static bool s_queue_overflow;
static uint32_t s_queue_overflow_count;

static bool is_priority_command(robot_command_type_t type)
{
    return type == ROBOT_CMD_ESTOP || type == ROBOT_CMD_DISARM;
}

esp_err_t app_control_enqueue(const robot_command_t *command)
{
    if (s_command_queue == NULL || command == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    BaseType_t queued = is_priority_command(command->type)
                            ? xQueueSendToFront(s_command_queue, command, 0)
                            : xQueueSend(s_command_queue, command, 0);
    if (queued == pdTRUE) {
        return ESP_OK;
    }

    if (is_priority_command(command->type)) {
        // 满队列不能阻挡安全命令：丢弃普通待执行命令，只保留 ESTOP/DISARM。
        xQueueReset(s_command_queue);
        return xQueueSendToFront(s_command_queue, command, 0) == pdTRUE ? ESP_OK : ESP_FAIL;
    }

    portENTER_CRITICAL(&s_queue_lock);
    s_queue_overflow = true;
    s_queue_overflow_count++;
    portEXIT_CRITICAL(&s_queue_lock);
    return ESP_ERR_NO_MEM;
}

static void publish_control_status(bool drv8833_fault_active)
{
    robot_status_t control_status;
    control_manager_get_status(&s_control_manager, &control_status);

    portENTER_CRITICAL(&s_queue_lock);
    uint32_t overflow_count = s_queue_overflow_count;
    portEXIT_CRITICAL(&s_queue_lock);
    app_status_publish_control(&control_status, drv8833_fault_active, overflow_count);
}

/**
 * 唯一执行器运行时所有者。每 10 ms 消费命令、同步硬件故障、推进控制算法并发布状态。
 */
static void control_task(void *argument)
{
    (void)argument;
    TickType_t last_wake = xTaskGetTickCount();
    bool drv8833_fault_active = false;

    while (true) {
        robot_command_t command;
        for (int handled = 0; handled < COMMAND_QUEUE_LENGTH &&
                              xQueueReceive(s_command_queue, &command, 0) == pdTRUE;
             ++handled) {
            robot_command_result_t result = control_manager_submit(
                &s_control_manager, &command, esp_timer_get_time());
            if (result != ROBOT_COMMAND_ACCEPTED) {
                ESP_LOGW(TAG, "command type=%d seq=%" PRIu32 " rejected=%d",
                         command.type, command.sequence, result);
            }
        }

        bool queue_overflow;
        portENTER_CRITICAL(&s_queue_lock);
        queue_overflow = s_queue_overflow;
        s_queue_overflow = false;
        portEXIT_CRITICAL(&s_queue_lock);
        if (queue_overflow) {
            control_manager_report_hardware_fault(&s_control_manager,
                                                  ROBOT_FAULT_QUEUE_OVERFLOW);
        }

        if (app_actuators_motor_available()) {
            bool fault_active = false;
            esp_err_t fault_err = app_actuators_read_motor_fault(&fault_active);
            drv8833_fault_active = fault_err != ESP_OK || fault_active;
            if (drv8833_fault_active) {
                control_manager_report_hardware_fault(&s_control_manager,
                                                      ROBOT_FAULT_DRV8833);
            } else {
                control_manager_clear_hardware_fault(&s_control_manager,
                                                     ROBOT_FAULT_DRV8833);
            }
        }

        // 无新命令时也必须 tick，以推进斜坡、换向死区和通信看门狗。
        esp_err_t tick_result = control_manager_tick(&s_control_manager,
                                                     esp_timer_get_time());
        if (tick_result != ESP_OK && tick_result != ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "control tick failed: %s", esp_err_to_name(tick_result));
        }
        publish_control_status(drv8833_fault_active);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}

static robot_control_config_t make_control_config(const robot_config_t *stored)
{
    robot_control_config_t config = control_manager_default_config();
    config.command_timeout_ms = stored->command_timeout_ms;
    config.command_max_age_ms = stored->command_timeout_ms;
    config.reverse_deadtime_ms = stored->motor_reverse_deadtime_ms;
    config.motor_ramp_percent_per_10ms = stored->motor_ramp_step_percent;
    config.invert_motor_a = stored->invert_motor_a;
    config.invert_motor_b = stored->invert_motor_b;
    config.servo_min_deg = stored->servo_min_angle_deg;
    config.servo_max_deg = stored->servo_max_angle_deg;
    config.servo_initial_deg = stored->servo_center_angle_deg;
    config.servo_slew_deg_per_second = stored->servo_slew_deg_per_second;
    return config;
}

esp_err_t app_control_start(const robot_config_t *config)
{
    ESP_RETURN_ON_FALSE(config != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "validated configuration required");

    robot_control_config_t control_config = make_control_config(config);
    ESP_RETURN_ON_ERROR(control_manager_init(&s_control_manager, &control_config,
                                             app_actuators_get_ops(),
                                             app_actuators_get_context()),
                        TAG, "initialize control manager");
    // BOOT -> SAFE，不会自动 ARM，也不会恢复任何历史运动命令。
    ESP_RETURN_ON_ERROR(control_manager_boot_complete(&s_control_manager),
                        TAG, "complete safe boot");
    publish_control_status(false);

    s_command_queue = xQueueCreate(COMMAND_QUEUE_LENGTH, sizeof(robot_command_t));
    ESP_RETURN_ON_FALSE(s_command_queue != NULL, ESP_ERR_NO_MEM, TAG,
                        "create command queue");
    BaseType_t created = xTaskCreatePinnedToCore(control_task, "control_task", 6144,
                                                 NULL, 8, NULL, 1);
    ESP_RETURN_ON_FALSE(created == pdPASS, ESP_ERR_NO_MEM, TAG,
                        "create control task");
    return ESP_OK;
}
