#include "app_control.h"

#include <inttypes.h>

#include "app_actuators.h"
#include "app_status.h"
#include "control_manager.h"
#include "control_strategy.h"
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
static const robot_control_strategy_ops_t *s_strategy_ops;
static void *s_strategy_context;
static bool s_strategy_active;
static int64_t s_strategy_last_step_us;
static uint32_t s_strategy_sequence;

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

esp_err_t app_control_register_strategy(const robot_control_strategy_ops_t *ops,
                                        void *context)
{
    /* 注册只允许发生在任务启动前，避免运行时替换回调产生跨核竞态。 */
    if (s_command_queue != NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (ops != NULL && ops->step == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    s_strategy_ops = ops;
    s_strategy_context = context;
    return ESP_OK;
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

static void reset_strategy_if_active(void)
{
    if (s_strategy_active && s_strategy_ops != NULL && s_strategy_ops->reset != NULL) {
        s_strategy_ops->reset(s_strategy_context);
    }
    s_strategy_active = false;
    s_strategy_last_step_us = 0;
}

static esp_err_t submit_strategy_command(robot_command_t *command, int64_t now_us)
{
    command->source = ROBOT_COMMAND_SOURCE_INTERNAL;
    command->sequence = ++s_strategy_sequence;
    command->received_at_us = now_us;
    const robot_command_result_t result = control_manager_submit(
        &s_control_manager, command, now_us);
    if (result == ROBOT_COMMAND_ACCEPTED) {
        return ESP_OK;
    }
    ESP_LOGE(TAG, "strategy command type=%d rejected=%d", command->type, result);
    control_manager_report_hardware_fault(&s_control_manager, ROBOT_FAULT_INTERNAL);
    return ESP_FAIL;
}

/**
 * 在唯一的执行器所有者 control_task 内运行可选策略。
 *
 * 策略默认不存在；存在时也只在 ARM 后执行。每次成功 step 必定提交一个电机目标，
 * output 的零初始化使未填写的策略安全地提交停止，而不是沿用未知的旧运动目标。
 */
static void run_registered_strategy(int64_t now_us)
{
    if (s_strategy_ops == NULL) {
        return;
    }

    robot_status_t current;
    control_manager_get_status(&s_control_manager, &current);
    if (!current.armed) {
        reset_strategy_if_active();
        return;
    }

    if (!s_strategy_active) {
        if (s_strategy_ops->reset != NULL) {
            s_strategy_ops->reset(s_strategy_context);
        }
        s_strategy_active = true;
    }

    robot_strategy_input_t input = {
        .now_us = now_us,
        .delta_us = s_strategy_last_step_us == 0
                        ? CONTROL_PERIOD_MS * 1000LL
                        : now_us - s_strategy_last_step_us,
        .status = current,
    };
    s_strategy_last_step_us = now_us;

    /* 控制字段使用本周期当前值；只从发布快照补入 IMU 和系统观测字段。 */
    robot_status_t published;
    if (app_status_get(&published)) {
        input.status.drv8833_fault_active = published.drv8833_fault_active;
        input.status.imu_valid = published.imu_valid;
        input.status.imu_ax = published.imu_ax;
        input.status.imu_ay = published.imu_ay;
        input.status.imu_az = published.imu_az;
        input.status.imu_gx = published.imu_gx;
        input.status.imu_gy = published.imu_gy;
        input.status.imu_gz = published.imu_gz;
        input.status.imu_temp_raw = published.imu_temp_raw;
        input.status.imu_last_update_us = published.imu_last_update_us;
        input.status.imu_sample_count = published.imu_sample_count;
        input.status.imu_error_count = published.imu_error_count;
        input.status.uptime_ms = published.uptime_ms;
        input.status.free_heap_bytes = published.free_heap_bytes;
        input.status.queue_overflow_count = published.queue_overflow_count;
    }

    robot_strategy_output_t output = {0};
    const esp_err_t err = s_strategy_ops->step(s_strategy_context, &input, &output);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "strategy %s failed: %s",
                 s_strategy_ops->name != NULL ? s_strategy_ops->name : "unnamed",
                 esp_err_to_name(err));
        control_manager_report_hardware_fault(&s_control_manager, ROBOT_FAULT_INTERNAL);
        reset_strategy_if_active();
        return;
    }

    robot_command_t motor_command = {0};
    switch (output.motor_mode) {
    case ROBOT_STRATEGY_MOTOR_STOP:
        motor_command.type = ROBOT_CMD_MOTOR_DIRECT;
        break;
    case ROBOT_STRATEGY_MOTOR_DRIVE:
        motor_command.type = ROBOT_CMD_DRIVE;
        motor_command.value.drive.throttle = output.motor.drive.throttle;
        motor_command.value.drive.steering = output.motor.drive.steering;
        break;
    case ROBOT_STRATEGY_MOTOR_DIRECT:
        motor_command.type = ROBOT_CMD_MOTOR_DIRECT;
        motor_command.value.motors.motor_a = output.motor.motors.motor_a;
        motor_command.value.motors.motor_b = output.motor.motors.motor_b;
        break;
    default:
        ESP_LOGE(TAG, "strategy returned invalid motor mode=%d", output.motor_mode);
        control_manager_report_hardware_fault(&s_control_manager, ROBOT_FAULT_INTERNAL);
        reset_strategy_if_active();
        return;
    }
    if (submit_strategy_command(&motor_command, now_us) != ESP_OK) {
        reset_strategy_if_active();
        return;
    }

    if (output.servo_valid) {
        robot_command_t servo_command = {
            .type = ROBOT_CMD_SERVO,
            .value.servo.angle_deg = output.servo_angle_deg,
        };
        if (submit_strategy_command(&servo_command, now_us) != ESP_OK) {
            reset_strategy_if_active();
        }
    }
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

        /*
         * 可选策略与控制算法在同一个任务中运行，保持执行器单一所有权。
         * 当前工程没有注册策略，因此这里不会改变手动控制行为。
         */
        const int64_t now_us = esp_timer_get_time();
        run_registered_strategy(now_us);

        // 无新命令时也必须 tick，以推进斜坡、换向死区和通信看门狗。
        esp_err_t tick_result = control_manager_tick(&s_control_manager,
                                                     now_us);
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
    if (s_strategy_ops != NULL && s_strategy_ops->init != NULL) {
        ESP_RETURN_ON_ERROR(s_strategy_ops->init(s_strategy_context, &control_config),
                            TAG, "initialize control strategy");
    }
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
