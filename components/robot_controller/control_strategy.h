#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "robot_types.h"

/*
 * 可插拔控制策略接口
 *
 * 本接口只描述“根据状态计算下一周期目标”，不允许策略直接访问 PWM、GPIO 或 I2C。
 * 策略输出由 control_task 转换为 robot_command_t，再经过 control_manager 的状态机、
 * 范围检查、斜坡、换向死区和通信看门狗后才能到达执行器。
 *
 * 后续可在不改变安全层的情况下实现 PID、姿态闭环、轨迹跟踪、CPG 等策略。
 */

typedef enum {
    /* 默认值。成功执行但不要求运动时，control_task 会提交双电机零目标。 */
    ROBOT_STRATEGY_MOTOR_STOP = 0,
    /* 差速控制：motor_a = throttle + steering，motor_b = throttle - steering。 */
    ROBOT_STRATEGY_MOTOR_DRIVE,
    /* 直接给出左右电机百分比目标。 */
    ROBOT_STRATEGY_MOTOR_DIRECT,
} robot_strategy_motor_mode_t;

typedef struct {
    /* 单调微秒时间及本次与上次调用的间隔，适合离散控制器计算。 */
    int64_t now_us;
    int64_t delta_us;

    /*
     * 最近一次发布的完整状态快照，包含电机、舵机和 MPU6050 原始数据。
     * 快照最多落后一个 10 ms 控制周期；策略输出提交时仍会依据当前状态重新校验。
     */
    robot_status_t status;
} robot_strategy_input_t;

typedef struct {
    robot_strategy_motor_mode_t motor_mode;
    union {
        struct {
            int16_t throttle;
            int16_t steering;
        } drive;
        struct {
            int16_t motor_a;
            int16_t motor_b;
        } motors;
    } motor;

    /* false 表示本周期不修改舵机目标；true 时 angle_deg 仍需通过安全范围检查。 */
    bool servo_valid;
    int16_t servo_angle_deg;
} robot_strategy_output_t;

typedef struct {
    const char *name;

    /* 可选。任务创建前调用，用于检查参数和初始化策略上下文。 */
    esp_err_t (*init)(void *context, const robot_control_config_t *control_config);

    /* 可选。每次进入或离开 ARM 状态时调用，用于清除积分量和历史状态。 */
    void (*reset)(void *context);

    /*
     * 必选。由 control_task 每 10 ms 调用一次。
     * output 在调用前已清零，因此不填写电机模式时默认为安全停止。
     * 回调必须快速返回，不能阻塞、延时或直接操作执行器。
     */
    esp_err_t (*step)(void *context, const robot_strategy_input_t *input,
                      robot_strategy_output_t *output);
} robot_control_strategy_ops_t;
