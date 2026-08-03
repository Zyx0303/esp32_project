#include "control_manager.h"

#include <stddef.h>
#include <string.h>

static int16_t clamp_percent(int32_t value)
{
    if (value > 100) {
        return 100;
    }
    if (value < -100) {
        return -100;
    }
    return (int16_t)value;
}

static int sign_of(int16_t value)
{
    return (value > 0) - (value < 0);
}

static int16_t approach(int16_t current, int16_t target, int32_t maximum_step)
{
    int32_t difference = (int32_t)target - current;
    if (difference > maximum_step) {
        return (int16_t)(current + maximum_step);
    }
    if (difference < -maximum_step) {
        return (int16_t)(current - maximum_step);
    }
    return target;
}

robot_control_config_t control_manager_default_config(void)
{
    return (robot_control_config_t) {
        .command_timeout_ms = 1000,
        .command_max_age_ms = 1000,
        .reverse_deadtime_ms = 30,
        .motor_ramp_percent_per_10ms = 5,
        .invert_motor_a = false,
        .invert_motor_b = false,
        .servo_min_deg = 0,
        .servo_max_deg = 180,
        .servo_initial_deg = 90,
        .servo_slew_deg_per_second = 180,
    };
}

esp_err_t control_manager_validate_config(const robot_control_config_t *config)
{
    if (config == NULL || config->command_timeout_ms == 0 ||
        config->command_max_age_ms == 0 || config->motor_ramp_percent_per_10ms == 0 ||
        config->motor_ramp_percent_per_10ms > 100 || config->servo_min_deg < 0 ||
        config->servo_max_deg > 180 || config->servo_min_deg > config->servo_max_deg ||
        config->servo_initial_deg < config->servo_min_deg ||
        config->servo_initial_deg > config->servo_max_deg ||
        config->servo_slew_deg_per_second == 0 ||
        config->servo_slew_deg_per_second > 1000) {
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
}

static void sync_status(control_manager_t *manager)
{
    manager->status.state = manager->state_machine.state;
    manager->status.armed = robot_state_machine_is_armed(&manager->state_machine);
    manager->status.estop_latched = manager->state_machine.estop_latched;
    manager->status.faults = manager->state_machine.faults;
    manager->status.motor_a_target = manager->motor_a.target;
    manager->status.motor_b_target = manager->motor_b.target;
    manager->status.motor_a_output = manager->motor_a.output;
    manager->status.motor_b_output = manager->motor_b.output;
    manager->status.servo_output_deg = manager->servo_output_deg;
}

static void reset_motor_axis(robot_motor_axis_t *axis)
{
    axis->target = 0;
    axis->output = 0;
    axis->reversing = false;
    axis->reverse_release_at_us = 0;
}

static void safe_stop(control_manager_t *manager)
{
    reset_motor_axis(&manager->motor_a);
    reset_motor_axis(&manager->motor_b);
    manager->status.servo_enabled = false;
    if (manager->actuator_ops.set_motors != NULL) {
        (void)manager->actuator_ops.set_motors(manager->actuator_context, 0, 0);
    }
    if (manager->actuator_ops.set_motor_sleep != NULL) {
        (void)manager->actuator_ops.set_motor_sleep(manager->actuator_context, true);
    }
    if (manager->actuator_ops.disable_servo != NULL) {
        (void)manager->actuator_ops.disable_servo(manager->actuator_context);
    }
    sync_status(manager);
}

static robot_command_result_t fail_hardware(control_manager_t *manager)
{
    robot_state_machine_latch_fault(&manager->state_machine, ROBOT_FAULT_INTERNAL);
    safe_stop(manager);
    return ROBOT_COMMAND_HARDWARE_ERROR;
}

esp_err_t control_manager_init(control_manager_t *manager,
                               const robot_control_config_t *config,
                               const robot_actuator_ops_t *actuator_ops,
                               void *actuator_context)
{
    if (manager == NULL || config == NULL ||
        control_manager_validate_config(config) != ESP_OK) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(manager, 0, sizeof(*manager));
    manager->config = *config;
    if (actuator_ops != NULL) {
        manager->actuator_ops = *actuator_ops;
    }
    manager->actuator_context = actuator_context;
    robot_state_machine_init(&manager->state_machine);
    manager->servo_output_deg = config->servo_initial_deg;
    manager->status.servo_target_deg = config->servo_initial_deg;
    safe_stop(manager);
    return ESP_OK;
}

esp_err_t control_manager_boot_complete(control_manager_t *manager)
{
    if (manager == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t result = robot_state_machine_boot_complete(&manager->state_machine);
    sync_status(manager);
    return result;
}

void control_manager_mix_drive(int16_t throttle, int16_t steering,
                               int16_t *motor_a, int16_t *motor_b)
{
    if (motor_a != NULL) {
        *motor_a = clamp_percent((int32_t)throttle + steering);
    }
    if (motor_b != NULL) {
        *motor_b = clamp_percent((int32_t)throttle - steering);
    }
}

static bool command_has_valid_range(const control_manager_t *manager,
                                    const robot_command_t *command)
{
    switch (command->type) {
    case ROBOT_CMD_DRIVE:
        return command->value.drive.throttle >= -100 && command->value.drive.throttle <= 100 &&
               command->value.drive.steering >= -100 && command->value.drive.steering <= 100;
    case ROBOT_CMD_MOTOR_DIRECT:
        return command->value.motors.motor_a >= -100 && command->value.motors.motor_a <= 100 &&
               command->value.motors.motor_b >= -100 && command->value.motors.motor_b <= 100;
    case ROBOT_CMD_SERVO:
        return command->value.servo.angle_deg >= manager->config.servo_min_deg &&
               command->value.servo.angle_deg <= manager->config.servo_max_deg;
    default:
        return command->type >= ROBOT_CMD_ARM && command->type <= ROBOT_CMD_POWER_CONTROL;
    }
}

static bool is_safety_priority_command(robot_command_type_t type)
{
    return type == ROBOT_CMD_ESTOP || type == ROBOT_CMD_DISARM;
}

static robot_command_result_t reject(control_manager_t *manager,
                                     robot_command_result_t reason)
{
    manager->status.commands_rejected++;
    if (reason == ROBOT_COMMAND_EXPIRED) {
        manager->status.commands_expired++;
    }
    sync_status(manager);
    return reason;
}

static void set_motion_targets(control_manager_t *manager, int16_t motor_a,
                               int16_t motor_b, int64_t now_us)
{
    manager->motor_a.target = manager->config.invert_motor_a ? -motor_a : motor_a;
    manager->motor_b.target = manager->config.invert_motor_b ? -motor_b : motor_b;
    manager->status.last_motion_command_us = now_us;
    if (motor_a != 0 || motor_b != 0) {
        (void)robot_state_machine_motion_started(&manager->state_machine);
    }
}

robot_command_result_t control_manager_submit(control_manager_t *manager,
                                              const robot_command_t *command,
                                              int64_t now_us)
{
    if (manager == NULL || command == NULL || now_us < 0 || command->received_at_us < 0 ||
        command->source < 0 || command->source >= ROBOT_COMMAND_SOURCE_COUNT) {
        return ROBOT_COMMAND_INVALID_ARGUMENT;
    }
    manager->status.commands_received++;

    /* Safety commands are deliberately processed before age and sequence checks. */
    if (command->type == ROBOT_CMD_ESTOP) {
        robot_state_machine_latch_estop(&manager->state_machine);
        safe_stop(manager);
        return ROBOT_COMMAND_ACCEPTED;
    }
    if (command->type == ROBOT_CMD_DISARM) {
        esp_err_t result = robot_state_machine_disarm(&manager->state_machine);
        safe_stop(manager);
        return result == ESP_OK ? ROBOT_COMMAND_ACCEPTED :
                                  reject(manager, ROBOT_COMMAND_INVALID_STATE);
    }
    if (!command_has_valid_range(manager, command)) {
        return reject(manager, ROBOT_COMMAND_INVALID_ARGUMENT);
    }
    if (!is_safety_priority_command(command->type) &&
        (now_us < command->received_at_us ||
         (uint64_t)(now_us - command->received_at_us) >
             (uint64_t)manager->config.command_max_age_ms * 1000ULL)) {
        return reject(manager, ROBOT_COMMAND_EXPIRED);
    }
    if (manager->sequence_seen[command->source] &&
        command->sequence <= manager->last_sequence[command->source]) {
        return reject(manager, ROBOT_COMMAND_OUT_OF_ORDER);
    }
    manager->sequence_seen[command->source] = true;
    manager->last_sequence[command->source] = command->sequence;

    esp_err_t result = ESP_OK;
    switch (command->type) {
    case ROBOT_CMD_ARM:
        result = robot_state_machine_arm(&manager->state_machine);
        if (result != ESP_OK) {
            return reject(manager, ROBOT_COMMAND_INVALID_STATE);
        }
        if (manager->actuator_ops.set_motor_sleep != NULL &&
            manager->actuator_ops.set_motor_sleep(manager->actuator_context, false) != ESP_OK) {
            return fail_hardware(manager);
        }
        break;
    case ROBOT_CMD_CLEAR_FAULT:
        result = robot_state_machine_clear(&manager->state_machine,
                                           manager->active_hardware_faults);
        if (result != ESP_OK) {
            return reject(manager, ROBOT_COMMAND_INVALID_STATE);
        }
        break;
    case ROBOT_CMD_DRIVE: {
        if (!robot_state_machine_allows_motion(&manager->state_machine)) {
            return reject(manager, ROBOT_COMMAND_INVALID_STATE);
        }
        int16_t motor_a = 0;
        int16_t motor_b = 0;
        control_manager_mix_drive(command->value.drive.throttle,
                                  command->value.drive.steering, &motor_a, &motor_b);
        set_motion_targets(manager, motor_a, motor_b, now_us);
        break;
    }
    case ROBOT_CMD_MOTOR_DIRECT:
        if (!robot_state_machine_allows_motion(&manager->state_machine)) {
            return reject(manager, ROBOT_COMMAND_INVALID_STATE);
        }
        set_motion_targets(manager, command->value.motors.motor_a,
                           command->value.motors.motor_b, now_us);
        break;
    case ROBOT_CMD_SERVO:
        if (!robot_state_machine_allows_motion(&manager->state_machine)) {
            return reject(manager, ROBOT_COMMAND_INVALID_STATE);
        }
        manager->status.servo_target_deg = command->value.servo.angle_deg;
        manager->status.servo_enabled = true;
        break;
    case ROBOT_CMD_POWER_CONTROL:
        /* GPIO35 is explicitly controlled and deliberately independent of motor ARM. */
        if (manager->state_machine.state != ROBOT_STATE_SAFE &&
            manager->state_machine.state != ROBOT_STATE_READY &&
            manager->state_machine.state != ROBOT_STATE_RUNNING) {
            return reject(manager, ROBOT_COMMAND_INVALID_STATE);
        }
        if (manager->actuator_ops.set_power_asserted != NULL) {
            result = manager->actuator_ops.set_power_asserted(
                manager->actuator_context, command->value.power.asserted);
        }
        if (result == ESP_OK) {
            manager->status.power_asserted = command->value.power.asserted;
        }
        break;
    default:
        return reject(manager, ROBOT_COMMAND_INVALID_ARGUMENT);
    }

    if (result != ESP_OK) {
        return fail_hardware(manager);
    }
    sync_status(manager);
    return ROBOT_COMMAND_ACCEPTED;
}

static void update_axis(robot_motor_axis_t *axis, int32_t step, int64_t now_us,
                        int64_t reverse_deadtime_us)
{
    if (!axis->reversing && axis->output != 0 && axis->target != 0 &&
        sign_of(axis->output) != sign_of(axis->target)) {
        axis->reversing = true;
        axis->reverse_release_at_us = 0;
    }
    if (axis->target == 0) {
        axis->reversing = false;
        axis->reverse_release_at_us = 0;
    }
    if (axis->reversing) {
        axis->output = approach(axis->output, 0, step);
        if (axis->output == 0) {
            if (axis->reverse_release_at_us == 0) {
                axis->reverse_release_at_us = now_us + reverse_deadtime_us;
            } else if (now_us >= axis->reverse_release_at_us) {
                axis->reversing = false;
                axis->reverse_release_at_us = 0;
            }
        }
        return;
    }
    axis->output = approach(axis->output, axis->target, step);
}

esp_err_t control_manager_tick(control_manager_t *manager, int64_t now_us)
{
    if (manager == NULL || now_us < 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (manager->state_machine.state == ROBOT_STATE_RUNNING &&
        now_us - manager->status.last_motion_command_us >
            (int64_t)manager->config.command_timeout_ms * 1000LL) {
        manager->status.watchdog_stops++;
        robot_state_machine_latch_fault(&manager->state_machine,
                                        ROBOT_FAULT_COMMAND_TIMEOUT);
        robot_state_machine_latch_estop(&manager->state_machine);
        safe_stop(manager);
        return ESP_ERR_TIMEOUT;
    }
    if (!robot_state_machine_allows_motion(&manager->state_machine)) {
        sync_status(manager);
        return ESP_OK;
    }

    int64_t elapsed_us = manager->last_tick_us == 0 ? 10000 : now_us - manager->last_tick_us;
    if (elapsed_us < 0) {
        return ESP_ERR_INVALID_ARG;
    }
    manager->last_tick_us = now_us;
    uint64_t ramp_numerator = (uint64_t)elapsed_us *
                                  manager->config.motor_ramp_percent_per_10ms +
                              manager->ramp_remainder;
    int32_t step = (int32_t)(ramp_numerator / 10000ULL);
    manager->ramp_remainder = (uint32_t)(ramp_numerator % 10000ULL);
    if (step > 100) {
        step = 100;
        manager->ramp_remainder = 0;
    }
    int64_t deadtime_us = (int64_t)manager->config.reverse_deadtime_ms * 1000LL;
    update_axis(&manager->motor_a, step, now_us, deadtime_us);
    update_axis(&manager->motor_b, step, now_us, deadtime_us);

    if (manager->actuator_ops.set_motors != NULL &&
        manager->actuator_ops.set_motors(manager->actuator_context,
                                         manager->motor_a.output,
                                         manager->motor_b.output) != ESP_OK) {
        (void)fail_hardware(manager);
        return ESP_FAIL;
    }
    if (manager->status.servo_enabled) {
        uint64_t servo_numerator = (uint64_t)elapsed_us *
                                       manager->config.servo_slew_deg_per_second +
                                   manager->servo_ramp_remainder;
        int32_t servo_step = (int32_t)(servo_numerator / 1000000ULL);
        manager->servo_ramp_remainder = (uint32_t)(servo_numerator % 1000000ULL);
        if (servo_step > 180) {
            servo_step = 180;
            manager->servo_ramp_remainder = 0;
        }
        if (servo_step > 0) {
            manager->servo_output_deg = approach(manager->servo_output_deg,
                                                  manager->status.servo_target_deg,
                                                  servo_step);
        }
        if (manager->actuator_ops.set_servo != NULL &&
            manager->actuator_ops.set_servo(manager->actuator_context,
                                            manager->servo_output_deg) != ESP_OK) {
            (void)fail_hardware(manager);
            return ESP_FAIL;
        }
    }
    if (manager->motor_a.target == 0 && manager->motor_b.target == 0 &&
        manager->motor_a.output == 0 && manager->motor_b.output == 0) {
        (void)robot_state_machine_motion_stopped(&manager->state_machine);
    }
    sync_status(manager);
    return ESP_OK;
}

void control_manager_report_hardware_fault(control_manager_t *manager,
                                           robot_fault_mask_t faults)
{
    if (manager == NULL || faults == ROBOT_FAULT_NONE) {
        return;
    }
    manager->active_hardware_faults |= faults;
    robot_state_machine_latch_fault(&manager->state_machine, faults);
    safe_stop(manager);
}

void control_manager_clear_hardware_fault(control_manager_t *manager,
                                          robot_fault_mask_t faults)
{
    if (manager == NULL) {
        return;
    }
    manager->active_hardware_faults &= ~faults;
}

void control_manager_get_status(const control_manager_t *manager,
                                robot_status_t *status)
{
    if (manager != NULL && status != NULL) {
        *status = manager->status;
    }
}
