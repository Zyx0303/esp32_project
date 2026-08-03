#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "robot_state_machine.h"
#include "robot_types.h"

typedef struct {
    esp_err_t (*set_motor_sleep)(void *context, bool sleep);
    esp_err_t (*set_motors)(void *context, int16_t motor_a_percent,
                            int16_t motor_b_percent);
    esp_err_t (*set_servo)(void *context, int16_t angle_deg);
    esp_err_t (*disable_servo)(void *context);
    esp_err_t (*set_power_asserted)(void *context, bool asserted);
} robot_actuator_ops_t;

typedef struct {
    int16_t target;
    int16_t output;
    bool reversing;
    int64_t reverse_release_at_us;
} robot_motor_axis_t;

typedef struct {
    robot_state_machine_t state_machine;
    robot_control_config_t config;
    robot_actuator_ops_t actuator_ops;
    void *actuator_context;
    robot_motor_axis_t motor_a;
    robot_motor_axis_t motor_b;
    robot_status_t status;
    uint32_t last_sequence[ROBOT_COMMAND_SOURCE_COUNT];
    bool sequence_seen[ROBOT_COMMAND_SOURCE_COUNT];
    int64_t last_tick_us;
    uint32_t ramp_remainder;
    uint32_t servo_ramp_remainder;
    int16_t servo_output_deg;
    robot_fault_mask_t active_hardware_faults;
} control_manager_t;

robot_control_config_t control_manager_default_config(void);
esp_err_t control_manager_validate_config(const robot_control_config_t *config);
esp_err_t control_manager_init(control_manager_t *manager,
                               const robot_control_config_t *config,
                               const robot_actuator_ops_t *actuator_ops,
                               void *actuator_context);
esp_err_t control_manager_boot_complete(control_manager_t *manager);
robot_command_result_t control_manager_submit(control_manager_t *manager,
                                              const robot_command_t *command,
                                              int64_t now_us);
esp_err_t control_manager_tick(control_manager_t *manager, int64_t now_us);
void control_manager_report_hardware_fault(control_manager_t *manager,
                                           robot_fault_mask_t faults);
void control_manager_clear_hardware_fault(control_manager_t *manager,
                                          robot_fault_mask_t faults);
void control_manager_get_status(const control_manager_t *manager,
                                robot_status_t *status);
void control_manager_mix_drive(int16_t throttle, int16_t steering,
                               int16_t *motor_a, int16_t *motor_b);
