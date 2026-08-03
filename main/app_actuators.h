#pragma once

#include <stdbool.h>

#include "control_manager.h"
#include "esp_err.h"
#include "robot_config.h"

/** Establish GPIO35's released state before nonessential services start. */
esp_err_t app_actuators_init_safe_power(void);

/** Initialize motor and servo drivers in their disabled/sleeping states. */
void app_actuators_init_motion(const robot_config_t *config);

/** Dependency-injection table consumed by control_manager. */
const robot_actuator_ops_t *app_actuators_get_ops(void);
void *app_actuators_get_context(void);

bool app_actuators_motor_available(void);
esp_err_t app_actuators_read_motor_fault(bool *fault_active);
