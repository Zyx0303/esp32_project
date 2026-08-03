#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#define ROBOT_CONFIG_VERSION 1U

typedef struct {
    uint32_t version;
    bool invert_motor_a;
    bool invert_motor_b;
    uint16_t motor_ramp_step_percent;
    uint16_t motor_reverse_deadtime_ms;
    uint16_t command_timeout_ms;
    uint16_t servo_min_angle_deg;
    uint16_t servo_max_angle_deg;
    uint16_t servo_center_angle_deg;
    uint16_t servo_min_pulse_us;
    uint16_t servo_max_pulse_us;
    uint16_t servo_slew_deg_per_second;
} robot_config_t;

void robot_config_defaults(robot_config_t *config);
bool robot_config_validate(const robot_config_t *config);
esp_err_t robot_config_load(robot_config_t *config, bool *used_defaults);
esp_err_t robot_config_save(const robot_config_t *config);
