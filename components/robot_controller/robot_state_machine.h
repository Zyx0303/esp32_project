#pragma once

#include <stdbool.h>

#include "esp_err.h"
#include "robot_types.h"

typedef struct {
    robot_state_t state;
    robot_fault_mask_t faults;
    bool estop_latched;
} robot_state_machine_t;

void robot_state_machine_init(robot_state_machine_t *machine);
esp_err_t robot_state_machine_boot_complete(robot_state_machine_t *machine);
esp_err_t robot_state_machine_arm(robot_state_machine_t *machine);
esp_err_t robot_state_machine_disarm(robot_state_machine_t *machine);
esp_err_t robot_state_machine_motion_started(robot_state_machine_t *machine);
esp_err_t robot_state_machine_motion_stopped(robot_state_machine_t *machine);
void robot_state_machine_latch_fault(robot_state_machine_t *machine,
                                     robot_fault_mask_t faults);
void robot_state_machine_latch_estop(robot_state_machine_t *machine);
esp_err_t robot_state_machine_clear(robot_state_machine_t *machine,
                                    robot_fault_mask_t active_hardware_faults);
bool robot_state_machine_is_armed(const robot_state_machine_t *machine);
bool robot_state_machine_allows_motion(const robot_state_machine_t *machine);
const char *robot_state_name(robot_state_t state);
