#include "robot_state_machine.h"

#include <stddef.h>

void robot_state_machine_init(robot_state_machine_t *machine)
{
    if (machine == NULL) {
        return;
    }
    machine->state = ROBOT_STATE_BOOT;
    machine->faults = ROBOT_FAULT_NONE;
    machine->estop_latched = false;
}

esp_err_t robot_state_machine_boot_complete(robot_state_machine_t *machine)
{
    if (machine == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (machine->state != ROBOT_STATE_BOOT || machine->faults != ROBOT_FAULT_NONE) {
        return ESP_ERR_INVALID_STATE;
    }
    machine->state = ROBOT_STATE_SAFE;
    return ESP_OK;
}

esp_err_t robot_state_machine_arm(robot_state_machine_t *machine)
{
    if (machine == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (machine->state != ROBOT_STATE_SAFE || machine->faults != ROBOT_FAULT_NONE ||
        machine->estop_latched) {
        return ESP_ERR_INVALID_STATE;
    }
    machine->state = ROBOT_STATE_READY;
    return ESP_OK;
}

esp_err_t robot_state_machine_disarm(robot_state_machine_t *machine)
{
    if (machine == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (machine->state == ROBOT_STATE_BOOT || machine->state == ROBOT_STATE_FAULT ||
        machine->state == ROBOT_STATE_ESTOP) {
        return ESP_ERR_INVALID_STATE;
    }
    machine->state = ROBOT_STATE_SAFE;
    return ESP_OK;
}

esp_err_t robot_state_machine_motion_started(robot_state_machine_t *machine)
{
    if (machine == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (machine->state != ROBOT_STATE_READY && machine->state != ROBOT_STATE_RUNNING) {
        return ESP_ERR_INVALID_STATE;
    }
    machine->state = ROBOT_STATE_RUNNING;
    return ESP_OK;
}

esp_err_t robot_state_machine_motion_stopped(robot_state_machine_t *machine)
{
    if (machine == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (machine->state == ROBOT_STATE_RUNNING) {
        machine->state = ROBOT_STATE_READY;
        return ESP_OK;
    }
    return machine->state == ROBOT_STATE_READY ? ESP_OK : ESP_ERR_INVALID_STATE;
}

void robot_state_machine_latch_fault(robot_state_machine_t *machine,
                                     robot_fault_mask_t faults)
{
    if (machine == NULL || faults == ROBOT_FAULT_NONE) {
        return;
    }
    machine->faults |= faults;
    machine->state = ROBOT_STATE_FAULT;
}

void robot_state_machine_latch_estop(robot_state_machine_t *machine)
{
    if (machine == NULL) {
        return;
    }
    machine->estop_latched = true;
    machine->state = ROBOT_STATE_ESTOP;
}

esp_err_t robot_state_machine_clear(robot_state_machine_t *machine,
                                    robot_fault_mask_t active_hardware_faults)
{
    if (machine == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (machine->state != ROBOT_STATE_FAULT && machine->state != ROBOT_STATE_ESTOP) {
        return ESP_ERR_INVALID_STATE;
    }
    if (active_hardware_faults != ROBOT_FAULT_NONE) {
        return ESP_ERR_INVALID_STATE;
    }
    machine->faults = ROBOT_FAULT_NONE;
    machine->estop_latched = false;
    machine->state = ROBOT_STATE_SAFE;
    return ESP_OK;
}

bool robot_state_machine_is_armed(const robot_state_machine_t *machine)
{
    return machine != NULL &&
           (machine->state == ROBOT_STATE_READY || machine->state == ROBOT_STATE_RUNNING);
}

bool robot_state_machine_allows_motion(const robot_state_machine_t *machine)
{
    return robot_state_machine_is_armed(machine) && machine->faults == ROBOT_FAULT_NONE &&
           !machine->estop_latched;
}

const char *robot_state_name(robot_state_t state)
{
    switch (state) {
    case ROBOT_STATE_BOOT: return "BOOT";
    case ROBOT_STATE_SAFE: return "SAFE";
    case ROBOT_STATE_READY: return "READY";
    case ROBOT_STATE_RUNNING: return "RUNNING";
    case ROBOT_STATE_FAULT: return "FAULT";
    case ROBOT_STATE_ESTOP: return "ESTOP";
    default: return "UNKNOWN";
    }
}
