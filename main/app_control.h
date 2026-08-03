#pragma once

#include "esp_err.h"
#include "robot_config.h"
#include "robot_types.h"

/** Initialize the state machine, command queue, and 100 Hz control task. */
esp_err_t app_control_start(const robot_config_t *config);

/** Shared non-blocking command ingress for HTTP and UART producers. */
esp_err_t app_control_enqueue(const robot_command_t *command);
