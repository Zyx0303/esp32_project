#pragma once

#include "control_strategy.h"
#include "esp_err.h"
#include "robot_config.h"
#include "robot_types.h"

/** Register an optional strategy before app_control_start(); NULL keeps manual control. */
esp_err_t app_control_register_strategy(const robot_control_strategy_ops_t *ops,
                                        void *context);

/** Initialize the state machine, command queue, and 100 Hz control task. */
esp_err_t app_control_start(const robot_config_t *config);

/** Shared non-blocking command ingress for HTTP and UART producers. */
esp_err_t app_control_enqueue(const robot_command_t *command);
