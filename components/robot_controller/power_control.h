#pragma once

#include <stdbool.h>

#include "driver/gpio.h"
#include "esp_err.h"

typedef struct {
    gpio_num_t gpio;
    bool initialized;
} power_control_t;

// The schematic uses an N-channel MOSFET as an open-drain output:
// asserted=true drives the gate high and pulls the external interface low.
esp_err_t power_control_init(power_control_t *control, gpio_num_t gpio);
esp_err_t power_control_set_asserted(power_control_t *control, bool asserted);
