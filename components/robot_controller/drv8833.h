#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

typedef struct {
    gpio_num_t nsleep_gpio;
    gpio_num_t nfault_gpio;

    gpio_num_t ain1_gpio;
    gpio_num_t ain2_gpio;
    gpio_num_t bin1_gpio;
    gpio_num_t bin2_gpio;

    // LEDC channels are bound during init
    int ch_ain1;
    int ch_ain2;
    int ch_bin1;
    int ch_bin2;

    // Runtime state maintained by the driver. The bridge is initialized asleep.
    bool initialized;
    bool awake;
} drv8833_t;

typedef struct {
    int pwm_freq_hz;               // e.g. 20000
    ledc_timer_bit_t pwm_resolution;  // LEDC duty resolution, e.g. LEDC_TIMER_10_BIT
} drv8833_pwm_cfg_t;

esp_err_t drv8833_init(drv8833_t *drv, const drv8833_pwm_cfg_t *pwm_cfg);
// Historical API name: enable=true wakes the bridge; enable=false safely sleeps it.
esp_err_t drv8833_set_sleep(drv8833_t *drv, bool enable);
esp_err_t drv8833_set_motor_a(drv8833_t *drv, float speed); // -1..+1
esp_err_t drv8833_set_motor_b(drv8833_t *drv, float speed); // -1..+1

// Immediately command all four PWM inputs low, then assert nSLEEP low.
// Every output is attempted even if an earlier GPIO/LEDC operation fails.
esp_err_t drv8833_hardware_safe_stop(drv8833_t *drv);

// nFAULT is active-low. Returns ESP_OK and writes the decoded fault state.
esp_err_t drv8833_read_fault(drv8833_t *drv, bool *fault_active);
