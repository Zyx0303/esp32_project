#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/gpio.h"

typedef struct {
    gpio_num_t gpio;
    int ledc_channel;
} servo_pwm_t;

typedef struct {
    uint32_t min_pulse_us; // e.g. 500
    uint32_t max_pulse_us; // e.g. 2500
    uint32_t freq_hz;      // 50
} servo_pwm_cfg_t;

esp_err_t servo_pwm_init(servo_pwm_t *servo, const servo_pwm_cfg_t *cfg);
esp_err_t servo_pwm_set_angle(servo_pwm_t *servo, const servo_pwm_cfg_t *cfg, float angle_deg); // 0..180

