#include "servo_pwm.h"

#include <inttypes.h>
#include <math.h>
#include "esp_log.h"
#include "esp_check.h"
#include "driver/ledc.h"

static const char *TAG = "servo_pwm";

static uint32_t calc_max_duty(ledc_timer_bit_t res)
{
    return (1u << res) - 1u;
}

static uint32_t us_to_duty(uint32_t pulse_us, uint32_t freq_hz, uint32_t max_duty)
{
    // duty = (pulse_us / period_us) * max_duty
    const float period_us = 1000000.0f / (float)freq_hz;
    float duty_f = ((float)pulse_us / period_us) * (float)max_duty;
    if (duty_f < 0) duty_f = 0;
    if (duty_f > (float)max_duty) duty_f = (float)max_duty;
    return (uint32_t)lroundf(duty_f);
}

esp_err_t servo_pwm_init(servo_pwm_t *servo, const servo_pwm_cfg_t *cfg)
{
    if (!servo || !cfg) return ESP_ERR_INVALID_ARG;

    // Low-speed timer for servo (50Hz), high resolution for pulse width accuracy
    const ledc_timer_config_t tim = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_14_BIT,
        .timer_num = LEDC_TIMER_1,
        .freq_hz = cfg->freq_hz,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&tim), TAG, "timer cfg");

    servo->ledc_channel = LEDC_CHANNEL_4;
    const ledc_channel_config_t ch = {
        .gpio_num = servo->gpio,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = servo->ledc_channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_1,
        .duty = 0,
        .hpoint = 0,
        .flags.output_invert = 0,
    };
    ESP_RETURN_ON_ERROR(ledc_channel_config(&ch), TAG, "channel cfg");

    ESP_LOGI(TAG, "servo pwm gpio=%d freq=%" PRIu32 "Hz", (int)servo->gpio, cfg->freq_hz);
    return ESP_OK;
}

esp_err_t servo_pwm_set_angle(servo_pwm_t *servo, const servo_pwm_cfg_t *cfg, float angle_deg)
{
    if (!servo || !cfg) return ESP_ERR_INVALID_ARG;
    if (angle_deg < 0) angle_deg = 0;
    if (angle_deg > 180) angle_deg = 180;

    float t = angle_deg / 180.0f;
    uint32_t pulse = (uint32_t)lroundf((1.0f - t) * (float)cfg->min_pulse_us + t * (float)cfg->max_pulse_us);

    uint32_t max_duty = calc_max_duty(LEDC_TIMER_14_BIT);
    uint32_t duty = us_to_duty(pulse, cfg->freq_hz, max_duty);

    ESP_RETURN_ON_ERROR(ledc_set_duty(LEDC_LOW_SPEED_MODE, servo->ledc_channel, duty), TAG, "set duty");
    ESP_RETURN_ON_ERROR(ledc_update_duty(LEDC_LOW_SPEED_MODE, servo->ledc_channel), TAG, "update duty");
    return ESP_OK;
}

