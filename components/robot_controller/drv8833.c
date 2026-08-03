#include "drv8833.h"

#include <inttypes.h>
#include <math.h>
#include "esp_log.h"
#include "esp_check.h"
#include "driver/ledc.h"

static const char *TAG = "drv8833";

static uint32_t s_max_duty = 0;

static void remember_first_error(esp_err_t candidate, esp_err_t *first_error)
{
    if (*first_error == ESP_OK && candidate != ESP_OK) {
        *first_error = candidate;
    }
}

static uint32_t clamp_u32(uint32_t v, uint32_t lo, uint32_t hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static uint32_t speed_to_duty(float speed)
{
    float s = fabsf(speed);
    if (s > 1.0f) s = 1.0f;
    uint32_t d = (uint32_t)lroundf(s * (float)s_max_duty);
    return clamp_u32(d, 0, s_max_duty);
}

static esp_err_t set_duty(int ch, uint32_t duty)
{
    ESP_RETURN_ON_ERROR(ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, duty), TAG, "set duty");
    ESP_RETURN_ON_ERROR(ledc_update_duty(LEDC_LOW_SPEED_MODE, ch), TAG, "upd duty");
    return ESP_OK;
}

esp_err_t drv8833_init(drv8833_t *drv, const drv8833_pwm_cfg_t *pwm_cfg)
{
    if (!drv || !pwm_cfg) return ESP_ERR_INVALID_ARG;

    drv->initialized = false;
    drv->awake = false;

    // Disable the bridge before touching PWM inputs.
    gpio_config_t out_cfg = {
        .pin_bit_mask = (1ULL << drv->nsleep_gpio),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&out_cfg), TAG, "nsleep cfg");
    ESP_RETURN_ON_ERROR(gpio_set_level(drv->nsleep_gpio, 0), TAG, "nsleep low");

    if (!GPIO_IS_VALID_OUTPUT_GPIO(drv->ain1_gpio) ||
        !GPIO_IS_VALID_OUTPUT_GPIO(drv->ain2_gpio) ||
        !GPIO_IS_VALID_OUTPUT_GPIO(drv->bin1_gpio) ||
        !GPIO_IS_VALID_OUTPUT_GPIO(drv->bin2_gpio)) {
        ESP_LOGE(TAG, "invalid motor output GPIO; bridge remains asleep");
        return ESP_ERR_NOT_SUPPORTED;
    }

    // nFAULT is an open-drain input on DRV8833.
    gpio_config_t in_cfg = {
        .pin_bit_mask = (1ULL << drv->nfault_gpio),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&in_cfg), TAG, "nfault cfg");

    // High-speed timer for motors
    const ledc_timer_config_t tim = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = pwm_cfg->pwm_resolution,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = pwm_cfg->pwm_freq_hz,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&tim), TAG, "timer cfg");

    s_max_duty = (1u << pwm_cfg->pwm_resolution) - 1u;

    // Assign fixed channels 0..3 for simplicity
    drv->ch_ain1 = LEDC_CHANNEL_0;
    drv->ch_ain2 = LEDC_CHANNEL_1;
    drv->ch_bin1 = LEDC_CHANNEL_2;
    drv->ch_bin2 = LEDC_CHANNEL_3;

    const ledc_channel_config_t ch_ain1 = {
        .gpio_num = drv->ain1_gpio,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = drv->ch_ain1,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
        .flags.output_invert = 0,
    };
    const ledc_channel_config_t ch_ain2 = { .gpio_num = drv->ain2_gpio, .speed_mode = LEDC_LOW_SPEED_MODE, .channel = drv->ch_ain2, .intr_type = LEDC_INTR_DISABLE, .timer_sel = LEDC_TIMER_0, .duty = 0, .hpoint = 0, .flags.output_invert = 0 };
    const ledc_channel_config_t ch_bin1 = { .gpio_num = drv->bin1_gpio, .speed_mode = LEDC_LOW_SPEED_MODE, .channel = drv->ch_bin1, .intr_type = LEDC_INTR_DISABLE, .timer_sel = LEDC_TIMER_0, .duty = 0, .hpoint = 0, .flags.output_invert = 0 };
    const ledc_channel_config_t ch_bin2 = { .gpio_num = drv->bin2_gpio, .speed_mode = LEDC_LOW_SPEED_MODE, .channel = drv->ch_bin2, .intr_type = LEDC_INTR_DISABLE, .timer_sel = LEDC_TIMER_0, .duty = 0, .hpoint = 0, .flags.output_invert = 0 };

    ESP_RETURN_ON_ERROR(ledc_channel_config(&ch_ain1), TAG, "ch ain1");
    ESP_RETURN_ON_ERROR(ledc_channel_config(&ch_ain2), TAG, "ch ain2");
    ESP_RETURN_ON_ERROR(ledc_channel_config(&ch_bin1), TAG, "ch bin1");
    ESP_RETURN_ON_ERROR(ledc_channel_config(&ch_bin2), TAG, "ch bin2");

    // All PWM channels were configured with zero duty and nSLEEP remains low.
    drv->initialized = true;
    ESP_LOGI(TAG, "PWM freq=%dHz res=%dbit max_duty=%" PRIu32 "; bridge asleep",
             pwm_cfg->pwm_freq_hz, pwm_cfg->pwm_resolution, s_max_duty);
    return ESP_OK;
}

esp_err_t drv8833_set_sleep(drv8833_t *drv, bool enable)
{
    if (!drv) return ESP_ERR_INVALID_ARG;
    if (!drv->initialized) return ESP_ERR_INVALID_STATE;

    // Sleeping is always a full hardware-safe stop, never just a pin toggle.
    if (!enable) {
        return drv8833_hardware_safe_stop(drv);
    }

    ESP_RETURN_ON_ERROR(gpio_set_level(drv->nsleep_gpio, 1), TAG, "wake bridge");
    drv->awake = true;
    return ESP_OK;
}

static esp_err_t set_hbridge(int ch1, int ch2, float speed)
{
    if (!isfinite(speed)) return ESP_ERR_INVALID_ARG;

    if (speed > 0.001f) {
        uint32_t d = speed_to_duty(speed);
        ESP_RETURN_ON_ERROR(set_duty(ch1, d), TAG, "duty fwd 1");
        ESP_RETURN_ON_ERROR(set_duty(ch2, 0), TAG, "duty fwd 2");
    } else if (speed < -0.001f) {
        uint32_t d = speed_to_duty(speed);
        ESP_RETURN_ON_ERROR(set_duty(ch1, 0), TAG, "duty rev 1");
        ESP_RETURN_ON_ERROR(set_duty(ch2, d), TAG, "duty rev 2");
    } else {
        ESP_RETURN_ON_ERROR(set_duty(ch1, 0), TAG, "duty stop 1");
        ESP_RETURN_ON_ERROR(set_duty(ch2, 0), TAG, "duty stop 2");
    }
    return ESP_OK;
}

esp_err_t drv8833_set_motor_a(drv8833_t *drv, float speed)
{
    if (!drv) return ESP_ERR_INVALID_ARG;
    if (!drv->initialized) return ESP_ERR_INVALID_STATE;
    if (!drv->awake && fabsf(speed) > 0.001f) return ESP_ERR_INVALID_STATE;
    return set_hbridge(drv->ch_ain1, drv->ch_ain2, speed);
}

esp_err_t drv8833_set_motor_b(drv8833_t *drv, float speed)
{
    if (!drv) return ESP_ERR_INVALID_ARG;
    if (!drv->initialized) return ESP_ERR_INVALID_STATE;
    if (!drv->awake && fabsf(speed) > 0.001f) return ESP_ERR_INVALID_STATE;
    return set_hbridge(drv->ch_bin1, drv->ch_bin2, speed);
}

esp_err_t drv8833_hardware_safe_stop(drv8833_t *drv)
{
    if (!drv) return ESP_ERR_INVALID_ARG;
    if (!drv->initialized) return ESP_ERR_INVALID_STATE;

    esp_err_t first_error = ESP_OK;
    remember_first_error(set_duty(drv->ch_ain1, 0), &first_error);
    remember_first_error(set_duty(drv->ch_ain2, 0), &first_error);
    remember_first_error(set_duty(drv->ch_bin1, 0), &first_error);
    remember_first_error(set_duty(drv->ch_bin2, 0), &first_error);
    remember_first_error(gpio_set_level(drv->nsleep_gpio, 0), &first_error);

    // Record the conservative software state even if a hardware call failed.
    drv->awake = false;
    return first_error;
}

esp_err_t drv8833_read_fault(drv8833_t *drv, bool *fault_active)
{
    if (!drv || !fault_active) return ESP_ERR_INVALID_ARG;
    if (!drv->initialized) return ESP_ERR_INVALID_STATE;

    const int level = gpio_get_level(drv->nfault_gpio);
    *fault_active = (level == 0);
    return ESP_OK;
}
