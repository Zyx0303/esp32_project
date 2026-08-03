#include "app_actuators.h"

#include "board_pins.h"
#include "drv8833.h"
#include "esp_check.h"
#include "esp_log.h"
#include "power_control.h"
#include "servo_pwm.h"

static const char *TAG = "app_actuators";

typedef struct {
    drv8833_t motor;
    servo_pwm_t servo;
    power_control_t power;
    bool motor_available;
    bool servo_available;
    bool power_available;
} actuator_context_t;

static actuator_context_t s_actuators = {
    .motor = {
        .nsleep_gpio = PIN_DRV_NSLEEP,
        .nfault_gpio = PIN_DRV_NFAULT,
        .ain1_gpio = PIN_DRV_AIN1,
        .ain2_gpio = PIN_DRV_AIN2,
        .bin1_gpio = PIN_DRV_BIN1,
        .bin2_gpio = PIN_DRV_BIN2,
    },
    .servo = {.gpio = PIN_SERVO_PWM},
};

static const drv8833_pwm_cfg_t s_motor_pwm_config = {
    .pwm_freq_hz = 16000,
    .pwm_resolution = LEDC_TIMER_10_BIT,
};

static servo_pwm_cfg_t s_servo_config = {
    .min_pulse_us = 500,
    .max_pulse_us = 2500,
    .freq_hz = 50,
};

static esp_err_t set_motor_sleep(void *context, bool sleep)
{
    actuator_context_t *actuators = context;
    if (actuators == NULL || !actuators->motor_available) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    // 唤醒 H 桥不会产生运动；非零 PWM 只能由后续 set_motors 写入。
    return sleep ? drv8833_hardware_safe_stop(&actuators->motor)
                 : drv8833_set_sleep(&actuators->motor, true);
}

static esp_err_t set_motors(void *context, int16_t motor_a, int16_t motor_b)
{
    actuator_context_t *actuators = context;
    if (actuators == NULL || !actuators->motor_available) {
        return (motor_a == 0 && motor_b == 0) ? ESP_OK : ESP_ERR_NOT_SUPPORTED;
    }
    ESP_RETURN_ON_ERROR(drv8833_set_motor_a(&actuators->motor, motor_a / 100.0f),
                        TAG, "motor A");
    esp_err_t err = drv8833_set_motor_b(&actuators->motor, motor_b / 100.0f);
    if (err != ESP_OK) {
        // 禁止在 Motor B 写失败后留下 Motor A 单侧运行。
        (void)drv8833_hardware_safe_stop(&actuators->motor);
    }
    return err;
}

static esp_err_t set_servo(void *context, int16_t angle_deg)
{
    actuator_context_t *actuators = context;
    if (actuators == NULL || !actuators->servo_available) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    // 初始化不输出脉冲，第一次合法舵机命令才显式启用 PWM。
    if (!actuators->servo.enabled) {
        ESP_RETURN_ON_ERROR(servo_pwm_enable(&actuators->servo), TAG, "enable servo");
    }
    return servo_pwm_set_angle(&actuators->servo, &s_servo_config, angle_deg);
}

static esp_err_t disable_servo(void *context)
{
    actuator_context_t *actuators = context;
    if (actuators == NULL || !actuators->servo_available) {
        return ESP_OK;
    }
    return servo_pwm_disable(&actuators->servo);
}

static esp_err_t set_power(void *context, bool asserted)
{
    actuator_context_t *actuators = context;
    if (actuators == NULL || !actuators->power_available) {
        return asserted ? ESP_ERR_NOT_SUPPORTED : ESP_OK;
    }
    return power_control_set_asserted(&actuators->power, asserted);
}

static const robot_actuator_ops_t s_actuator_ops = {
    .set_motor_sleep = set_motor_sleep,
    .set_motors = set_motors,
    .set_servo = set_servo,
    .disable_servo = disable_servo,
    .set_power_asserted = set_power,
};

esp_err_t app_actuators_init_safe_power(void)
{
    ESP_RETURN_ON_ERROR(power_control_init(&s_actuators.power, PIN_POWER_CTRL),
                        TAG, "initialize power control");
    ESP_RETURN_ON_ERROR(power_control_set_asserted(&s_actuators.power, false),
                        TAG, "release external interface");
    s_actuators.power_available = true;
    return ESP_OK;
}

void app_actuators_init_motion(const robot_config_t *config)
{
    if (config == NULL) {
        ESP_LOGE(TAG, "validated configuration required");
        return;
    }
    s_servo_config.min_pulse_us = config->servo_min_pulse_us;
    s_servo_config.max_pulse_us = config->servo_max_pulse_us;

    esp_err_t motor_err = drv8833_init(&s_actuators.motor, &s_motor_pwm_config);
    s_actuators.motor_available = motor_err == ESP_OK;
    if (motor_err != ESP_OK) {
        ESP_LOGE(TAG, "DRV8833 init failed: %s", esp_err_to_name(motor_err));
    }

    esp_err_t servo_err = servo_pwm_init(&s_actuators.servo, &s_servo_config);
    s_actuators.servo_available = servo_err == ESP_OK;
    if (servo_err != ESP_OK) {
        ESP_LOGE(TAG, "servo init failed: %s", esp_err_to_name(servo_err));
    }
}

const robot_actuator_ops_t *app_actuators_get_ops(void)
{
    return &s_actuator_ops;
}

void *app_actuators_get_context(void)
{
    return &s_actuators;
}

bool app_actuators_motor_available(void)
{
    return s_actuators.motor_available;
}

esp_err_t app_actuators_read_motor_fault(bool *fault_active)
{
    if (!s_actuators.motor_available) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    return drv8833_read_fault(&s_actuators.motor, fault_active);
}
