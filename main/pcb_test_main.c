/*
 * ESP32-S3 minimum-system-board signal generator for robot PCB bring-up.
 *
 * Safety properties:
 *   - every actuator output is inactive before the command interface starts;
 *   - output commands require an explicit "arm";
 *   - active outputs are stopped after five seconds without a new command;
 *   - an active-low DRV8833 nFAULT immediately stops and disarms everything.
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "pcb_test_config.h"

#define TEST_UART             UART_NUM_0
#define TEST_UART_BUFFER_SIZE 256
#define MOTOR_TIMER           LEDC_TIMER_0
#define MOTOR_RESOLUTION      LEDC_TIMER_10_BIT
#define MOTOR_MAX_DUTY        ((1U << MOTOR_RESOLUTION) - 1U)
#define SERVO_TIMER           LEDC_TIMER_1
#define SERVO_RESOLUTION      LEDC_TIMER_14_BIT
#define SERVO_MAX_DUTY        ((1U << SERVO_RESOLUTION) - 1U)

typedef struct {
    bool armed;
    bool fault_latched;
    bool servo_enabled;
    bool magnet_on;
    int motor_a_percent;
    int motor_b_percent;
    int servo_angle_deg;
    uint32_t ain1_duty;
    uint32_t ain2_duty;
    uint32_t bin1_duty;
    uint32_t bin2_duty;
    uint32_t servo_duty;
    uint32_t servo_pulse_us;
    int64_t output_deadline_us;
} pcb_test_state_t;

static const char *TAG = "pcb_test";
static pcb_test_state_t s_state;

static esp_err_t set_pwm(ledc_channel_t channel, uint32_t duty)
{
    esp_err_t err = ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty);
    if (err != ESP_OK) {
        return err;
    }
    return ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
}

static void remember_first_error(esp_err_t candidate, esp_err_t *first_error)
{
    if (*first_error == ESP_OK && candidate != ESP_OK) {
        *first_error = candidate;
    }
}

static bool outputs_active(void)
{
    return s_state.motor_a_percent != 0 || s_state.motor_b_percent != 0 ||
           s_state.servo_enabled || s_state.magnet_on;
}

static void refresh_output_watchdog(void)
{
    s_state.output_deadline_us = esp_timer_get_time() +
        (int64_t)PCB_TEST_OUTPUT_TIMEOUT_MS * 1000;
}

/* Attempt every shutdown action even if one peripheral call fails. */
static esp_err_t stop_all_outputs(bool disarm)
{
    esp_err_t first_error = ESP_OK;

    remember_first_error(set_pwm(LEDC_CHANNEL_0, 0), &first_error);
    remember_first_error(set_pwm(LEDC_CHANNEL_1, 0), &first_error);
    remember_first_error(set_pwm(LEDC_CHANNEL_2, 0), &first_error);
    remember_first_error(set_pwm(LEDC_CHANNEL_3, 0), &first_error);
    remember_first_error(gpio_set_level(PCB_TEST_PIN_DRV_NSLEEP, 0), &first_error);
    remember_first_error(set_pwm(LEDC_CHANNEL_4, 0), &first_error);
    remember_first_error(gpio_set_level(PCB_TEST_PIN_MAGNET, 0), &first_error);

    s_state.motor_a_percent = 0;
    s_state.motor_b_percent = 0;
    s_state.ain1_duty = 0;
    s_state.ain2_duty = 0;
    s_state.bin1_duty = 0;
    s_state.bin2_duty = 0;
    s_state.servo_enabled = false;
    s_state.servo_duty = 0;
    s_state.servo_pulse_us = 0;
    s_state.magnet_on = false;
    s_state.output_deadline_us = 0;
    if (disarm) {
        s_state.armed = false;
    }
    return first_error;
}

static esp_err_t configure_output_gpio_safe(gpio_num_t gpio)
{
    esp_err_t err = gpio_set_level(gpio, 0);
    if (err != ESP_OK) {
        return err;
    }
    const gpio_config_t config = {
        .pin_bit_mask = 1ULL << gpio,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    return gpio_config(&config);
}

static esp_err_t configure_ledc_channel(gpio_num_t gpio, ledc_channel_t channel,
                                        ledc_timer_t timer)
{
    if (!GPIO_IS_VALID_OUTPUT_GPIO(gpio)) {
        ESP_LOGE(TAG, "GPIO%d is not output-capable for this ESP-IDF target", (int)gpio);
        return ESP_ERR_NOT_SUPPORTED;
    }
    const ledc_channel_config_t config = {
        .gpio_num = gpio,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = timer,
        .duty = 0,
        .hpoint = 0,
        .flags.output_invert = 0,
    };
    return ledc_channel_config(&config);
}

static esp_err_t hardware_init(void)
{
    /* Disable the external loads before PWM pin muxing or serial setup. */
    esp_err_t err = configure_output_gpio_safe(PCB_TEST_PIN_MAGNET);
    if (err != ESP_OK) {
        return err;
    }
    err = configure_output_gpio_safe(PCB_TEST_PIN_DRV_NSLEEP);
    if (err != ESP_OK) {
        return err;
    }

    const gpio_config_t fault_config = {
        .pin_bit_mask = 1ULL << PCB_TEST_PIN_DRV_NFAULT,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    err = gpio_config(&fault_config);
    if (err != ESP_OK) {
        return err;
    }

    const ledc_timer_config_t motor_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = MOTOR_RESOLUTION,
        .timer_num = MOTOR_TIMER,
        .freq_hz = PCB_TEST_MOTOR_PWM_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    err = ledc_timer_config(&motor_timer);
    if (err != ESP_OK) {
        return err;
    }

    const ledc_timer_config_t servo_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = SERVO_RESOLUTION,
        .timer_num = SERVO_TIMER,
        .freq_hz = PCB_TEST_SERVO_PWM_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    err = ledc_timer_config(&servo_timer);
    if (err != ESP_OK) {
        return err;
    }

    const gpio_num_t motor_pins[] = {
        PCB_TEST_PIN_DRV_AIN1, PCB_TEST_PIN_DRV_AIN2,
        PCB_TEST_PIN_DRV_BIN1, PCB_TEST_PIN_DRV_BIN2,
    };
    for (int i = 0; i < 4; ++i) {
        err = configure_ledc_channel(motor_pins[i], (ledc_channel_t)i, MOTOR_TIMER);
        if (err != ESP_OK) {
            return err;
        }
    }
    err = configure_ledc_channel(PCB_TEST_PIN_SERVO_PWM, LEDC_CHANNEL_4, SERVO_TIMER);
    if (err != ESP_OK) {
        return err;
    }

    memset(&s_state, 0, sizeof(s_state));
    return stop_all_outputs(true);
}

static esp_err_t uart_init(void)
{
    const uart_config_t config = {
        .baud_rate = PCB_TEST_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t err = uart_param_config(TEST_UART, &config);
    if (err != ESP_OK) {
        return err;
    }
    err = uart_set_pin(TEST_UART, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        return err;
    }
    return uart_driver_install(TEST_UART, TEST_UART_BUFFER_SIZE * 2, 0, 0, NULL, 0);
}

static bool parse_int(const char *text, int minimum, int maximum, int *value)
{
    if (text == NULL || value == NULL) {
        return false;
    }
    char *end = NULL;
    const long parsed = strtol(text, &end, 10);
    if (end == text || *end != '\0' || parsed < minimum || parsed > maximum) {
        return false;
    }
    *value = (int)parsed;
    return true;
}

static bool require_armed(void)
{
    if (!s_state.armed) {
        ESP_LOGW(TAG, "output rejected: type 'arm' first");
        return false;
    }
    if (s_state.fault_latched || gpio_get_level(PCB_TEST_PIN_DRV_NFAULT) == 0) {
        ESP_LOGE(TAG, "output rejected: nFAULT is active; fix hardware then type 'clear'");
        return false;
    }
    return true;
}

static esp_err_t set_motor(bool motor_a, int percent)
{
    const uint32_t duty = (uint32_t)(abs(percent) * (int)MOTOR_MAX_DUTY / 100);
    const ledc_channel_t input_1 = motor_a ? LEDC_CHANNEL_0 : LEDC_CHANNEL_2;
    const ledc_channel_t input_2 = motor_a ? LEDC_CHANNEL_1 : LEDC_CHANNEL_3;
    const uint32_t duty_1 = percent > 0 ? duty : 0;
    const uint32_t duty_2 = percent < 0 ? duty : 0;

    esp_err_t err = set_pwm(input_1, duty_1);
    if (err != ESP_OK) {
        return err;
    }
    err = set_pwm(input_2, duty_2);
    if (err != ESP_OK) {
        (void)set_pwm(input_1, 0);
        return err;
    }
    if (motor_a) {
        s_state.motor_a_percent = percent;
        s_state.ain1_duty = duty_1;
        s_state.ain2_duty = duty_2;
    } else {
        s_state.motor_b_percent = percent;
        s_state.bin1_duty = duty_1;
        s_state.bin2_duty = duty_2;
    }
    refresh_output_watchdog();
    return ESP_OK;
}

static esp_err_t set_both_motors(int motor_a, int motor_b)
{
    esp_err_t err = set_motor(true, motor_a);
    if (err != ESP_OK) {
        return err;
    }
    err = set_motor(false, motor_b);
    if (err != ESP_OK) {
        (void)stop_all_outputs(true);
    }
    return err;
}

static esp_err_t set_servo(int angle_deg)
{
    const uint32_t pulse_us = PCB_TEST_SERVO_MIN_PULSE_US +
        (uint32_t)angle_deg *
        (PCB_TEST_SERVO_MAX_PULSE_US - PCB_TEST_SERVO_MIN_PULSE_US) / 180U;
    const uint32_t duty = (uint32_t)(((uint64_t)pulse_us * PCB_TEST_SERVO_PWM_HZ *
                                      SERVO_MAX_DUTY) / 1000000ULL);
    esp_err_t err = set_pwm(LEDC_CHANNEL_4, duty);
    if (err == ESP_OK) {
        s_state.servo_enabled = true;
        s_state.servo_angle_deg = angle_deg;
        s_state.servo_pulse_us = pulse_us;
        s_state.servo_duty = duty;
        refresh_output_watchdog();
    }
    return err;
}

static esp_err_t set_servo_off(void)
{
    esp_err_t err = set_pwm(LEDC_CHANNEL_4, 0);
    if (err == ESP_OK) {
        s_state.servo_enabled = false;
        s_state.servo_pulse_us = 0;
        s_state.servo_duty = 0;
    }
    return err;
}

static esp_err_t set_magnet(bool on)
{
    esp_err_t err = gpio_set_level(PCB_TEST_PIN_MAGNET, on ? 1 : 0);
    if (err == ESP_OK) {
        s_state.magnet_on = on;
        if (on) {
            refresh_output_watchdog();
        }
    }
    return err;
}

static void print_help(void)
{
    ESP_LOGI(TAG, "help | status | arm | stop | clear | test");
    ESP_LOGI(TAG, "motor <a|b> <-100..100> | motors <A> <B>");
    ESP_LOGI(TAG, "servo <0..180|off> | magnet <on|off>");
    ESP_LOGI(TAG, "active outputs stop after %d ms without a new command",
             PCB_TEST_OUTPUT_TIMEOUT_MS);
}

static void print_status(void)
{
    int64_t watchdog_ms = 0;
    if (outputs_active() && s_state.output_deadline_us > esp_timer_get_time()) {
        watchdog_ms = (s_state.output_deadline_us - esp_timer_get_time()) / 1000;
    }
    ESP_LOGI(TAG, "armed=%d fault_latched=%d nFAULT(GPIO4)=%s watchdog=%lldms",
             s_state.armed, s_state.fault_latched,
             gpio_get_level(PCB_TEST_PIN_DRV_NFAULT) ? "HIGH/OK" : "LOW/FAULT",
             (long long)watchdog_ms);
    ESP_LOGI(TAG, "nSLEEP(GPIO3)=%d motorA=%d%% AIN1(GPIO46)=%u/%u AIN2(GPIO9)=%u/%u",
             gpio_get_level(PCB_TEST_PIN_DRV_NSLEEP), s_state.motor_a_percent,
             (unsigned)s_state.ain1_duty, (unsigned)MOTOR_MAX_DUTY,
             (unsigned)s_state.ain2_duty, (unsigned)MOTOR_MAX_DUTY);
    ESP_LOGI(TAG, "motorB=%d%% BIN1(GPIO5)=%u/%u BIN2(GPIO6)=%u/%u PWM=%dHz",
             s_state.motor_b_percent, (unsigned)s_state.bin1_duty,
             (unsigned)MOTOR_MAX_DUTY, (unsigned)s_state.bin2_duty,
             (unsigned)MOTOR_MAX_DUTY, PCB_TEST_MOTOR_PWM_HZ);
    ESP_LOGI(TAG, "servo(GPIO45)=%s angle=%d pulse=%uus PWM=%dHz",
             s_state.servo_enabled ? "ON" : "OFF", s_state.servo_angle_deg,
             (unsigned)s_state.servo_pulse_us, PCB_TEST_SERVO_PWM_HZ);
    ESP_LOGI(TAG, "magnet(GPIO35)=%s GPIO=%d external-interface=%s",
             s_state.magnet_on ? "ON" : "OFF",
             gpio_get_level(PCB_TEST_PIN_MAGNET),
             s_state.magnet_on ? "LOW/asserted" : "released");
}

static bool wait_test_step(uint32_t milliseconds)
{
    const int64_t end_us = esp_timer_get_time() + (int64_t)milliseconds * 1000;
    while (esp_timer_get_time() < end_us) {
        if (gpio_get_level(PCB_TEST_PIN_DRV_NFAULT) == 0) {
            s_state.fault_latched = true;
            (void)stop_all_outputs(true);
            ESP_LOGE(TAG, "automatic test aborted: nFAULT went LOW");
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return true;
}

static bool automatic_test_stage(const char *name, esp_err_t result, uint32_t hold_ms)
{
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "%s failed: %s", name, esp_err_to_name(result));
        (void)stop_all_outputs(true);
        return false;
    }
    ESP_LOGI(TAG, "AUTO: %s", name);
    return wait_test_step(hold_ms);
}

static void run_automatic_test(void)
{
    if (!require_armed()) {
        return;
    }
    ESP_LOGW(TAG, "AUTO starts: motors use +/-10%%; keep the board clear");

    if (!automatic_test_stage("motor A +10%", set_motor(true, PCB_TEST_AUTO_MOTOR_PERCENT), 700) ||
        !automatic_test_stage("motor A stop", set_motor(true, 0), 250) ||
        !automatic_test_stage("motor A -10%", set_motor(true, -PCB_TEST_AUTO_MOTOR_PERCENT), 700) ||
        !automatic_test_stage("motor A stop", set_motor(true, 0), 250) ||
        !automatic_test_stage("motor B +10%", set_motor(false, PCB_TEST_AUTO_MOTOR_PERCENT), 700) ||
        !automatic_test_stage("motor B stop", set_motor(false, 0), 250) ||
        !automatic_test_stage("motor B -10%", set_motor(false, -PCB_TEST_AUTO_MOTOR_PERCENT), 700) ||
        !automatic_test_stage("motor B stop", set_motor(false, 0), 250) ||
        !automatic_test_stage("magnet asserted (external interface LOW)", set_magnet(true), 500) ||
        !automatic_test_stage("magnet released", set_magnet(false), 250) ||
        !automatic_test_stage("servo 45 degrees", set_servo(45), 600) ||
        !automatic_test_stage("servo 90 degrees", set_servo(90), 600) ||
        !automatic_test_stage("servo 135 degrees", set_servo(135), 600)) {
        return;
    }
    (void)stop_all_outputs(true);
    ESP_LOGI(TAG, "AUTO complete; all outputs OFF and tester disarmed");
}

static void log_command_error(const char *operation, esp_err_t err)
{
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s failed: %s; stopping all outputs", operation,
                 esp_err_to_name(err));
        (void)stop_all_outputs(true);
    }
}

static void handle_command(char *line)
{
    char *save = NULL;
    char *command = strtok_r(line, " \t\r\n", &save);
    if (command == NULL) {
        return;
    }

    if (strcmp(command, "help") == 0) {
        print_help();
    } else if (strcmp(command, "status") == 0) {
        print_status();
    } else if (strcmp(command, "arm") == 0) {
        if (s_state.fault_latched || gpio_get_level(PCB_TEST_PIN_DRV_NFAULT) == 0) {
            ESP_LOGE(TAG, "cannot arm while nFAULT is active/latched");
            return;
        }
        log_command_error("arm", gpio_set_level(PCB_TEST_PIN_DRV_NSLEEP, 1));
        if (gpio_get_level(PCB_TEST_PIN_DRV_NSLEEP) == 1) {
            s_state.armed = true;
            ESP_LOGW(TAG, "ARMED; outputs remain zero until a test command");
        }
    } else if (strcmp(command, "stop") == 0) {
        log_command_error("stop", stop_all_outputs(true));
        ESP_LOGI(TAG, "all outputs OFF; tester disarmed");
    } else if (strcmp(command, "clear") == 0) {
        if (gpio_get_level(PCB_TEST_PIN_DRV_NFAULT) == 0) {
            ESP_LOGE(TAG, "cannot clear: nFAULT is still LOW");
        } else {
            s_state.fault_latched = false;
            ESP_LOGI(TAG, "fault latch cleared; tester remains disarmed");
        }
    } else if (strcmp(command, "motor") == 0) {
        char *axis = strtok_r(NULL, " \t\r\n", &save);
        int percent;
        if (axis == NULL || (strcmp(axis, "a") != 0 && strcmp(axis, "b") != 0) ||
            !parse_int(strtok_r(NULL, " \t\r\n", &save), -100, 100, &percent)) {
            ESP_LOGW(TAG, "usage: motor <a|b> <-100..100>");
        } else if (require_armed()) {
            log_command_error("motor", set_motor(strcmp(axis, "a") == 0, percent));
        }
    } else if (strcmp(command, "motors") == 0) {
        int motor_a;
        int motor_b;
        if (!parse_int(strtok_r(NULL, " \t\r\n", &save), -100, 100, &motor_a) ||
            !parse_int(strtok_r(NULL, " \t\r\n", &save), -100, 100, &motor_b)) {
            ESP_LOGW(TAG, "usage: motors <A:-100..100> <B:-100..100>");
        } else if (require_armed()) {
            log_command_error("motors", set_both_motors(motor_a, motor_b));
        }
    } else if (strcmp(command, "servo") == 0) {
        char *value = strtok_r(NULL, " \t\r\n", &save);
        int angle;
        if (value != NULL && strcmp(value, "off") == 0) {
            log_command_error("servo off", set_servo_off());
        } else if (!parse_int(value, 0, 180, &angle)) {
            ESP_LOGW(TAG, "usage: servo <0..180|off>");
        } else if (require_armed()) {
            log_command_error("servo", set_servo(angle));
        }
    } else if (strcmp(command, "magnet") == 0) {
        char *value = strtok_r(NULL, " \t\r\n", &save);
        if (value == NULL || (strcmp(value, "on") != 0 && strcmp(value, "off") != 0)) {
            ESP_LOGW(TAG, "usage: magnet <on|off>");
        } else if (strcmp(value, "off") == 0) {
            log_command_error("magnet off", set_magnet(false));
        } else if (require_armed()) {
            log_command_error("magnet on", set_magnet(true));
        }
    } else if (strcmp(command, "test") == 0) {
        run_automatic_test();
    } else {
        ESP_LOGW(TAG, "unknown command: %s (type 'help')", command);
    }
}

static void service_safety(void)
{
    if (gpio_get_level(PCB_TEST_PIN_DRV_NFAULT) == 0 && !s_state.fault_latched) {
        s_state.fault_latched = true;
        (void)stop_all_outputs(true);
        ESP_LOGE(TAG, "nFAULT LOW: all outputs stopped and fault latched");
    }
    if (outputs_active() && s_state.output_deadline_us > 0 &&
        esp_timer_get_time() >= s_state.output_deadline_us) {
        (void)stop_all_outputs(true);
        ESP_LOGW(TAG, "output watchdog expired: all outputs OFF and tester disarmed");
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "=== ESP32-S3 PCB SIGNAL TESTER ===");
    ESP_LOGI(TAG, "build %s %s", __DATE__, __TIME__);

    ESP_ERROR_CHECK(hardware_init());
    ESP_ERROR_CHECK(uart_init());

    ESP_LOGI(TAG, "safe boot: motors asleep, servo OFF, magnet released");
    ESP_LOGI(TAG, "UART0 %d baud; type 'help'", PCB_TEST_UART_BAUD_RATE);
    print_status();

    char line[TEST_UART_BUFFER_SIZE];
    size_t used = 0;
    while (true) {
        service_safety();

        uint8_t byte = 0;
        const int length = uart_read_bytes(TEST_UART, &byte, 1, pdMS_TO_TICKS(20));
        if (length <= 0) {
            continue;
        }
        if (byte == '\r' || byte == '\n') {
            if (used > 0) {
                line[used] = '\0';
                handle_command(line);
                used = 0;
            }
        } else if ((byte == '\b' || byte == 0x7f) && used > 0) {
            --used;
        } else if (byte >= 0x20 && byte <= 0x7e && used < sizeof(line) - 1) {
            line[used++] = (char)byte;
        }
    }
}

