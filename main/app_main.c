#include <inttypes.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c.h"
#include "esp_timer.h"

#include "ble_control.h"
#include "board_pins.h"

static const char *TAG = "app";

// UART
#define UART_NUM UART_NUM_0
#define BUF_SIZE 1024

#define MOTOR_PWM_MAX_DUTY 1023

static bool s_motor_driver_available;

static void uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
}

static esp_err_t motor_init(void)
{
    // Configure nSLEEP first and hold the bridge disabled until every control
    // input has been validated and configured.
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_DRV_NSLEEP),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_config(&io_conf));
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_level(PIN_DRV_NSLEEP, 0));

    gpio_config_t fault_conf = {
        .pin_bit_mask = (1ULL << PIN_DRV_NFAULT),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_config(&fault_conf));

    if (!GPIO_IS_VALID_OUTPUT_GPIO(PIN_DRV_AIN1)) {
        ESP_LOGE(TAG,
                 "Motor driver disabled: schematic routes AIN1 to input-only GPIO%d; "
                 "rework AIN1 to an output-capable GPIO",
                 PIN_DRV_AIN1);
        s_motor_driver_available = false;
        return ESP_ERR_NOT_SUPPORTED;
    }

    // PWM for motor speed (16kHz)
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 16000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer_conf);

    ledc_channel_config_t ledc_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .gpio_num = PIN_DRV_AIN1,
        .intr_type = LEDC_INTR_DISABLE,
        .hpoint = 0,
    };
    ESP_RETURN_ON_ERROR(ledc_channel_config(&ledc_conf), TAG, "motor AIN1 PWM");

    ledc_conf.channel = LEDC_CHANNEL_1;
    ledc_conf.gpio_num = PIN_DRV_AIN2;
    ESP_RETURN_ON_ERROR(ledc_channel_config(&ledc_conf), TAG, "motor AIN2 PWM");

    ledc_conf.channel = LEDC_CHANNEL_2;
    ledc_conf.gpio_num = PIN_DRV_BIN1;
    ESP_RETURN_ON_ERROR(ledc_channel_config(&ledc_conf), TAG, "motor BIN1 PWM");

    ledc_conf.channel = LEDC_CHANNEL_3;
    ledc_conf.gpio_num = PIN_DRV_BIN2;
    ESP_RETURN_ON_ERROR(ledc_channel_config(&ledc_conf), TAG, "motor BIN2 PWM");

    // Enable motor driver
    ESP_RETURN_ON_ERROR(gpio_set_level(PIN_DRV_NSLEEP, 1), TAG, "enable DRV8833");
    s_motor_driver_available = true;
    return ESP_OK;
}

static void servo_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_SERVO_PWM),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    // 50Hz for servo
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_14_BIT,
        .timer_num = LEDC_TIMER_1,
        .freq_hz = 50,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer_conf);

    ledc_channel_config_t ledc_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_4,
        .timer_sel = LEDC_TIMER_1,
        .duty = 0,
        .gpio_num = PIN_SERVO_PWM,
    };
    ledc_channel_config(&ledc_conf);
}

static void i2c_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = PIN_I2C_SDA,
        .scl_io_num = PIN_I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER, 0, 0, 0));
}

static esp_err_t set_motor_channels(int speed_percent)
{
    if (!s_motor_driver_available) {
        ESP_LOGW(TAG, "Motor command ignored: DRV8833 is safely asleep because GPIO46 cannot drive AIN1");
        return ESP_ERR_NOT_SUPPORTED;
    }

    if (speed_percent > 100) speed_percent = 100;
    if (speed_percent < -100) speed_percent = -100;

    uint32_t duty = (abs(speed_percent) * MOTOR_PWM_MAX_DUTY) / 100;
    uint32_t in1_duty = speed_percent > 0 ? duty : 0;
    uint32_t in2_duty = speed_percent < 0 ? duty : 0;

    ESP_RETURN_ON_ERROR(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, in1_duty), TAG, "set AIN1");
    ESP_RETURN_ON_ERROR(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0), TAG, "update AIN1");
    ESP_RETURN_ON_ERROR(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, in2_duty), TAG, "set AIN2");
    ESP_RETURN_ON_ERROR(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1), TAG, "update AIN2");
    ESP_RETURN_ON_ERROR(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, in1_duty), TAG, "set BIN1");
    ESP_RETURN_ON_ERROR(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2), TAG, "update BIN1");
    ESP_RETURN_ON_ERROR(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, in2_duty), TAG, "set BIN2");
    ESP_RETURN_ON_ERROR(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3), TAG, "update BIN2");
    return ESP_OK;
}

// BLE callbacks
static void ble_motor_callback(uint8_t duty_percent)
{
    set_motor_channels(duty_percent);
    ESP_LOGI(TAG, "[BLE] Motor: %d%%", duty_percent);
}

static void ble_servo_callback(uint8_t angle_deg)
{
    if (angle_deg > 180) angle_deg = 180;

    // 50Hz, 14-bit resolution
    // Period = 20ms = 20000us, ticks = 16384 (14-bit)
    // 500us = 410 ticks, 2500us = 2048 ticks
    int duty_ticks = 410 + (angle_deg * 1638 / 180);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4, duty_ticks);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4);

    ESP_LOGI(TAG, "[BLE] Servo: %d°", angle_deg);
}

static void set_motor_speed(int speed_percent)
{
    set_motor_channels(speed_percent);
    ESP_LOGI(TAG, "Motor speed: %d%%", speed_percent);
}

static void set_servo_angle(int angle)
{
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;

    int duty_ticks = 410 + (angle * 1638 / 180);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4, duty_ticks);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4);

    ESP_LOGI(TAG, "Servo angle: %d°", angle);
}

static void i2c_scan(void)
{
    uint8_t addr;
    ESP_LOGI(TAG, "I2C scan start...");

    for (addr = 1; addr < 128; addr++) {
        int ret;
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 50 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);

        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "I2C device found: 0x%02X", addr);
        }
    }
    ESP_LOGI(TAG, "I2C scan done.");
}

static void read_mpu6050(void)
{
    static const uint8_t who_am_i_reg = 0x75;
    uint8_t who_am_i = 0;
    esp_err_t err = i2c_master_write_read_device(I2C_MASTER_NUM, MPU6050_I2C_ADDR,
                                                  &who_am_i_reg, 1, &who_am_i, 1,
                                                  100 / portTICK_PERIOD_MS);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "MPU6050 WHO_AM_I: 0x%02X (expected 0x68)", who_am_i);
    } else {
        ESP_LOGW(TAG, "MPU6050 read failed: %s", esp_err_to_name(err));
    }
}

static void parse_command(char *cmd)
{
    char *token = strtok(cmd, " \r\n");
    if (token == NULL) return;

    if (strcmp(token, "motor") == 0) {
        token = strtok(NULL, " \r\n");
        if (token) {
            int speed = atoi(token);
            set_motor_speed(speed);
        }
    } else if (strcmp(token, "servo") == 0) {
        token = strtok(NULL, " \r\n");
        if (token) {
            int angle = atoi(token);
            set_servo_angle(angle);
        }
    } else if (strcmp(token, "i2c") == 0) {
        token = strtok(NULL, " \r\n");
        if (token && strcmp(token, "scan") == 0) {
            i2c_scan();
        }
    } else if (strcmp(token, "imu") == 0) {
        read_mpu6050();
    } else if (strcmp(token, "help") == 0) {
        ESP_LOGI(TAG, "Commands:");
        ESP_LOGI(TAG, "  motor <0-100>  - set motor speed");
        ESP_LOGI(TAG, "  servo <0-180>  - set servo angle");
        ESP_LOGI(TAG, "  i2c scan       - scan I2C devices");
        ESP_LOGI(TAG, "  imu            - read MPU6050");
    } else {
        ESP_LOGW(TAG, "Unknown command: %s", token);
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "=== ESP32-S3 Component Test ===");
    ESP_LOGI(TAG, "Build time: " __TIME__ " " __DATE__);

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    ESP_LOGI(TAG, "NVS initialized");

    // Initialize peripherals
    uart_init();
    esp_err_t motor_ret = motor_init();
    if (motor_ret != ESP_OK) {
        ESP_LOGW(TAG, "Motor subsystem unavailable: %s", esp_err_to_name(motor_ret));
    }
    servo_init();
    i2c_init();

    ESP_LOGI(TAG, "All drivers initialized");

    // Initialize BLE
    ESP_LOGI(TAG, "Initializing BLE...");
    ble_control_init(ble_motor_callback, ble_servo_callback);
    ESP_LOGI(TAG, "BLE ready! Device name: ESP32-Robot");

    ESP_LOGI(TAG, "Type 'help' for serial commands");
    ESP_LOGI(TAG, "Or use BLE app to connect and control");

    // Command buffer
    char cmd_buf[BUF_SIZE];
    int cmd_len = 0;

    while (1) {
        uint8_t data;
        int len = uart_read_bytes(UART_NUM, &data, 1, 10 / portTICK_PERIOD_MS);
        if (len > 0) {
            if (data == '\n' || data == '\r') {
                if (cmd_len > 0) {
                    cmd_buf[cmd_len] = '\0';
                    ESP_LOGI(TAG, "> %s", cmd_buf);
                    parse_command(cmd_buf);
                    cmd_len = 0;
                }
            } else if (cmd_len < BUF_SIZE - 1) {
                cmd_buf[cmd_len++] = data;
            }
        }
    }
}
