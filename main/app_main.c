#include <inttypes.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "board_pins.h"
#include "wifi_control.h"

static const char *TAG = "app";

#define APP_UART_NUM UART_NUM_0
#define UART_BUFFER_SIZE 1024
#define MOTOR_PWM_MAX_DUTY 1023

static bool s_motor_driver_available;

static void uart_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(APP_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(APP_UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(APP_UART_NUM, UART_BUFFER_SIZE * 2, 0, 0, NULL, 0));
}

static esp_err_t motor_init(void)
{
    // Hold the bridge asleep until every motor control pin is known to be safe.
    const gpio_config_t sleep_config = {
        .pin_bit_mask = (1ULL << PIN_DRV_NSLEEP),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&sleep_config), TAG, "configure DRV8833 nSLEEP");
    ESP_RETURN_ON_ERROR(gpio_set_level(PIN_DRV_NSLEEP, 0), TAG, "hold DRV8833 asleep");

    const gpio_config_t fault_config = {
        .pin_bit_mask = (1ULL << PIN_DRV_NFAULT),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&fault_config), TAG, "configure DRV8833 nFAULT");

    if (!GPIO_IS_VALID_OUTPUT_GPIO(PIN_DRV_AIN1)) {
        ESP_LOGE(TAG,
                 "Motor driver disabled: schematic routes AIN1 to input-only GPIO%d; "
                 "rework AIN1 to an output-capable GPIO",
                 PIN_DRV_AIN1);
        s_motor_driver_available = false;
        return ESP_ERR_NOT_SUPPORTED;
    }

    const ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 16000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer_config), TAG, "configure motor PWM timer");

    ledc_channel_config_t channel_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .gpio_num = PIN_DRV_AIN1,
        .intr_type = LEDC_INTR_DISABLE,
        .hpoint = 0,
    };
    ESP_RETURN_ON_ERROR(ledc_channel_config(&channel_config), TAG, "configure motor AIN1");

    channel_config.channel = LEDC_CHANNEL_1;
    channel_config.gpio_num = PIN_DRV_AIN2;
    ESP_RETURN_ON_ERROR(ledc_channel_config(&channel_config), TAG, "configure motor AIN2");

    channel_config.channel = LEDC_CHANNEL_2;
    channel_config.gpio_num = PIN_DRV_BIN1;
    ESP_RETURN_ON_ERROR(ledc_channel_config(&channel_config), TAG, "configure motor BIN1");

    channel_config.channel = LEDC_CHANNEL_3;
    channel_config.gpio_num = PIN_DRV_BIN2;
    ESP_RETURN_ON_ERROR(ledc_channel_config(&channel_config), TAG, "configure motor BIN2");

    ESP_RETURN_ON_ERROR(gpio_set_level(PIN_DRV_NSLEEP, 1), TAG, "enable DRV8833");
    s_motor_driver_available = true;
    return ESP_OK;
}

static esp_err_t servo_init(void)
{
    const gpio_config_t output_config = {
        .pin_bit_mask = (1ULL << PIN_SERVO_PWM),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&output_config), TAG, "configure servo GPIO");

    const ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_14_BIT,
        .timer_num = LEDC_TIMER_1,
        .freq_hz = 50,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer_config), TAG, "configure servo PWM timer");

    const ledc_channel_config_t channel_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_4,
        .timer_sel = LEDC_TIMER_1,
        .duty = 0,
        .gpio_num = PIN_SERVO_PWM,
        .intr_type = LEDC_INTR_DISABLE,
        .hpoint = 0,
    };
    return ledc_channel_config(&channel_config);
}

static esp_err_t i2c_init(void)
{
    const i2c_config_t config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = PIN_I2C_SDA,
        .scl_io_num = PIN_I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
    };
    ESP_RETURN_ON_ERROR(i2c_param_config(I2C_MASTER_NUM, &config), TAG, "configure I2C");
    return i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER, 0, 0, 0);
}

static esp_err_t set_motor_channels(int speed_percent)
{
    if (!s_motor_driver_available) {
        ESP_LOGW(TAG, "Motor command ignored: DRV8833 is safely asleep because GPIO46 cannot drive AIN1");
        return ESP_ERR_NOT_SUPPORTED;
    }

    if (speed_percent > 100) {
        speed_percent = 100;
    } else if (speed_percent < -100) {
        speed_percent = -100;
    }

    const uint32_t duty = (abs(speed_percent) * MOTOR_PWM_MAX_DUTY) / 100;
    const uint32_t in1_duty = speed_percent > 0 ? duty : 0;
    const uint32_t in2_duty = speed_percent < 0 ? duty : 0;

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

static esp_err_t set_motor_speed(int speed_percent)
{
    esp_err_t err = set_motor_channels(speed_percent);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Motor speed: %d%%", speed_percent);
    }
    return err;
}

static esp_err_t set_servo_angle(int angle)
{
    if (angle < 0) {
        angle = 0;
    } else if (angle > 180) {
        angle = 180;
    }

    // At 50 Hz and 14-bit resolution: 500 us = 410 ticks, 2500 us = 2048 ticks.
    const int duty_ticks = 410 + (angle * 1638 / 180);
    ESP_RETURN_ON_ERROR(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4, duty_ticks),
                        TAG, "set servo duty");
    ESP_RETURN_ON_ERROR(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4),
                        TAG, "update servo duty");
    ESP_LOGI(TAG, "Servo angle: %d degrees", angle);
    return ESP_OK;
}

static void i2c_scan(void)
{
    ESP_LOGI(TAG, "I2C scan start...");
    for (uint8_t address = 1; address < 128; ++address) {
        i2c_cmd_handle_t command = i2c_cmd_link_create();
        i2c_master_start(command);
        i2c_master_write_byte(command, (address << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(command);
        esp_err_t err = i2c_master_cmd_begin(I2C_MASTER_NUM, command,
                                              pdMS_TO_TICKS(50));
        i2c_cmd_link_delete(command);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "I2C device found: 0x%02X", address);
        }
    }
    ESP_LOGI(TAG, "I2C scan done");
}

static void read_mpu6050(void)
{
    static const uint8_t who_am_i_register = 0x75;
    uint8_t who_am_i = 0;
    esp_err_t err = i2c_master_write_read_device(I2C_MASTER_NUM, MPU6050_I2C_ADDR,
                                                  &who_am_i_register, 1,
                                                  &who_am_i, 1,
                                                  pdMS_TO_TICKS(100));
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "MPU6050 WHO_AM_I: 0x%02X (expected 0x68)", who_am_i);
    } else {
        ESP_LOGW(TAG, "MPU6050 read failed: %s", esp_err_to_name(err));
    }
}

static void parse_command(char *command)
{
    char *token = strtok(command, " \r\n");
    if (token == NULL) {
        return;
    }

    if (strcmp(token, "motor") == 0) {
        token = strtok(NULL, " \r\n");
        if (token != NULL) {
            ESP_ERROR_CHECK_WITHOUT_ABORT(set_motor_speed(atoi(token)));
        }
    } else if (strcmp(token, "servo") == 0) {
        token = strtok(NULL, " \r\n");
        if (token != NULL) {
            ESP_ERROR_CHECK_WITHOUT_ABORT(set_servo_angle(atoi(token)));
        }
    } else if (strcmp(token, "i2c") == 0) {
        token = strtok(NULL, " \r\n");
        if (token != NULL && strcmp(token, "scan") == 0) {
            i2c_scan();
        }
    } else if (strcmp(token, "imu") == 0) {
        read_mpu6050();
    } else if (strcmp(token, "help") == 0) {
        ESP_LOGI(TAG, "Commands:");
        ESP_LOGI(TAG, "  motor <-100..100> - signed motor speed");
        ESP_LOGI(TAG, "  servo <0..180>    - servo angle");
        ESP_LOGI(TAG, "  i2c scan          - scan I2C devices");
        ESP_LOGI(TAG, "  imu               - read MPU6050 WHO_AM_I");
    } else {
        ESP_LOGW(TAG, "Unknown command: %s", token);
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "=== ESP32-S3 Robot Controller ===");
    ESP_LOGI(TAG, "Build time: " __TIME__ " " __DATE__);

    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    } else {
        ESP_ERROR_CHECK(nvs_err);
    }

    uart_init();

    esp_err_t motor_err = motor_init();
    if (motor_err != ESP_OK) {
        ESP_LOGW(TAG, "Motor subsystem unavailable: %s", esp_err_to_name(motor_err));
    }
    ESP_ERROR_CHECK(servo_init());
    ESP_ERROR_CHECK(set_servo_angle(90));
    ESP_ERROR_CHECK(i2c_init());
    ESP_LOGI(TAG, "Peripheral drivers initialized");

    ESP_LOGI(TAG, "Initializing Wi-Fi control...");
    esp_err_t wifi_err = wifi_control_init(set_motor_speed, set_servo_angle);
    if (wifi_err == ESP_OK) {
        ESP_LOGI(TAG, "Wi-Fi ready: connect to %s (password: %s)",
                 WIFI_CONTROL_AP_SSID, WIFI_CONTROL_AP_PASSWORD);
        ESP_LOGI(TAG, "Open http://%s in a browser", WIFI_CONTROL_AP_IP);
    } else {
        ESP_LOGE(TAG, "Wi-Fi control unavailable: %s", esp_err_to_name(wifi_err));
    }

    ESP_LOGI(TAG, "Type 'help' for serial commands");

    char command_buffer[UART_BUFFER_SIZE];
    int command_length = 0;

    while (true) {
        uint8_t data;
        int length = uart_read_bytes(APP_UART_NUM, &data, 1, pdMS_TO_TICKS(10));
        if (length <= 0) {
            continue;
        }

        if (data == '\n' || data == '\r') {
            if (command_length > 0) {
                command_buffer[command_length] = '\0';
                ESP_LOGI(TAG, "> %s", command_buffer);
                parse_command(command_buffer);
                command_length = 0;
            }
        } else if (command_length < UART_BUFFER_SIZE - 1) {
            command_buffer[command_length++] = (char)data;
        }
    }
}
