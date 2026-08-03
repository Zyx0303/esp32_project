#include "app_console.h"

#include <inttypes.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/uart.h"
#include "esp_log.h"
#include "esp_netif_ip_addr.h"
#include "esp_timer.h"

#include "app_control.h"
#include "app_imu.h"
#include "app_status.h"
#include "robot_state_machine.h"
#include "robot_types.h"
#include "wifi_control.h"

static const char *TAG = "app_console";

#define APP_UART_NUM UART_NUM_0
#define UART_BUFFER_SIZE 1024

static uint32_t s_serial_sequence;

esp_err_t app_console_init(void)
{
    const uart_config_t config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_param_config(APP_UART_NUM, &config);
    if (err != ESP_OK) {
        return err;
    }
    err = uart_set_pin(APP_UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        return err;
    }
    return uart_driver_install(APP_UART_NUM, UART_BUFFER_SIZE * 2, 0, 0, NULL, 0);
}

/** 为串口来源生成统一的命令元数据，使其经过与 HTTP 相同的校验链路。 */
static robot_command_t serial_command(robot_command_type_t type)
{
    return (robot_command_t) {
        .type = type,
        .sequence = ++s_serial_sequence,
        .received_at_us = esp_timer_get_time(),
        .source = ROBOT_COMMAND_SOURCE_SERIAL,
    };
}

/** 严格解析十进制整数：拒绝空值、尾随字符和超出命令范围的数值。 */
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

/** 输出只读状态；该函数不 ARM，也不直接访问任何执行器。 */
static void print_status(void)
{
    robot_status_t status;
    if (app_status_get(&status)) {
        ESP_LOGI(TAG, "state=%s armed=%d estop=%d faults=0x%08" PRIx32
                 " motorA=%d/%d motorB=%d/%d servo=%d enabled=%d power=%d",
                 robot_state_name(status.state), status.armed, status.estop_latched,
                 status.faults, status.motor_a_target, status.motor_a_output,
                 status.motor_b_target, status.motor_b_output, status.servo_target_deg,
                 status.servo_enabled, status.power_asserted);
    }

    // 串口 status 始终显示 DHCP 地址，便于在启动日志滚走后重新找到设备。
    wifi_control_network_status_t network;
    if (wifi_control_get_network_status(&network)) {
        const esp_ip4_addr_t ip = {.addr = network.sta_ip};
        ESP_LOGI(TAG, "network=%s sta_connected=%d sta_ip=" IPSTR
                 " reconnects=%" PRIu32,
                 network.sta_configured ? "APSTA" : "AP",
                 network.sta_connected, IP2STR(&ip), network.reconnect_count);
    }
}

/**
 * 把一行 UART 文本适配为 robot_command_t。
 *
 * imu/status/help 是本地只读命令；其余改变状态的命令全部进入控制队列，
 * 串口线程绝不直接调用电机、舵机或电源驱动。
 */
static void parse_serial_command(char *line)
{
    char *name = strtok(line, " \r\n");
    if (name == NULL) {
        return;
    }

    robot_command_t command;
    bool submit = true;
    if (strcmp(name, "arm") == 0) {
        command = serial_command(ROBOT_CMD_ARM);
    } else if (strcmp(name, "disarm") == 0) {
        command = serial_command(ROBOT_CMD_DISARM);
    } else if (strcmp(name, "estop") == 0) {
        command = serial_command(ROBOT_CMD_ESTOP);
    } else if (strcmp(name, "clear") == 0) {
        command = serial_command(ROBOT_CMD_CLEAR_FAULT);
    } else if (strcmp(name, "drive") == 0) {
        int throttle;
        int steering;
        if (!parse_int(strtok(NULL, " \r\n"), -100, 100, &throttle) ||
            !parse_int(strtok(NULL, " \r\n"), -100, 100, &steering)) {
            ESP_LOGW(TAG, "usage: drive <-100..100> <-100..100>");
            return;
        }
        command = serial_command(ROBOT_CMD_DRIVE);
        command.value.drive.throttle = throttle;
        command.value.drive.steering = steering;
    } else if (strcmp(name, "motors") == 0) {
        int motor_a;
        int motor_b;
        if (!parse_int(strtok(NULL, " \r\n"), -100, 100, &motor_a) ||
            !parse_int(strtok(NULL, " \r\n"), -100, 100, &motor_b)) {
            ESP_LOGW(TAG, "usage: motors <-100..100> <-100..100>");
            return;
        }
        command = serial_command(ROBOT_CMD_MOTOR_DIRECT);
        command.value.motors.motor_a = motor_a;
        command.value.motors.motor_b = motor_b;
    } else if (strcmp(name, "servo") == 0) {
        int angle;
        if (!parse_int(strtok(NULL, " \r\n"), 0, 180, &angle)) {
            ESP_LOGW(TAG, "usage: servo <0..180>");
            return;
        }
        command = serial_command(ROBOT_CMD_SERVO);
        command.value.servo.angle_deg = angle;
    } else if (strcmp(name, "power") == 0) {
        int asserted;
        if (!parse_int(strtok(NULL, " \r\n"), 0, 1, &asserted)) {
            ESP_LOGW(TAG, "usage: power <0|1>");
            return;
        }
        command = serial_command(ROBOT_CMD_POWER_CONTROL);
        command.value.power.asserted = asserted == 1;
    } else if (strcmp(name, "imu") == 0) {
        app_imu_log_latest();
        submit = false;
    } else if (strcmp(name, "status") == 0) {
        print_status();
        submit = false;
    } else if (strcmp(name, "help") == 0) {
        ESP_LOGI(TAG, "arm | disarm | estop | clear | status | imu");
        ESP_LOGI(TAG, "drive <throttle> <steering> | motors <A> <B>");
        ESP_LOGI(TAG, "servo <0..180> | power <0|1>");
        submit = false;
    } else {
        ESP_LOGW(TAG, "unknown command: %s", name);
        submit = false;
    }

    if (submit && app_control_enqueue(&command) != ESP_OK) {
        ESP_LOGE(TAG, "command queue full");
    }
}

void app_console_run(void)
{
    char buffer[UART_BUFFER_SIZE];
    int used = 0;

    while (true) {
        uint8_t byte;
        const int length = uart_read_bytes(APP_UART_NUM, &byte, 1, pdMS_TO_TICKS(10));
        if (length <= 0) {
            continue;
        }

        if (byte == '\n' || byte == '\r') {
            if (used > 0) {
                buffer[used] = '\0';
                parse_serial_command(buffer);
                used = 0;
            }
        } else if (used < UART_BUFFER_SIZE - 1) {
            buffer[used++] = (char)byte;
        }
    }
}
