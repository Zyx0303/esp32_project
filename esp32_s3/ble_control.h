#pragma once

#include <stdint.h>
#include "esp_err.h"

#define BLE_SERVICE_NAME "MSRR-1"

typedef void (*ble_motor_speed_cb_t)(uint8_t duty_percent);
typedef void (*ble_servo_angle_cb_t)(uint8_t angle_deg);

esp_err_t ble_control_init(ble_motor_speed_cb_t motor_cb, ble_servo_angle_cb_t servo_cb);
esp_err_t ble_control_set_motor_speed(uint8_t duty_percent);
esp_err_t ble_control_set_servo_angle(uint8_t angle_deg);
