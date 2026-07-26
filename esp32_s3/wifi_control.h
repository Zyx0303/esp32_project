#pragma once

#include "esp_err.h"

#define WIFI_CONTROL_AP_SSID       "ESP32-Robot"
#define WIFI_CONTROL_AP_PASSWORD   "esp32robot"
#define WIFI_CONTROL_AP_IP         "192.168.4.1"

typedef esp_err_t (*wifi_motor_speed_cb_t)(int speed_percent);
typedef esp_err_t (*wifi_servo_angle_cb_t)(int angle_deg);

/**
 * Start a Wi-Fi SoftAP and HTTP control server.
 *
 * HTTP endpoints:
 *   GET /                       Browser control page
 *   GET /api/status             Current motor/servo state
 *   GET /api/motor?speed=-100   Signed motor speed (-100..100)
 *   GET /api/servo?angle=90     Servo angle (0..180)
 *   GET /api/stop               Stop both motors
 */
esp_err_t wifi_control_init(wifi_motor_speed_cb_t motor_cb,
                            wifi_servo_angle_cb_t servo_cb);
