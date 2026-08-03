#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "robot_types.h"

#define WIFI_CONTROL_AP_SSID       "ESP32-Robot"
#define WIFI_CONTROL_AP_PASSWORD   "esp32robot"
#define WIFI_CONTROL_AP_IP         "192.168.4.1"
#define ROBOT_HTTP_API_VERSION     1

typedef esp_err_t (*wifi_command_submit_cb_t)(const robot_command_t *command);
typedef bool (*wifi_status_snapshot_cb_t)(robot_status_t *status);

typedef struct {
    bool sta_configured;
    bool sta_connected;
    uint32_t sta_ip;
    uint32_t reconnect_count;
    uint16_t last_disconnect_reason;
    uint32_t reconnect_delay_ms;
} wifi_control_network_status_t;

/** Start AP+STA networking and the versioned HTTP control API.
 *
 * If local STA credentials are absent, the rescue SoftAP still starts.
 * Network handlers only validate and enqueue commands; they never drive hardware.
 */
esp_err_t wifi_control_init(wifi_command_submit_cb_t submit_cb,
                            wifi_status_snapshot_cb_t status_cb);

/** Copy the current network state for serial diagnostics. */
bool wifi_control_get_network_status(wifi_control_network_status_t *status);
