#include "robot_config.h"

#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "nvs.h"

static const char *TAG = "robot_config";
static const char *NVS_NAMESPACE = "robot_cfg";
static const char *NVS_KEY = "v1";

void robot_config_defaults(robot_config_t *config)
{
    if (config == NULL) {
        return;
    }
    *config = (robot_config_t) {
        .version = ROBOT_CONFIG_VERSION,
        .invert_motor_a = false,
        .invert_motor_b = false,
        .motor_ramp_step_percent = 5,
        .motor_reverse_deadtime_ms = 50,
        .command_timeout_ms = 1000,
        .servo_min_angle_deg = 0,
        .servo_max_angle_deg = 180,
        .servo_center_angle_deg = 90,
        .servo_min_pulse_us = 500,
        .servo_max_pulse_us = 2500,
        .servo_slew_deg_per_second = 180,
    };
}

bool robot_config_validate(const robot_config_t *config)
{
    return config != NULL &&
           config->version == ROBOT_CONFIG_VERSION &&
           config->motor_ramp_step_percent >= 1 &&
           config->motor_ramp_step_percent <= 100 &&
           config->motor_reverse_deadtime_ms <= 2000 &&
           config->command_timeout_ms >= 100 &&
           config->command_timeout_ms <= 10000 &&
           config->servo_min_angle_deg < config->servo_max_angle_deg &&
           config->servo_max_angle_deg <= 180 &&
           config->servo_center_angle_deg >= config->servo_min_angle_deg &&
           config->servo_center_angle_deg <= config->servo_max_angle_deg &&
           config->servo_min_pulse_us >= 300 &&
           config->servo_min_pulse_us < config->servo_max_pulse_us &&
           config->servo_max_pulse_us <= 3000 &&
           config->servo_slew_deg_per_second >= 1 &&
           config->servo_slew_deg_per_second <= 1000;
}

esp_err_t robot_config_load(robot_config_t *config, bool *used_defaults)
{
    ESP_RETURN_ON_FALSE(config != NULL, ESP_ERR_INVALID_ARG, TAG, "config required");
    if (used_defaults != NULL) {
        *used_defaults = false;
    }

    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        robot_config_defaults(config);
        if (used_defaults != NULL) {
            *used_defaults = true;
        }
        return ESP_OK;
    }
    ESP_RETURN_ON_ERROR(err, TAG, "open NVS");

    size_t size = sizeof(*config);
    err = nvs_get_blob(handle, NVS_KEY, config, &size);
    nvs_close(handle);
    if (err != ESP_OK || size != sizeof(*config) || !robot_config_validate(config)) {
        ESP_LOGW(TAG, "stored configuration missing or invalid; using safe defaults");
        robot_config_defaults(config);
        if (used_defaults != NULL) {
            *used_defaults = true;
        }
        return ESP_OK;
    }
    return ESP_OK;
}

esp_err_t robot_config_save(const robot_config_t *config)
{
    ESP_RETURN_ON_FALSE(robot_config_validate(config), ESP_ERR_INVALID_ARG, TAG,
                        "invalid configuration");
    nvs_handle_t handle;
    ESP_RETURN_ON_ERROR(nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle), TAG, "open NVS");
    esp_err_t err = nvs_set_blob(handle, NVS_KEY, config, sizeof(*config));
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    return err;
}
