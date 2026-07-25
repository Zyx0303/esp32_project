#include "ble_control.h"

#include <string.h>
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_gatt_common_api.h"
#include "esp_timer.h"

static const char *TAG = "ble_control";

static ble_motor_speed_cb_t s_motor_cb;
static ble_servo_angle_cb_t s_servo_cb;
static uint8_t s_current_speed;
static uint8_t s_current_angle;

#define DEVICE_NAME "ESP32-Robot"
#define APP_ID 0
#define SERVICE_UUID 0xFF00
#define CHAR_UUID_MOTOR 0xFF01
#define CHAR_UUID_SERVO 0xFF02

#define ADV_CONFIG_FLAG      (1 << 0)
#define SCAN_RSP_CONFIG_FLAG (1 << 1)

static uint8_t s_adv_config_done = 0;
static uint16_t s_motor_handle = 0;
static uint16_t s_servo_handle = 0;
static uint16_t s_service_handle = 0;
static uint16_t s_conn_id = 0;
static uint8_t s_motor_value = 0;
static uint8_t s_servo_value = 90;

static esp_bd_addr_t s_local_addr;
static uint8_t s_local_addr_type;

static esp_attr_value_t s_motor_val = {
    .attr_max_len = 1,
    .attr_len = 1,
    .attr_value = &s_motor_value,
};

static esp_attr_value_t s_servo_val = {
    .attr_max_len = 1,
    .attr_len = 1,
    .attr_value = &s_servo_value,
};

// Advertising parameters - match beacon example
static esp_ble_adv_params_t s_adv_params = {
    .adv_int_min = 0x20,  // 20ms
    .adv_int_max = 0x20,  // 20ms
    .adv_type = ADV_TYPE_IND,  // Connectable undirected advertising
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

// Advertising data - includes name
static uint8_t s_adv_raw_data[] = {
    0x02, ESP_BLE_AD_TYPE_FLAG, 0x06,  // Flags
    0x0B, ESP_BLE_AD_TYPE_NAME_CMPL, 'E', 'S', 'P', '3', '2', '-', 'R', 'o', 'b', 'o', 't',  // Name
};

// Scan response data
static uint8_t s_scan_rsp_raw_data[] = {
    0x11, ESP_BLE_AD_TYPE_NAME_CMPL, 'E', 'S', 'P', '3', '2', '-', 'R', 'o', 'b', 'o', 't', '-', 'S', 'E', 'R', 'V', 'O',
};

static void start_advertising(void);

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
        ESP_LOGI(TAG, "Advertising raw data set, status=%d", param->adv_data_raw_cmpl.status);
        s_adv_config_done &= (~ADV_CONFIG_FLAG);
        if (s_adv_config_done == 0) {
            start_advertising();
        }
        break;

    case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
        ESP_LOGI(TAG, "Scan response raw data set, status=%d", param->scan_rsp_data_raw_cmpl.status);
        s_adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
        if (s_adv_config_done == 0) {
            start_advertising();
        }
        break;

    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "Advertising started successfully!");
        } else {
            ESP_LOGE(TAG, "Advertising start failed, status=%d", param->adv_start_cmpl.status);
        }
        break;

    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        ESP_LOGI(TAG, "Advertising stopped");
        break;

    default:
        break;
    }
}

static void start_advertising(void)
{
    esp_err_t ret = esp_ble_gap_start_advertising(&s_adv_params);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start advertising: %s", esp_err_to_name(ret));
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                                 esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
    case ESP_GATTS_REG_EVT:
        ESP_LOGI(TAG, "GATTS_REG_EVT, status=%d", param->reg.status);

        esp_ble_gap_set_device_name(DEVICE_NAME);

        // Get local address for scan response
        esp_err_t err = esp_ble_gap_get_local_used_addr(s_local_addr, &s_local_addr_type);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Get local addr failed: %s", esp_err_to_name(err));
        }

        // Configure advertising data
        s_adv_config_done |= ADV_CONFIG_FLAG;
        s_adv_config_done |= SCAN_RSP_CONFIG_FLAG;

        err = esp_ble_gap_config_adv_data_raw(s_adv_raw_data, sizeof(s_adv_raw_data));
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Config adv data failed: %s", esp_err_to_name(err));
            s_adv_config_done &= (~ADV_CONFIG_FLAG);
        }

        // Update scan response with local address
        s_scan_rsp_raw_data[2] = s_local_addr[5];
        s_scan_rsp_raw_data[3] = s_local_addr[4];
        s_scan_rsp_raw_data[4] = s_local_addr[3];
        s_scan_rsp_raw_data[5] = s_local_addr[2];
        s_scan_rsp_raw_data[6] = s_local_addr[1];
        s_scan_rsp_raw_data[7] = s_local_addr[0];

        err = esp_ble_gap_config_scan_rsp_data_raw(s_scan_rsp_raw_data, sizeof(s_scan_rsp_raw_data));
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Config scan rsp failed: %s", esp_err_to_name(err));
            s_adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
        }

        // Create GATT service
        {
            esp_gatt_srvc_id_t svc_id = {
                .is_primary = true,
                .id.inst_id = 0x00,
                .id.uuid.len = ESP_UUID_LEN_16,
                .id.uuid.uuid.uuid16 = SERVICE_UUID,
            };
            esp_ble_gatts_create_service(gatts_if, &svc_id, 4);
        }
        break;

    case ESP_GATTS_CREATE_EVT:
        ESP_LOGI(TAG, "GATTS_CREATE_EVT, service_handle=%d", param->create.service_handle);
        s_service_handle = param->create.service_handle;
        esp_ble_gatts_start_service(s_service_handle);

        {
            esp_bt_uuid_t uuid = {
                .len = ESP_UUID_LEN_16,
                .uuid.uuid16 = CHAR_UUID_MOTOR,
            };
            esp_ble_gatts_add_char(s_service_handle, &uuid,
                                   ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                   ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE,
                                   &s_motor_val, NULL);
        }
        break;

    case ESP_GATTS_ADD_CHAR_EVT:
        ESP_LOGI(TAG, "ADD_CHAR_EVT, attr_handle=%d", param->add_char.attr_handle);

        if (s_motor_handle == 0) {
            s_motor_handle = param->add_char.attr_handle;
            ESP_LOGI(TAG, "Motor handle: %d", s_motor_handle);

            esp_bt_uuid_t uuid = {
                .len = ESP_UUID_LEN_16,
                .uuid.uuid16 = CHAR_UUID_SERVO,
            };
            esp_ble_gatts_add_char(s_service_handle, &uuid,
                                   ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                   ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE,
                                   &s_servo_val, NULL);
        } else if (s_servo_handle == 0) {
            s_servo_handle = param->add_char.attr_handle;
            ESP_LOGI(TAG, "Servo handle: %d", s_servo_handle);
            ESP_LOGI(TAG, "All services ready!");
        }
        break;

    case ESP_GATTS_READ_EVT: {
        uint16_t handle = param->read.handle;
        ESP_LOGI(TAG, "READ_EVT, handle=%d", handle);

        esp_gatt_rsp_t rsp = {0};
        rsp.attr_value.handle = handle;

        if (handle == s_motor_handle) {
            rsp.attr_value.len = 1;
            rsp.attr_value.value[0] = s_motor_value;
            ESP_LOGI(TAG, "Read motor: %d%%", s_motor_value);
        } else if (handle == s_servo_handle) {
            rsp.attr_value.len = 1;
            rsp.attr_value.value[0] = s_servo_value;
            ESP_LOGI(TAG, "Read servo: %d deg", s_servo_value);
        }

        esp_ble_gatts_send_response(gatts_if, param->read.conn_id,
                                    param->read.trans_id, ESP_GATT_OK, &rsp);
        break;
    }

    case ESP_GATTS_WRITE_EVT: {
        uint16_t handle = param->write.handle;
        uint8_t len = param->write.len;
        uint8_t *value = param->write.value;

        ESP_LOGI(TAG, "WRITE_EVT, handle=%d, len=%d", handle, len);

        if (len > 0 && value != NULL) {
            if (handle == s_motor_handle) {
                uint8_t speed = value[0] > 100 ? 100 : value[0];
                s_motor_value = speed;
                s_current_speed = speed;
                if (s_motor_cb) {
                    s_motor_cb(s_current_speed);
                }
                ESP_LOGI(TAG, "BLE Motor -> %d%%", s_current_speed);
            } else if (handle == s_servo_handle) {
                uint8_t angle = value[0] > 180 ? 180 : value[0];
                s_servo_value = angle;
                s_current_angle = angle;
                if (s_servo_cb) {
                    s_servo_cb(s_current_angle);
                }
                ESP_LOGI(TAG, "BLE Servo -> %d deg", s_current_angle);
            }
        }

        esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
                                    param->write.trans_id, ESP_GATT_OK, NULL);
        break;
    }

    case ESP_GATTS_CONNECT_EVT:
        s_conn_id = param->connect.conn_id;
        ESP_LOGI(TAG, "CONNECT_EVT, conn_id=%d", s_conn_id);
        break;

    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(TAG, "DISCONNECT_EVT, reconnecting...");
        start_advertising();
        break;

    default:
        break;
    }
}

esp_err_t ble_control_init(ble_motor_speed_cb_t motor_cb, ble_servo_angle_cb_t servo_cb)
{
    s_motor_cb = motor_cb;
    s_servo_cb = servo_cb;
    s_current_speed = 0;
    s_current_angle = 90;

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatts_event_handler));

    // Register the application to receive GATT events
    esp_err_t err = esp_ble_gatts_app_register(APP_ID);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "GATTS app register failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "BLE initialized");
    ESP_LOGI(TAG, "Device name: %s", DEVICE_NAME);
    ESP_LOGI(TAG, "Service UUID: 0x%04X", SERVICE_UUID);
    ESP_LOGI(TAG, "Motor Char: 0x%04X (0-100)", CHAR_UUID_MOTOR);
    ESP_LOGI(TAG, "Servo Char: 0x%04X (0-180)", CHAR_UUID_SERVO);

    return ESP_OK;
}

esp_err_t ble_control_set_motor_speed(uint8_t duty_percent)
{
    if (duty_percent > 100) duty_percent = 100;
    s_current_speed = duty_percent;
    if (s_motor_cb) s_motor_cb(s_current_speed);
    return ESP_OK;
}

esp_err_t ble_control_set_servo_angle(uint8_t angle_deg)
{
    if (angle_deg > 180) angle_deg = 180;
    s_current_angle = angle_deg;
    if (s_servo_cb) s_servo_cb(s_current_angle);
    return ESP_OK;
}
