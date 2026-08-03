#include "wifi_control.h"

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cJSON.h"
#include "esp_check.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_netif_ip_addr.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "robot_state_machine.h"

// 私密凭据只放在被 Git 忽略的本地头文件；缺失时仍可构建救援 AP 固件。
#if __has_include("wifi_credentials.h")
#include "wifi_credentials.h"
#else
#define ROBOT_WIFI_STA_SSID     ""
#define ROBOT_WIFI_STA_PASSWORD ""
#endif

static const char *TAG = "wifi_control";

static wifi_command_submit_cb_t s_submit_cb;
static wifi_status_snapshot_cb_t s_status_cb;
static httpd_handle_t s_http_server;
static uint32_t s_sequence;

// Wi-Fi/IP 事件循环会写这些字段，HTTP 和串口任务会读；临界区只复制定长值，
// 不能在锁内打印日志、分配内存或调用网络 API。
static portMUX_TYPE s_network_lock = portMUX_INITIALIZER_UNLOCKED;
static bool s_sta_configured;
static bool s_sta_connected;
static esp_ip4_addr_t s_sta_ip;
static uint32_t s_sta_reconnect_count;
static uint16_t s_last_disconnect_reason;
static uint32_t s_reconnect_delay_ms;
static uint8_t s_consecutive_disconnects;
static esp_timer_handle_t s_reconnect_timer;

#define STA_RECONNECT_INITIAL_DELAY_MS 2000U
#define STA_RECONNECT_MAX_DELAY_MS    30000U

bool wifi_control_get_network_status(wifi_control_network_status_t *status)
{
    if (status == NULL) {
        return false;
    }
    portENTER_CRITICAL(&s_network_lock);
    status->sta_configured = s_sta_configured;
    status->sta_connected = s_sta_connected;
    status->sta_ip = s_sta_ip.addr;
    status->reconnect_count = s_sta_reconnect_count;
    status->last_disconnect_reason = s_last_disconnect_reason;
    status->reconnect_delay_ms = s_reconnect_delay_ms;
    portEXIT_CRITICAL(&s_network_lock);
    return true;
}

static const char *disconnect_reason_name(uint16_t reason)
{
    switch (reason) {
    case WIFI_REASON_AUTH_EXPIRE:
        return "auth_expired";
    case WIFI_REASON_AUTH_LEAVE:
        return "auth_left";
    case WIFI_REASON_DISASSOC_DUE_TO_INACTIVITY:
        return "inactive";
    case WIFI_REASON_ASSOC_LEAVE:
        return "association_left";
    case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT:
        return "4way_handshake_timeout";
    case WIFI_REASON_BEACON_TIMEOUT:
        return "beacon_timeout";
    case WIFI_REASON_NO_AP_FOUND:
        return "ap_not_found";
    case WIFI_REASON_AUTH_FAIL:
        return "authentication_failed";
    case WIFI_REASON_ASSOC_FAIL:
        return "association_failed";
    case WIFI_REASON_HANDSHAKE_TIMEOUT:
        return "handshake_timeout";
    case WIFI_REASON_CONNECTION_FAIL:
        return "connection_failed";
    case WIFI_REASON_NO_AP_FOUND_W_COMPATIBLE_SECURITY:
        return "security_incompatible";
    case WIFI_REASON_NO_AP_FOUND_IN_AUTHMODE_THRESHOLD:
        return "authmode_below_threshold";
    case WIFI_REASON_NO_AP_FOUND_IN_RSSI_THRESHOLD:
        return "rssi_below_threshold";
    default:
        return "other";
    }
}

static uint32_t reconnect_delay_ms(uint8_t consecutive_disconnects)
{
    uint32_t delay = STA_RECONNECT_INITIAL_DELAY_MS;
    // 2、4、8、16、30 秒封顶；给救援 AP 留出稳定广播和接入窗口。
    for (uint8_t index = 1; index < consecutive_disconnects &&
                            delay < STA_RECONNECT_MAX_DELAY_MS;
         ++index) {
        delay *= 2U;
    }
    return delay > STA_RECONNECT_MAX_DELAY_MS ? STA_RECONNECT_MAX_DELAY_MS : delay;
}

static void schedule_reconnect(void);

static void reconnect_timer_callback(void *argument)
{
    (void)argument;
    esp_err_t err = esp_wifi_connect();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "STA reconnect request failed: %s", esp_err_to_name(err));
        schedule_reconnect();
    }
}

static void schedule_reconnect(void)
{
    if (s_reconnect_timer == NULL || !s_sta_configured) {
        return;
    }
    uint8_t failures;
    portENTER_CRITICAL(&s_network_lock);
    failures = s_consecutive_disconnects;
    s_reconnect_delay_ms = reconnect_delay_ms(failures);
    uint32_t delay_ms = s_reconnect_delay_ms;
    portEXIT_CRITICAL(&s_network_lock);

    // timer 可能刚刚到期或已经停止；两种状态都统一重新装载为单次定时器。
    (void)esp_timer_stop(s_reconnect_timer);
    esp_err_t err = esp_timer_start_once(s_reconnect_timer, (uint64_t)delay_ms * 1000U);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "schedule STA reconnect failed: %s", esp_err_to_name(err));
    }
}

static void network_event_handler(void *argument, esp_event_base_t event_base,
                                  int32_t event_id, void *event_data)
{
    (void)argument;
    // STA 启动事件只发起连接，不等待 DHCP，避免阻塞 ESP-IDF 系统事件循环。
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_err_t err = esp_wifi_connect();
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "initial STA connect failed: %s", esp_err_to_name(err));
        }
        return;
    }
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        const wifi_event_sta_disconnected_t *event = event_data;
        const uint16_t reason = event != NULL ? event->reason : 0;
        // 先清除旧 IP，再安排延迟重连。不能在事件回调中立即开启下一轮全信道扫描，
        // 否则目标 AP 长期离线时会挤占救援 SoftAP 的信标发送时间。
        portENTER_CRITICAL(&s_network_lock);
        s_sta_connected = false;
        s_sta_ip.addr = 0;
        s_sta_reconnect_count++;
        s_last_disconnect_reason = reason;
        if (s_consecutive_disconnects < UINT8_MAX) {
            s_consecutive_disconnects++;
        }
        portEXIT_CRITICAL(&s_network_lock);
        schedule_reconnect();
        ESP_LOGW(TAG, "STA disconnected: reason=%u (%s); retry %s in %" PRIu32 " ms",
                 reason, disconnect_reason_name(reason), ROBOT_WIFI_STA_SSID,
                 s_reconnect_delay_ms);
        return;
    }
    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        // 只有收到 DHCP GOT_IP 才把 STA 标为可访问；“已关联”不等于已有 IP。
        const ip_event_got_ip_t *event = event_data;
        portENTER_CRITICAL(&s_network_lock);
        s_sta_connected = true;
        s_sta_ip = event->ip_info.ip;
        s_consecutive_disconnects = 0;
        s_reconnect_delay_ms = 0;
        portEXIT_CRITICAL(&s_network_lock);
        if (s_reconnect_timer != NULL) {
            (void)esp_timer_stop(s_reconnect_timer);
        }
        ESP_LOGI(TAG, "STA connected: SSID=%s IP=" IPSTR " URL=http://" IPSTR,
                 ROBOT_WIFI_STA_SSID, IP2STR(&event->ip_info.ip),
                 IP2STR(&event->ip_info.ip));
    }
}

static const char CONTROL_PAGE[] =
    "<!doctype html><html lang='zh-CN'><head><meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>ESP32 Robot</title><style>body{font-family:system-ui;max-width:600px;margin:auto;padding:24px}"
    "button{font-size:18px;padding:12px;margin:5px}input{width:100%}.danger{background:#d33;color:white}"
    "pre{background:#eee;padding:12px;overflow:auto}</style></head><body>"
    "<h1>ESP32-S3 Robot</h1><p>上电默认锁定。先解锁，控制中断后固件会自动急停。</p>"
    "<button onclick=post('/api/v1/arm')>ARM 解锁</button>"
    "<button onclick=post('/api/v1/disarm')>DISARM</button>"
    "<button class=danger onclick=post('/api/v1/estop')>急停</button>"
    "<button onclick=post('/api/v1/fault/clear')>清除故障</button>"
    "<h2>行驶</h2><label>油门 <span id=tv>0</span></label><input id=t type=range min=-100 max=100 value=0>"
    "<label>转向 <span id=sv>0</span></label><input id=s type=range min=-100 max=100 value=0>"
    "<button onclick=drive()>发送</button><button class=danger onclick=drive(0,0)>停车</button>"
    "<h2>舵机</h2><input id=a type=range min=0 max=180 value=90>"
    "<button onclick=servo()>发送角度</button><h2>状态</h2><pre id=status></pre>"
    "<script>const post=(u,b)=>fetch(u,{method:'POST',headers:{'Content-Type':'application/json'},body:b?JSON.stringify(b):''});"
    "t.oninput=()=>tv.textContent=t.value;s.oninput=()=>sv.textContent=s.value;"
    "function drive(x,y){post('/api/v1/drive',{throttle:x??+t.value,steering:y??+s.value})}"
    "function servo(){post('/api/v1/servo',{angle:+a.value})}"
    "setInterval(()=>fetch('/api/v1/status',{cache:'no-store'}).then(r=>r.json()).then(j=>status.textContent=JSON.stringify(j,null,2)).catch(()=>{}),500);"
    "</script></body></html>";

static void common_headers(httpd_req_t *req)
{
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
}

static esp_err_t send_json(httpd_req_t *req, const char *json)
{
    common_headers(req);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, json);
}

static esp_err_t send_error(httpd_req_t *req, const char *status, const char *code)
{
    char response[192];
    robot_status_t snapshot = {0};
    const char *state = (s_status_cb != NULL && s_status_cb(&snapshot))
                            ? robot_state_name(snapshot.state)
                            : "UNKNOWN";
    httpd_resp_set_status(req, status);
    snprintf(response, sizeof(response),
             "{\"ok\":false,\"sequence\":null,\"state\":\"%s\",\"error\":\"%s\"}",
             state, code);
    return send_json(req, response);
}

static esp_err_t root_handler(httpd_req_t *req)
{
    common_headers(req);
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, CONTROL_PAGE, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t status_handler(httpd_req_t *req)
{
    robot_status_t status;
    if (s_status_cb == NULL || !s_status_cb(&status)) {
        return send_error(req, "503 Service Unavailable", "status_unavailable");
    }
    wifi_sta_list_t stations = {0};
    int clients = esp_wifi_ap_get_sta_list(&stations) == ESP_OK ? stations.num : 0;
    // 用快照组装 JSON，不在格式化和发送期间持有跨任务锁。
    wifi_control_network_status_t network = {0};
    wifi_control_get_network_status(&network);
    esp_ip4_addr_t sta_ip = {.addr = network.sta_ip};
    char ip_text[16];
    snprintf(ip_text, sizeof(ip_text), IPSTR, IP2STR(&sta_ip));
    char response[1200];
    int length = snprintf(response, sizeof(response),
             "{\"ok\":true,\"api_version\":%d,\"state\":\"%s\",\"armed\":%s,"
             "\"estop\":%s,\"faults\":%" PRIu32 ",\"motor_a\":{\"target\":%d,\"output\":%d},"
             "\"motor_b\":{\"target\":%d,\"output\":%d},\"servo\":{\"target\":%d,\"output\":%d,\"enabled\":%s},"
             "\"power\":%s,\"drv8833_fault\":%s,"
             "\"imu\":{\"valid\":%s,\"accel\":[%d,%d,%d],\"gyro\":[%d,%d,%d],\"temp_raw\":%d,"
             "\"samples\":%" PRIu32 ",\"errors\":%" PRIu32 ",\"last_update_us\":%" PRId64 "},"
             "\"network\":{\"mode\":\"%s\",\"sta_configured\":%s,\"sta_connected\":%s,"
             "\"sta_ip\":\"%s\",\"ap_clients\":%d,\"reconnects\":%" PRIu32
             ",\"last_disconnect_reason\":%u,\"reconnect_delay_ms\":%" PRIu32 "},"
             "\"system\":{\"uptime_ms\":%" PRIu32 ",\"free_heap\":%" PRIu32 ",\"wifi_clients\":%d},"
             "\"counters\":{\"received\":%" PRIu32 ",\"rejected\":%" PRIu32
             ",\"expired\":%" PRIu32 ",\"watchdog_stops\":%" PRIu32
             ",\"queue_overflow\":%" PRIu32 "}}",
             ROBOT_HTTP_API_VERSION, robot_state_name(status.state),
             status.armed ? "true" : "false",
             status.estop_latched ? "true" : "false", status.faults,
             status.motor_a_target, status.motor_a_output, status.motor_b_target,
             status.motor_b_output, status.servo_target_deg, status.servo_output_deg,
             status.servo_enabled ? "true" : "false",
             status.power_asserted ? "true" : "false",
             status.drv8833_fault_active ? "true" : "false",
             status.imu_valid ? "true" : "false", status.imu_ax, status.imu_ay,
             status.imu_az, status.imu_gx, status.imu_gy, status.imu_gz,
             status.imu_temp_raw, status.imu_sample_count, status.imu_error_count,
             status.imu_last_update_us, s_sta_configured ? "APSTA" : "AP",
             s_sta_configured ? "true" : "false",
             network.sta_connected ? "true" : "false",
             ip_text, clients, network.reconnect_count,
             network.last_disconnect_reason, network.reconnect_delay_ms,
             status.uptime_ms, status.free_heap_bytes, clients,
             status.commands_received, status.commands_rejected, status.commands_expired,
             status.watchdog_stops, status.queue_overflow_count);
    if (length < 0 || length >= (int)sizeof(response)) {
        return send_error(req, "500 Internal Server Error", "status_response_too_large");
    }
    return send_json(req, response);
}

static esp_err_t device_handler(httpd_req_t *req)
{
    char response[192];
    snprintf(response, sizeof(response),
             "{\"ok\":true,\"api_version\":%d,\"target\":\"esp32s3\","
             "\"build_date\":\"%s\",\"build_time\":\"%s\"}",
             ROBOT_HTTP_API_VERSION, __DATE__, __TIME__);
    return send_json(req, response);
}

static esp_err_t read_body_json(httpd_req_t *req, cJSON **root)
{
    if (req->content_len <= 0 || req->content_len > 256) {
        return ESP_ERR_INVALID_SIZE;
    }
    char body[257];
    size_t received = 0;
    while (received < req->content_len) {
        int length = httpd_req_recv(req, body + received, req->content_len - received);
        if (length <= 0) {
            return ESP_FAIL;
        }
        received += (size_t)length;
    }
    body[received] = '\0';
    *root = cJSON_Parse(body);
    return *root != NULL ? ESP_OK : ESP_ERR_INVALID_ARG;
}

static bool json_int(const cJSON *root, const char *name, int *value)
{
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(root, name);
    if (!cJSON_IsNumber(item) || item->valuedouble != (double)item->valueint) {
        return false;
    }
    *value = item->valueint;
    return true;
}

static esp_err_t submit(httpd_req_t *req, robot_command_t *command)
{
    command->source = ROBOT_COMMAND_SOURCE_WIFI;
    command->received_at_us = esp_timer_get_time();
    command->sequence = ++s_sequence;
    if (s_submit_cb == NULL || s_submit_cb(command) != ESP_OK) {
        return send_error(req, "503 Service Unavailable", "command_queue_unavailable");
    }
    robot_status_t status = {0};
    const char *state = (s_status_cb != NULL && s_status_cb(&status))
                            ? robot_state_name(status.state)
                            : "UNKNOWN";
    char response[144];
    snprintf(response, sizeof(response),
             "{\"ok\":true,\"accepted\":true,\"sequence\":%" PRIu32
             ",\"state\":\"%s\",\"error\":null}",
             command->sequence, state);
    return send_json(req, response);
}

static esp_err_t simple_command_handler(httpd_req_t *req)
{
    robot_command_t command = {.type = (robot_command_type_t)(intptr_t)req->user_ctx};
    return submit(req, &command);
}

static esp_err_t drive_handler(httpd_req_t *req)
{
    cJSON *root = NULL;
    if (read_body_json(req, &root) != ESP_OK) {
        return send_error(req, "400 Bad Request", "invalid_json");
    }
    int throttle, steering;
    bool valid = json_int(root, "throttle", &throttle) &&
                 json_int(root, "steering", &steering) &&
                 throttle >= -100 && throttle <= 100 && steering >= -100 && steering <= 100;
    cJSON_Delete(root);
    if (!valid) {
        return send_error(req, "400 Bad Request", "drive_range");
    }
    robot_command_t command = {.type = ROBOT_CMD_DRIVE};
    command.value.drive.throttle = throttle;
    command.value.drive.steering = steering;
    return submit(req, &command);
}

static esp_err_t motors_handler(httpd_req_t *req)
{
    cJSON *root = NULL;
    if (read_body_json(req, &root) != ESP_OK) {
        return send_error(req, "400 Bad Request", "invalid_json");
    }
    int motor_a, motor_b;
    bool valid = json_int(root, "motor_a", &motor_a) && json_int(root, "motor_b", &motor_b) &&
                 motor_a >= -100 && motor_a <= 100 && motor_b >= -100 && motor_b <= 100;
    cJSON_Delete(root);
    if (!valid) {
        return send_error(req, "400 Bad Request", "motor_range");
    }
    robot_command_t command = {.type = ROBOT_CMD_MOTOR_DIRECT};
    command.value.motors.motor_a = motor_a;
    command.value.motors.motor_b = motor_b;
    return submit(req, &command);
}

static esp_err_t servo_handler(httpd_req_t *req)
{
    cJSON *root = NULL;
    if (read_body_json(req, &root) != ESP_OK) {
        return send_error(req, "400 Bad Request", "invalid_json");
    }
    int angle;
    bool valid = json_int(root, "angle", &angle) && angle >= 0 && angle <= 180;
    cJSON_Delete(root);
    if (!valid) {
        return send_error(req, "400 Bad Request", "servo_range");
    }
    robot_command_t command = {.type = ROBOT_CMD_SERVO};
    command.value.servo.angle_deg = angle;
    return submit(req, &command);
}

static esp_err_t power_handler(httpd_req_t *req)
{
    cJSON *root = NULL;
    if (read_body_json(req, &root) != ESP_OK) {
        return send_error(req, "400 Bad Request", "invalid_json");
    }
    const cJSON *asserted = cJSON_GetObjectItemCaseSensitive(root, "asserted");
    bool valid = cJSON_IsBool(asserted);
    bool value = cJSON_IsTrue(asserted);
    cJSON_Delete(root);
    if (!valid) {
        return send_error(req, "400 Bad Request", "power_boolean_required");
    }
    robot_command_t command = {.type = ROBOT_CMD_POWER_CONTROL};
    command.value.power.asserted = value;
    return submit(req, &command);
}

static esp_err_t legacy_int_query(httpd_req_t *req, const char *key, int *value)
{
    char query[64], raw[16], *end = NULL;
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
        httpd_query_key_value(query, key, raw, sizeof(raw)) != ESP_OK) {
        return ESP_ERR_INVALID_ARG;
    }
    long parsed = strtol(raw, &end, 10);
    if (end == raw || *end != '\0') {
        return ESP_ERR_INVALID_ARG;
    }
    *value = (int)parsed;
    return ESP_OK;
}

static esp_err_t legacy_motor_handler(httpd_req_t *req)
{
    int speed;
    if (legacy_int_query(req, "speed", &speed) != ESP_OK || speed < -100 || speed > 100) {
        return send_error(req, "400 Bad Request", "speed_range");
    }
    robot_command_t command = {.type = ROBOT_CMD_DRIVE};
    command.value.drive.throttle = speed;
    command.value.drive.steering = 0;
    return submit(req, &command);
}

static esp_err_t legacy_servo_handler(httpd_req_t *req)
{
    int angle;
    if (legacy_int_query(req, "angle", &angle) != ESP_OK || angle < 0 || angle > 180) {
        return send_error(req, "400 Bad Request", "servo_range");
    }
    robot_command_t command = {.type = ROBOT_CMD_SERVO};
    command.value.servo.angle_deg = angle;
    return submit(req, &command);
}

static esp_err_t legacy_power_handler(httpd_req_t *req)
{
    int on;
    if (legacy_int_query(req, "on", &on) != ESP_OK || (on != 0 && on != 1)) {
        return send_error(req, "400 Bad Request", "on_boolean_required");
    }
    robot_command_t command = {.type = ROBOT_CMD_POWER_CONTROL};
    command.value.power.asserted = on == 1;
    return submit(req, &command);
}

static esp_err_t start_http_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;
    config.max_uri_handlers = 20;
    ESP_RETURN_ON_ERROR(httpd_start(&s_http_server, &config), TAG, "start HTTP server");

    const httpd_uri_t handlers[] = {
        {.uri = "/", .method = HTTP_GET, .handler = root_handler},
        {.uri = "/api/v1/status", .method = HTTP_GET, .handler = status_handler},
        {.uri = "/api/v1/device", .method = HTTP_GET, .handler = device_handler},
        {.uri = "/api/v1/arm", .method = HTTP_POST, .handler = simple_command_handler,
         .user_ctx = (void *)(intptr_t)ROBOT_CMD_ARM},
        {.uri = "/api/v1/disarm", .method = HTTP_POST, .handler = simple_command_handler,
         .user_ctx = (void *)(intptr_t)ROBOT_CMD_DISARM},
        {.uri = "/api/v1/estop", .method = HTTP_POST, .handler = simple_command_handler,
         .user_ctx = (void *)(intptr_t)ROBOT_CMD_ESTOP},
        {.uri = "/api/v1/fault/clear", .method = HTTP_POST, .handler = simple_command_handler,
         .user_ctx = (void *)(intptr_t)ROBOT_CMD_CLEAR_FAULT},
        {.uri = "/api/v1/drive", .method = HTTP_POST, .handler = drive_handler},
        {.uri = "/api/v1/motors", .method = HTTP_POST, .handler = motors_handler},
        {.uri = "/api/v1/servo", .method = HTTP_POST, .handler = servo_handler},
        {.uri = "/api/v1/power", .method = HTTP_POST, .handler = power_handler},
        {.uri = "/api/status", .method = HTTP_GET, .handler = status_handler},
        {.uri = "/api/motor", .method = HTTP_GET, .handler = legacy_motor_handler},
        {.uri = "/api/servo", .method = HTTP_GET, .handler = legacy_servo_handler},
        {.uri = "/api/power", .method = HTTP_GET, .handler = legacy_power_handler},
        {.uri = "/api/stop", .method = HTTP_GET, .handler = simple_command_handler,
         .user_ctx = (void *)(intptr_t)ROBOT_CMD_DISARM},
    };
    for (size_t i = 0; i < sizeof(handlers) / sizeof(handlers[0]); ++i) {
        ESP_RETURN_ON_ERROR(httpd_register_uri_handler(s_http_server, &handlers[i]),
                            TAG, "register HTTP handler");
    }
    return ESP_OK;
}

esp_err_t wifi_control_init(wifi_command_submit_cb_t submit_cb,
                            wifi_status_snapshot_cb_t status_cb)
{
    ESP_RETURN_ON_FALSE(submit_cb != NULL && status_cb != NULL, ESP_ERR_INVALID_ARG,
                        TAG, "callbacks required");
    s_submit_cb = submit_cb;
    s_status_cb = status_cb;

    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "initialize TCP/IP stack");
    esp_err_t event_err = esp_event_loop_create_default();
    if (event_err != ESP_OK && event_err != ESP_ERR_INVALID_STATE) {
        return event_err;
    }
    // 始终创建救援 AP；只有本地凭据非空时才额外创建 STA。
    // 因此路由器故障不会让设备失去最后一条管理链路。
    s_sta_configured = strlen(ROBOT_WIFI_STA_SSID) > 0;
    ESP_RETURN_ON_FALSE(esp_netif_create_default_wifi_ap() != NULL, ESP_FAIL, TAG,
                        "create AP interface");
    if (s_sta_configured) {
        ESP_RETURN_ON_FALSE(esp_netif_create_default_wifi_sta() != NULL, ESP_FAIL, TAG,
                            "create STA interface");
    }
    wifi_init_config_t init_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&init_config), TAG, "initialize Wi-Fi");
    if (s_sta_configured) {
        const esp_timer_create_args_t reconnect_timer_config = {
            .callback = reconnect_timer_callback,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "wifi_reconnect",
        };
        ESP_RETURN_ON_ERROR(esp_timer_create(&reconnect_timer_config, &s_reconnect_timer),
                            TAG, "create reconnect timer");
    }
    // 凭据来自编译期本地文件，不写入 Wi-Fi NVS，避免旧配置覆盖当前配置。
    ESP_RETURN_ON_ERROR(esp_wifi_set_storage(WIFI_STORAGE_RAM), TAG, "set Wi-Fi storage");
    ESP_RETURN_ON_ERROR(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                   network_event_handler, NULL),
                        TAG, "register Wi-Fi events");
    ESP_RETURN_ON_ERROR(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                   network_event_handler, NULL),
                        TAG, "register IP events");
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(s_sta_configured ? WIFI_MODE_APSTA : WIFI_MODE_AP),
                        TAG, "set Wi-Fi mode");

    wifi_config_t ap_config = {0};
    snprintf((char *)ap_config.ap.ssid, sizeof(ap_config.ap.ssid), "%s", WIFI_CONTROL_AP_SSID);
    snprintf((char *)ap_config.ap.password, sizeof(ap_config.ap.password), "%s",
             WIFI_CONTROL_AP_PASSWORD);
    ap_config.ap.ssid_len = strlen(WIFI_CONTROL_AP_SSID);
    // APSTA 连接成功后，ESP-IDF 会让救援 AP 跟随路由器所在信道。
    ap_config.ap.channel = 1;
    ap_config.ap.max_connection = 4;
    ap_config.ap.authmode = WIFI_AUTH_WPA2_PSK;
    ap_config.ap.pmf_cfg.capable = true;
    ap_config.ap.pmf_cfg.required = false;
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_AP, &ap_config), TAG, "configure AP");

    if (s_sta_configured) {
        wifi_config_t sta_config = {0};
        snprintf((char *)sta_config.sta.ssid, sizeof(sta_config.sta.ssid), "%s",
                 ROBOT_WIFI_STA_SSID);
        snprintf((char *)sta_config.sta.password, sizeof(sta_config.sta.password), "%s",
                 ROBOT_WIFI_STA_PASSWORD);
        sta_config.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
        sta_config.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
        sta_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
        sta_config.sta.pmf_cfg.capable = true;
        sta_config.sta.pmf_cfg.required = false;
        ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_STA, &sta_config), TAG,
                            "configure STA");
    }

    // esp_wifi_start() 会触发 STA_START；连接和 DHCP 均由事件回调推进。
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "start Wi-Fi");
    ESP_RETURN_ON_ERROR(start_http_server(), TAG, "start control server");
    ESP_LOGI(TAG, "rescue AP ready: SSID=%s URL=http://%s", WIFI_CONTROL_AP_SSID,
             WIFI_CONTROL_AP_IP);
    if (s_sta_configured) {
        ESP_LOGI(TAG, "connecting STA to %s; wait for the DHCP IP log", ROBOT_WIFI_STA_SSID);
    } else {
        ESP_LOGW(TAG, "STA credentials absent; running rescue AP only");
    }
    return ESP_OK;
}
