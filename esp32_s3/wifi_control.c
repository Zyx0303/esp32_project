#include "wifi_control.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_check.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"

#define MOTOR_WATCHDOG_TIMEOUT_US 1000000

static const char *TAG = "wifi_control";

static wifi_motor_speed_cb_t s_motor_cb;
static wifi_servo_angle_cb_t s_servo_cb;
static httpd_handle_t s_http_server;
static esp_timer_handle_t s_motor_watchdog;
static int s_motor_speed;
static int s_servo_angle = 90;

static const char CONTROL_PAGE[] =
    "<!doctype html><html lang='zh-CN'><head>"
    "<meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>ESP32 Robot</title><style>"
    "body{font-family:system-ui;background:#111827;color:#f9fafb;max-width:560px;margin:auto;padding:24px}"
    "section{background:#1f2937;border-radius:16px;padding:20px;margin:16px 0}"
    "button{font-size:20px;padding:18px;margin:6px;border:0;border-radius:12px;min-width:120px}"
    ".go{background:#22c55e}.back{background:#f59e0b}.stop{background:#ef4444;color:white}"
    "input{width:100%}.value{font-size:24px;font-weight:700}small{color:#9ca3af}"
    "</style></head><body><h1>ESP32 Robot</h1>"
    "<small>按住前进/后退，松开即停止；网络中断 1 秒后也会自动停止。</small>"
    "<section><h2>电机</h2><label>速度 <span id='speedV' class='value'>60%</span></label>"
    "<input id='speed' type='range' min='0' max='100' value='60'>"
    "<div><button class='back' data-dir='-1'>后退</button>"
    "<button class='stop' id='stop'>停止</button>"
    "<button class='go' data-dir='1'>前进</button></div></section>"
    "<section><h2>舵机</h2><label>角度 <span id='angleV' class='value'>90°</span></label>"
    "<input id='angle' type='range' min='0' max='180' value='90'></section>"
    "<script>"
    "const speed=document.querySelector('#speed'),angle=document.querySelector('#angle'),"
    "speedV=document.querySelector('#speedV'),angleV=document.querySelector('#angleV');"
    "let driveValue=0,heartbeat=null;"
    "const api=p=>fetch(p,{cache:'no-store'}).catch(()=>{});"
    "speed.oninput=()=>speedV.textContent=speed.value+'%';"
    "function drive(dir){driveValue=dir*Number(speed.value);api('/api/motor?speed='+driveValue);"
    "clearInterval(heartbeat);if(dir)heartbeat=setInterval(()=>api('/api/motor?speed='+driveValue),300);}"
    "document.querySelectorAll('[data-dir]').forEach(b=>{"
    "b.onpointerdown=e=>{e.preventDefault();drive(Number(b.dataset.dir));};"
    "b.onpointerup=b.onpointercancel=b.onpointerleave=()=>drive(0);});"
    "document.querySelector('#stop').onclick=()=>drive(0);"
    "angle.oninput=()=>{angleV.textContent=angle.value+'°';api('/api/servo?angle='+angle.value);};"
    "window.onbeforeunload=()=>api('/api/stop');"
    "</script></body></html>";

static void add_common_headers(httpd_req_t *req)
{
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
}

static esp_err_t send_json(httpd_req_t *req, const char *json)
{
    add_common_headers(req);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, json);
}

static esp_err_t send_service_unavailable(httpd_req_t *req, const char *message)
{
    add_common_headers(req);
    httpd_resp_set_status(req, "503 Service Unavailable");
    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    return httpd_resp_sendstr(req, message);
}

static esp_err_t read_int_query(httpd_req_t *req, const char *key, int *value)
{
    char query[64];
    char raw_value[16];

    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
        httpd_query_key_value(query, key, raw_value, sizeof(raw_value)) != ESP_OK) {
        return ESP_ERR_INVALID_ARG;
    }

    char *end = NULL;
    long parsed = strtol(raw_value, &end, 10);
    if (end == raw_value || *end != '\0') {
        return ESP_ERR_INVALID_ARG;
    }

    *value = (int)parsed;
    return ESP_OK;
}

static void stop_motor_watchdog(void)
{
    if (s_motor_watchdog != NULL) {
        esp_err_t err = esp_timer_stop(s_motor_watchdog);
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
            ESP_LOGW(TAG, "Unable to stop motor watchdog: %s", esp_err_to_name(err));
        }
    }
}

static esp_err_t apply_motor_speed(int speed_percent)
{
    if (speed_percent < -100 || speed_percent > 100) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_RETURN_ON_FALSE(s_motor_cb != NULL, ESP_ERR_INVALID_STATE, TAG, "motor callback missing");
    ESP_RETURN_ON_ERROR(s_motor_cb(speed_percent), TAG, "motor command rejected");
    s_motor_speed = speed_percent;

    stop_motor_watchdog();
    if (speed_percent != 0) {
        ESP_RETURN_ON_ERROR(esp_timer_start_once(s_motor_watchdog, MOTOR_WATCHDOG_TIMEOUT_US),
                            TAG, "start motor watchdog");
    }
    return ESP_OK;
}

static void motor_watchdog_callback(void *arg)
{
    (void)arg;
    if (s_motor_speed != 0 && s_motor_cb != NULL) {
        ESP_LOGW(TAG, "Motor command timeout; stopping motors");
        s_motor_speed = 0;
        ESP_ERROR_CHECK_WITHOUT_ABORT(s_motor_cb(0));
    }
}

static esp_err_t root_handler(httpd_req_t *req)
{
    add_common_headers(req);
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, CONTROL_PAGE, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t status_handler(httpd_req_t *req)
{
    char response[160];
    snprintf(response, sizeof(response),
             "{\"motor\":%d,\"servo\":%d,\"ssid\":\"%s\",\"ip\":\"%s\"}",
             s_motor_speed, s_servo_angle, WIFI_CONTROL_AP_SSID, WIFI_CONTROL_AP_IP);
    return send_json(req, response);
}

static esp_err_t motor_handler(httpd_req_t *req)
{
    int speed;
    if (read_int_query(req, "speed", &speed) != ESP_OK || speed < -100 || speed > 100) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "speed must be -100..100");
    }

    esp_err_t err = apply_motor_speed(speed);
    if (err != ESP_OK) {
        return send_service_unavailable(req,
                                        "motor unavailable; check GPIO46 hardware limitation");
    }

    char response[48];
    snprintf(response, sizeof(response), "{\"ok\":true,\"motor\":%d}", s_motor_speed);
    return send_json(req, response);
}

static esp_err_t servo_handler(httpd_req_t *req)
{
    int angle;
    if (read_int_query(req, "angle", &angle) != ESP_OK || angle < 0 || angle > 180) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "angle must be 0..180");
    }

    ESP_RETURN_ON_FALSE(s_servo_cb != NULL, ESP_ERR_INVALID_STATE, TAG, "servo callback missing");
    esp_err_t err = s_servo_cb(angle);
    if (err != ESP_OK) {
        return send_service_unavailable(req, "servo unavailable");
    }
    s_servo_angle = angle;

    char response[48];
    snprintf(response, sizeof(response), "{\"ok\":true,\"servo\":%d}", s_servo_angle);
    return send_json(req, response);
}

static esp_err_t stop_handler(httpd_req_t *req)
{
    esp_err_t err = apply_motor_speed(0);
    if (err != ESP_OK) {
        return send_service_unavailable(req, "motor unavailable");
    }
    return send_json(req, "{\"ok\":true,\"motor\":0}");
}

static esp_err_t start_http_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;
    config.max_uri_handlers = 8;

    ESP_RETURN_ON_ERROR(httpd_start(&s_http_server, &config), TAG, "start HTTP server");

    const httpd_uri_t handlers[] = {
        {.uri = "/",           .method = HTTP_GET, .handler = root_handler},
        {.uri = "/api/status", .method = HTTP_GET, .handler = status_handler},
        {.uri = "/api/motor",  .method = HTTP_GET, .handler = motor_handler},
        {.uri = "/api/servo",  .method = HTTP_GET, .handler = servo_handler},
        {.uri = "/api/stop",   .method = HTTP_GET, .handler = stop_handler},
    };

    for (size_t i = 0; i < sizeof(handlers) / sizeof(handlers[0]); ++i) {
        ESP_RETURN_ON_ERROR(httpd_register_uri_handler(s_http_server, &handlers[i]),
                            TAG, "register HTTP handler");
    }
    return ESP_OK;
}

esp_err_t wifi_control_init(wifi_motor_speed_cb_t motor_cb,
                            wifi_servo_angle_cb_t servo_cb)
{
    ESP_RETURN_ON_FALSE(motor_cb != NULL && servo_cb != NULL,
                        ESP_ERR_INVALID_ARG, TAG, "control callbacks required");
    s_motor_cb = motor_cb;
    s_servo_cb = servo_cb;

    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "initialize TCP/IP stack");
    esp_err_t event_err = esp_event_loop_create_default();
    if (event_err != ESP_OK && event_err != ESP_ERR_INVALID_STATE) {
        return event_err;
    }
    ESP_RETURN_ON_FALSE(esp_netif_create_default_wifi_ap() != NULL,
                        ESP_FAIL, TAG, "create Wi-Fi AP interface");

    wifi_init_config_t init_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&init_config), TAG, "initialize Wi-Fi");
    ESP_RETURN_ON_ERROR(esp_wifi_set_storage(WIFI_STORAGE_RAM), TAG, "set Wi-Fi storage");
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_AP), TAG, "set AP mode");

    wifi_config_t wifi_config = {0};
    snprintf((char *)wifi_config.ap.ssid, sizeof(wifi_config.ap.ssid), "%s", WIFI_CONTROL_AP_SSID);
    snprintf((char *)wifi_config.ap.password, sizeof(wifi_config.ap.password), "%s",
             WIFI_CONTROL_AP_PASSWORD);
    wifi_config.ap.ssid_len = strlen(WIFI_CONTROL_AP_SSID);
    wifi_config.ap.channel = 1;
    wifi_config.ap.max_connection = 4;
    wifi_config.ap.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.ap.pmf_cfg.capable = true;
    wifi_config.ap.pmf_cfg.required = false;

    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_AP, &wifi_config), TAG, "configure AP");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "start Wi-Fi AP");

    const esp_timer_create_args_t watchdog_args = {
        .callback = motor_watchdog_callback,
        .name = "motor_watchdog",
    };
    ESP_RETURN_ON_ERROR(esp_timer_create(&watchdog_args, &s_motor_watchdog),
                        TAG, "create motor watchdog");
    ESP_RETURN_ON_ERROR(start_http_server(), TAG, "start control server");

    ESP_LOGI(TAG, "Wi-Fi control ready: SSID=%s, URL=http://%s",
             WIFI_CONTROL_AP_SSID, WIFI_CONTROL_AP_IP);
    return ESP_OK;
}
