#include "imu_log.h"
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_vfs_fat.h"
#include "esp_partition.h"
#include "esp_timer.h"
#include "ff.h"
#include "diskio_wl.h"
#include "wear_levelling.h"

#define LOG_ROOT "/imu"
#define QUEUE_LENGTH 256
/* Leave room for buffered writes and the log footer on the mounted volume. */
#define LOG_FREE_RESERVE (16 * 1024)

typedef struct {
    mpu6050_raw_t sample;
    int64_t time_us;
    uint32_t sequence, ready;
    esp_err_t error;
} log_sample_t;

static QueueHandle_t s_queue;
static SemaphoreHandle_t s_file_lock;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static bool s_recording, s_sealed;
static uint32_t s_dropped;
static int64_t s_last_valid_us;
static FILE *s_file;
static wl_handle_t s_wl = WL_INVALID_HANDLE;
static char s_name[16], s_path[32], s_fat_path[32];
static const char *s_error = "";
static size_t s_bytes;
static uint64_t s_log_limit;
static bool s_download_active;

typedef struct {
    httpd_req_t *request;
    FILE *file;
    char name[16];
    FSIZE_t size;
} log_download_t;

static void set_recording(bool active)
{
    portENTER_CRITICAL(&s_lock);
    s_recording = active;
    portEXIT_CRITICAL(&s_lock);
}

void imu_log_record(const mpu6050_raw_t *sample, int64_t time_us,
                    uint32_t sequence, uint32_t ready_count, esp_err_t error)
{
    log_sample_t row = {*sample, time_us, sequence, ready_count, error};
    portENTER_CRITICAL(&s_lock);
    if (error == ESP_OK) s_last_valid_us = time_us;
    if (s_recording && xQueueSend(s_queue, &row, 0) != pdTRUE) s_dropped++;
    portEXIT_CRITICAL(&s_lock);
}

/* Caller holds file lock; queue admission is stopped before draining/finalizing. */
static void seal_file(void)
{
    set_recording(false);
    if (!s_file) {
        if (s_fat_path[0]) s_sealed = f_chmod(s_fat_path, AM_RDO, AM_RDO) == FR_OK;
        return;
    }
    portENTER_CRITICAL(&s_lock);
    s_dropped += uxQueueMessagesWaiting(s_queue);
    uint32_t dropped = s_dropped;
    portEXIT_CRITICAL(&s_lock);
    xQueueReset(s_queue);
    if (fprintf(s_file, "# queue_dropped=%" PRIu32 ", reason=%s\n", dropped, s_error) < 0)
        s_error = "write_failed";
    if (fflush(s_file) != 0) s_error = "flush_failed";
    if (fclose(s_file) != 0) s_error = "close_failed";
    s_file = NULL;
    s_sealed = f_chmod(s_fat_path, AM_RDO, AM_RDO) == FR_OK;
    if (!s_sealed) s_error = "readonly_failed";
}

static void drain_samples(void)
{
    log_sample_t row;
    while (s_file && xQueueReceive(s_queue, &row, 0) == pdTRUE) {
        int count = fprintf(s_file,
            "%" PRId64 ",%" PRIu32 ",%" PRIu32 ",%d,%d,%d,%d,%d,%d,%d,%d\n",
            row.time_us, row.sequence, row.ready, (int)row.error,
            row.sample.ax, row.sample.ay, row.sample.az,
            row.sample.gx, row.sample.gy, row.sample.gz, row.sample.temp_raw);
        if (count < 0) { s_error = "write_failed"; seal_file(); break; }
        s_bytes += count;
        if (s_bytes >= s_log_limit) { s_error = "size_limit"; seal_file(); break; }
    }
}

static void writer_task(void *arg)
{
    (void)arg;
    while (true) {
        xSemaphoreTake(s_file_lock, portMAX_DELAY);
        drain_samples();
        xSemaphoreGive(s_file_lock);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

esp_err_t imu_log_init(void)
{
    /* Only format a completely erased partition. Never auto-format existing logs on mount failure. */
    const esp_partition_t *partition = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_FAT, "imu_data");
    if (!partition) return ESP_ERR_NOT_FOUND;
    bool blank = true;
    uint8_t block[256];
    for (size_t offset = 0; offset < partition->size && blank; offset += sizeof(block)) {
        esp_err_t err = esp_partition_read(partition, offset, block, sizeof(block));
        if (err != ESP_OK) return err;
        for (size_t i = 0; i < sizeof(block); ++i) if (block[i] != 0xff) { blank = false; break; }
    }
    esp_vfs_fat_mount_config_t config = {
        .format_if_mount_failed = blank, .max_files = 2, .allocation_unit_size = 4096,
    };
    esp_err_t err = esp_vfs_fat_spiflash_mount_rw_wl(LOG_ROOT, "imu_data", &config, &s_wl);
    if (err != ESP_OK) return err;
    s_queue = xQueueCreate(QUEUE_LENGTH, sizeof(log_sample_t));
    s_file_lock = xSemaphoreCreateMutex();
    if (!s_queue || !s_file_lock) {
        if (s_queue) vQueueDelete(s_queue);
        if (s_file_lock) vSemaphoreDelete(s_file_lock);
        s_queue = NULL;
        s_file_lock = NULL;
        return ESP_ERR_NO_MEM;
    }
    /* Recover the latest completed file after reboot; interrupted logs remain untouched. */
    for (unsigned index = 1; index <= 999999; ++index) {
        char candidate[32];
        FILINFO info;
        snprintf(candidate, sizeof(candidate), "%u:/I%06u.log",
                 (unsigned)ff_diskio_get_pdrv_wl(s_wl), index);
        if (f_stat(candidate, &info) != FR_OK) break;
        if (info.fattrib & AM_RDO) {
            snprintf(s_name, sizeof(s_name), "I%06u.log", index);
            snprintf(s_path, sizeof(s_path), LOG_ROOT "/%s", s_name);
            snprintf(s_fat_path, sizeof(s_fat_path), "%s", candidate);
            s_sealed = true;
        }
    }
    if (xTaskCreate(writer_task, "imu_writer", 4096, NULL, 3, NULL) != pdPASS) {
        vSemaphoreDelete(s_file_lock);
        s_file_lock = NULL;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

static esp_err_t reply(httpd_req_t *req)
{
    char json[256];
    portENTER_CRITICAL(&s_lock);
    bool active = s_recording;
    uint32_t dropped = s_dropped;
    portEXIT_CRITICAL(&s_lock);
    snprintf(json, sizeof(json),
        "{\"ok\":true,\"recording\":%s,\"sealed\":%s,\"file\":\"%s\",\"dropped\":%" PRIu32 ",\"error\":\"%s\"}",
        active ? "true" : "false", s_sealed ? "true" : "false", s_name, dropped, s_error);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, json);
}

static void set_download_active(bool active)
{
    portENTER_CRITICAL(&s_lock);
    s_download_active = active;
    portEXIT_CRITICAL(&s_lock);
}

static void download_task(void *argument)
{
    log_download_t *download = argument;
    httpd_req_t *req = download->request;
    httpd_handle_t server = req->handle;
    int socket = httpd_req_to_sockfd(req);
    char disposition[80], length[24], buffer[1024];

    snprintf(disposition, sizeof(disposition), "attachment; filename=\"%s\"", download->name);
    snprintf(length, sizeof(length), "%" PRIu64, (uint64_t)download->size);
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Content-Disposition", disposition);
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_hdr(req, "X-Log-Size", length);

    esp_err_t result = ESP_OK;
    size_t count;
    while ((count = fread(buffer, 1, sizeof(buffer), download->file)) > 0) {
        result = httpd_resp_send_chunk(req, buffer, count);
        if (result != ESP_OK) break;
    }
    if (ferror(download->file)) result = ESP_FAIL;
    fclose(download->file);
    if (result == ESP_OK) result = httpd_resp_send_chunk(req, NULL, 0);
    httpd_req_async_handler_complete(req);
    if (result != ESP_OK) httpd_sess_trigger_close(server, socket);

    free(download);
    set_download_active(false);
    vTaskDelete(NULL);
}

/* Caller holds the file lock while validating and opening a sealed log. */
static esp_err_t start_download(httpd_req_t *req)
{
    char name[16], path[32], fat_path[32], query[64];
    snprintf(name, sizeof(name), "%s", s_name);
    if (httpd_req_get_url_query_len(req) > 0 &&
        (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
         httpd_query_key_value(query, "file", name, sizeof(name)) != ESP_OK)) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid filename query");
    }

    bool valid = strlen(name) == 11 && name[0] == 'I' && strcmp(name + 7, ".log") == 0;
    for (int i = 1; valid && i <= 6; ++i) valid = name[i] >= '0' && name[i] <= '9';
    snprintf(path, sizeof(path), LOG_ROOT "/%s", name);
    snprintf(fat_path, sizeof(fat_path), "%u:/%s", (unsigned)ff_diskio_get_pdrv_wl(s_wl), name);
    FILINFO info;
    if (!valid || f_stat(fat_path, &info) != FR_OK || !(info.fattrib & AM_RDO)) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Stop and seal recording first");
    }

    // Keep at most one socket and one FAT file occupied by a download.
    portENTER_CRITICAL(&s_lock);
    bool busy = s_download_active;
    portEXIT_CRITICAL(&s_lock);
    if (busy) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        return httpd_resp_sendstr(req, "Another log download is in progress");
    }
    log_download_t *download = calloc(1, sizeof(*download));
    if (!download) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Log download unavailable");
    }
    download->file = fopen(path, "rb");
    if (!download->file ||
        httpd_req_async_handler_begin(req, &download->request) != ESP_OK) {
        if (download->file) fclose(download->file);
        free(download);
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Log download unavailable");
    }
    snprintf(download->name, sizeof(download->name), "%s", name);
    download->size = info.fsize;
    set_download_active(true);
    if (xTaskCreate(download_task, "imu_download", 4096, download, 3, NULL) != pdPASS) {
        fclose(download->file);
        httpd_resp_send_err(download->request, HTTPD_500_INTERNAL_SERVER_ERROR, "Log download unavailable");
        httpd_req_async_handler_complete(download->request);
        free(download);
        set_download_active(false);
    }
    return ESP_OK;
}

static esp_err_t log_handler(httpd_req_t *req)
{
    if (!s_file_lock) return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "IMU storage unavailable");
    xSemaphoreTake(s_file_lock, portMAX_DELAY);
    esp_err_t result = ESP_OK;
    if (strcmp(req->uri, "/api/v1/imu/log/start") == 0) {
        if (!s_file) {
            portENTER_CRITICAL(&s_lock);
            int64_t last_valid = s_last_valid_us;
            portEXIT_CRITICAL(&s_lock);
            if (!last_valid || esp_timer_get_time() - last_valid > 100000) {
                result = httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "IMU has no fresh interrupt samples");
                goto done;
            }
            uint64_t total_bytes, free_bytes;
            if (esp_vfs_fat_info(LOG_ROOT, &total_bytes, &free_bytes) != ESP_OK ||
                free_bytes <= LOG_FREE_RESERVE) {
                result = httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "IMU log storage is full or unreadable");
                goto done;
            }
            s_log_limit = free_bytes - LOG_FREE_RESERVE;
            /* Unique 8.3 names work without optional FatFs long filename support. */
            char previous_name[sizeof(s_name)], previous_path[sizeof(s_path)], previous_fat[sizeof(s_fat_path)];
            memcpy(previous_name, s_name, sizeof(s_name));
            memcpy(previous_path, s_path, sizeof(s_path));
            memcpy(previous_fat, s_fat_path, sizeof(s_fat_path));
            struct stat st;
            unsigned index;
            for (index = 1; index <= 999999; ++index) {
                snprintf(s_name, sizeof(s_name), "I%06u.log", index);
                snprintf(s_path, sizeof(s_path), LOG_ROOT "/%s", s_name);
                if (stat(s_path, &st) != 0) break;
            }
            snprintf(s_fat_path, sizeof(s_fat_path), "%u:/%s", (unsigned)ff_diskio_get_pdrv_wl(s_wl), s_name);
            s_file = index <= 999999 ? fopen(s_path, "wx") : NULL;
            if (!s_file) {
                memcpy(s_name, previous_name, sizeof(s_name));
                memcpy(s_path, previous_path, sizeof(s_path));
                memcpy(s_fat_path, previous_fat, sizeof(s_fat_path));
                result = httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Cannot create log (storage full?)");
                goto done;
            }
            s_error = "";
            s_sealed = false;
            xQueueReset(s_queue);
            portENTER_CRITICAL(&s_lock);
            s_dropped = 0;
            portEXIT_CRITICAL(&s_lock);
            int count = fprintf(s_file, "# MPU6050 100Hz raw; time_us=ESP read-start uptime; ready>1 means missed frames\n"
                "time_us,sequence,ready_count,error,ax,ay,az,gx,gy,gz,temp_raw\n");
            if (count < 0 || fflush(s_file) != 0) {
                s_error = "write_failed";
                seal_file();
            } else {
                s_bytes = count;
                set_recording(true);
            }
        }
    } else if (strcmp(req->uri, "/api/v1/imu/log/stop") == 0) {
        set_recording(false);
        drain_samples();
        seal_file();
    } else if (strncmp(req->uri, "/api/v1/imu/log/download", strlen("/api/v1/imu/log/download")) == 0) {
        result = start_download(req);
        goto done;
    }
    result = reply(req);
done:
    xSemaphoreGive(s_file_lock);
    return result;
}

esp_err_t imu_log_register_http(httpd_handle_t server)
{
    const httpd_uri_t routes[] = {
        {.uri="/api/v1/imu/log/start", .method=HTTP_POST, .handler=log_handler},
        {.uri="/api/v1/imu/log/stop", .method=HTTP_POST, .handler=log_handler},
        {.uri="/api/v1/imu/log/status", .method=HTTP_GET, .handler=log_handler},
        {.uri="/api/v1/imu/log/download", .method=HTTP_GET, .handler=log_handler},
    };
    for (size_t i = 0; i < sizeof(routes)/sizeof(routes[0]); ++i) {
        esp_err_t err = httpd_register_uri_handler(server, &routes[i]);
        if (err != ESP_OK) return err;
    }
    return ESP_OK;
}
