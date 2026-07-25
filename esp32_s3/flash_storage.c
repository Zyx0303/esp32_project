#include "flash_storage.h"

#include <string.h>
#include "esp_log.h"
#include "esp_partition.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

static const char *TAG = "flash_storage";

#define BUFFER_SIZE 64

static const esp_partition_t *s_partition;
static QueueHandle_t s_queue;
static StaticQueue_t s_queue_buffer;
static uint8_t s_queue_storage[BUFFER_SIZE * sizeof(imu_data_entry_t)];

static imu_data_entry_t s_buffer[BUFFER_SIZE];
static size_t s_buffer_count;
static uint32_t s_total_written;
static uint32_t s_seq;
static TaskHandle_t s_flush_task_handle;

static void flush_task(void *arg)
{
    imu_data_entry_t entry;

    while (true) {
        if (xQueueReceive(s_queue, &entry, portMAX_DELAY) == pdTRUE) {
            s_buffer[s_buffer_count++] = entry;

            if (s_buffer_count >= BUFFER_SIZE) {
                flash_storage_flush();
            }
        }
    }
}

esp_err_t flash_storage_init(void)
{
    s_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_FAT, "imu_data");
    if (s_partition == NULL) {
        ESP_LOGE(TAG, "partition 'imu_data' not found");
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "partition found: addr=0x%" PRIx32 " size=0x%" PRIx32, (uint32_t)s_partition->address, (uint32_t)s_partition->size);

    s_queue = xQueueCreateStatic(BUFFER_SIZE, sizeof(imu_data_entry_t), s_queue_storage, &s_queue_buffer);
    if (s_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }

    s_buffer_count = 0;
    s_total_written = 0;
    s_seq = 0;

    BaseType_t ret = xTaskCreatePinnedToCore(flush_task, "flush_task", 4096, NULL, 2, &s_flush_task_handle, 0);
    if (ret != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "flash storage init OK");
    return ESP_OK;
}

esp_err_t flash_storage_write_entry(const imu_data_entry_t *entry)
{
    if (s_queue == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    imu_data_entry_t e = *entry;
    e.magic = IMU_DATA_MAGIC;
    e.seq = s_seq++;

    if (xQueueSend(s_queue, &e, 0) != pdTRUE) {
        ESP_LOGW(TAG, "queue full, dropping entry");
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

esp_err_t flash_storage_flush(void)
{
    if (s_buffer_count == 0 || s_partition == NULL) {
        return ESP_OK;
    }

    size_t entry_size = sizeof(imu_data_entry_t);
    size_t data_size = s_buffer_count * entry_size;
    size_t flash_addr = (s_total_written % s_partition->size);

    // Check if we need to wrap around
    if (flash_addr + data_size > s_partition->size) {
        flash_addr = 0;
        s_total_written = 0;
        ESP_LOGW(TAG, "wrap around, resetting");
    }

    // Calculate sector boundaries (4KB sectors)
    size_t sector_size = 4096;
    size_t start_sector = flash_addr / sector_size;
    size_t end_sector = (flash_addr + data_size - 1) / sector_size;
    size_t sectors_to_erase = end_sector - start_sector + 1;

    // Erase all sectors involved
    for (size_t i = 0; i < sectors_to_erase; i++) {
        esp_err_t err = esp_partition_erase_range(s_partition, (start_sector + i) * sector_size, sector_size);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "erase failed at sector %d", (int)(start_sector + i));
            return err;
        }
    }

    // Write entries
    for (size_t i = 0; i < s_buffer_count; i++) {
        esp_err_t err = esp_partition_write_raw(s_partition, flash_addr + i * entry_size,
                                                 &s_buffer[i], entry_size);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "write failed at 0x%" PRIx32, (uint32_t)(flash_addr + i * entry_size));
            return err;
        }
    }

    ESP_LOGI(TAG, "flushed %d entries to 0x%" PRIx32 " (%d sectors)", s_buffer_count, (uint32_t)flash_addr, (int)sectors_to_erase);
    s_total_written += data_size;
    s_buffer_count = 0;

    return ESP_OK;
}

esp_err_t flash_storage_read_all(void (*callback)(const imu_data_entry_t *, size_t))
{
    if (s_partition == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    size_t addr = 0;
    size_t count = 0;
    imu_data_entry_t entry;

    while (addr + sizeof(entry) <= s_partition->size) {
        esp_err_t err = esp_partition_read_raw(s_partition, addr, &entry, sizeof(entry));
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "read failed at 0x%" PRIx32 ", stopping", (uint32_t)addr);
            break;
        }

        if (entry.magic == IMU_DATA_MAGIC) {
            callback(&entry, count++);
        }

        addr += sizeof(entry);
    }

    ESP_LOGI(TAG, "read %d entries from flash", count);
    return ESP_OK;
}

size_t flash_storage_get_count(void)
{
    return s_seq;
}
