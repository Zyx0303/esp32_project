#include "mpu6050.h"

#include <string.h>
#include "esp_log.h"
#include "esp_check.h"

static const char *TAG = "mpu6050";

// Registers
#define REG_WHO_AM_I     0x75
#define REG_PWR_MGMT_1   0x6B
#define REG_SMPLRT_DIV   0x19
#define REG_CONFIG       0x1A
#define REG_GYRO_CONFIG  0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_INT_PIN_CFG  0x37
#define REG_INT_ENABLE   0x38
#define REG_INT_STATUS   0x3A
#define REG_ACCEL_XOUT_H 0x3B

static esp_err_t mpu_write_u8(mpu6050_t *imu, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(imu->dev, buf, sizeof(buf), -1);
}

static esp_err_t mpu_read(mpu6050_t *imu, uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(imu->dev, &reg, 1, data, len, -1);
}

esp_err_t mpu6050_init(mpu6050_t *imu, i2c_master_bus_handle_t bus, uint8_t i2c_addr)
{
    if (!imu || !bus) return ESP_ERR_INVALID_ARG;
    memset(imu, 0, sizeof(*imu));

    const i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = i2c_addr,
        .scl_speed_hz = 400000,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bus, &dev_cfg, &imu->dev), TAG, "add dev");

    uint8_t who = 0;
    ESP_RETURN_ON_ERROR(mpu_read(imu, REG_WHO_AM_I, &who, 1), TAG, "whoami");
    ESP_LOGI(TAG, "WHO_AM_I=0x%02x", who);

    // Wake up (clear sleep bit), use internal 8MHz oscillator (CLKSEL=0)
    ESP_RETURN_ON_ERROR(mpu_write_u8(imu, REG_PWR_MGMT_1, 0x00), TAG, "pwr");

    // Basic config: ~1kHz internal sample, set filters modestly
    ESP_RETURN_ON_ERROR(mpu_write_u8(imu, REG_SMPLRT_DIV, 0x07), TAG, "smplrt"); // 1kHz/(1+7)=125Hz if DLPF on
    ESP_RETURN_ON_ERROR(mpu_write_u8(imu, REG_CONFIG, 0x03), TAG, "dlpf");      // DLPF_CFG=3
    ESP_RETURN_ON_ERROR(mpu_write_u8(imu, REG_GYRO_CONFIG, 0x00), TAG, "gyro"); // ±250 dps
    ESP_RETURN_ON_ERROR(mpu_write_u8(imu, REG_ACCEL_CONFIG, 0x00), TAG, "acc"); // ±2g

    return ESP_OK;
}

esp_err_t mpu6050_config_data_ready_int(mpu6050_t *imu, bool enable_latched)
{
    if (!imu) return ESP_ERR_INVALID_ARG;

    // INT_PIN_CFG:
    // bit5 LATCH_INT_EN (1=latched until INT_STATUS read)
    // bit4 INT_ANYRD_2CLEAR (1=any read clears latched int)
    uint8_t int_pin_cfg = 0;
    if (enable_latched) {
        int_pin_cfg |= (1 << 5);
        int_pin_cfg |= (1 << 4);
    }
    ESP_RETURN_ON_ERROR(mpu_write_u8(imu, REG_INT_PIN_CFG, int_pin_cfg), TAG, "int pin cfg");

    // INT_ENABLE: bit0 DATA_RDY_EN
    ESP_RETURN_ON_ERROR(mpu_write_u8(imu, REG_INT_ENABLE, 0x01), TAG, "int en");
    return ESP_OK;
}

esp_err_t mpu6050_read_int_status(mpu6050_t *imu, uint8_t *int_status)
{
    if (!imu || !int_status) return ESP_ERR_INVALID_ARG;
    return mpu_read(imu, REG_INT_STATUS, int_status, 1);
}

esp_err_t mpu6050_read_raw(mpu6050_t *imu, mpu6050_raw_t *out)
{
    if (!imu || !out) return ESP_ERR_INVALID_ARG;
    uint8_t buf[14] = {0};
    ESP_RETURN_ON_ERROR(mpu_read(imu, REG_ACCEL_XOUT_H, buf, sizeof(buf)), TAG, "read raw");

    out->ax = (int16_t)((buf[0] << 8) | buf[1]);
    out->ay = (int16_t)((buf[2] << 8) | buf[3]);
    out->az = (int16_t)((buf[4] << 8) | buf[5]);
    out->temp_raw = (int16_t)((buf[6] << 8) | buf[7]);
    out->gx = (int16_t)((buf[8] << 8) | buf[9]);
    out->gy = (int16_t)((buf[10] << 8) | buf[11]);
    out->gz = (int16_t)((buf[12] << 8) | buf[13]);
    return ESP_OK;
}

