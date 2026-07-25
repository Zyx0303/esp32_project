#pragma once

#include "driver/gpio.h"

// Final schematic: ESP32-S3-WROOM-1-N16R8 peripheral mapping.

// MPU6050 (AD0 is tied low, so the 7-bit I2C address is 0x68)
#define PIN_MPU_INT       GPIO_NUM_8
#define PIN_I2C_SCL       GPIO_NUM_11
#define PIN_I2C_SDA       GPIO_NUM_12
#define MPU6050_I2C_ADDR  0x68

// DRV8833
// NOTE: The final schematic connects AIN1 to GPIO46. GPIO46 is input-only on
// ESP32-S3, so firmware must keep the driver asleep until this net is reworked
// to an output-capable GPIO.
#define PIN_DRV_NSLEEP GPIO_NUM_3
#define PIN_DRV_AIN1   GPIO_NUM_46
#define PIN_DRV_AIN2   GPIO_NUM_9
#define PIN_DRV_BIN1   GPIO_NUM_5
#define PIN_DRV_BIN2   GPIO_NUM_6
#define PIN_DRV_NFAULT GPIO_NUM_4

// Servo
#define PIN_SERVO_PWM  GPIO_NUM_45

// Boot / reset (strap)
#define PIN_BOOT_RESET GPIO_NUM_0

// I2C configuration
#define I2C_MASTER_NUM      I2C_NUM_0
#define I2C_MASTER_FREQ_HZ   400000
