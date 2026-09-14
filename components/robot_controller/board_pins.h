#pragma once

#include "driver/gpio.h"

// 2026-07-31 schematic: ESP32-S3-WROOM-1-N16R8 peripheral mapping.

// MPU6050 (AD0 is tied low, so the 7-bit I2C address is 0x68)
#define PIN_MPU_INT       GPIO_NUM_8  // 100 Hz DATA_READY rising edge
#define PIN_I2C_SCL       GPIO_NUM_11
#define PIN_I2C_SDA       GPIO_NUM_12
#define MPU6050_I2C_ADDR  0x68

// DRV8833. GPIO46 is output-capable on ESP32-S3, but is also a strapping pin;
// do not externally force it to an invalid level during reset sampling.
#define PIN_DRV_NSLEEP GPIO_NUM_3
#define PIN_DRV_AIN1   GPIO_NUM_46
#define PIN_DRV_AIN2   GPIO_NUM_9
#define PIN_DRV_BIN1   GPIO_NUM_5
#define PIN_DRV_BIN2   GPIO_NUM_6
#define PIN_DRV_NFAULT GPIO_NUM_4

// Servo
#define PIN_SERVO_PWM  GPIO_NUM_45

// External power-control interface. GPIO35 drives a 2N7002 gate through 330R;
// driving the GPIO high pulls the external interface low.
#define PIN_POWER_CTRL GPIO_NUM_35

// Boot / reset (strap)
#define PIN_BOOT_RESET GPIO_NUM_0

// I2C configuration
#define I2C_MASTER_NUM      I2C_NUM_0
#define I2C_MASTER_FREQ_HZ   400000
