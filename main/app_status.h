#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "mpu6050.h"
#include "robot_types.h"

/** Reset the process-wide status snapshot before tasks start. */
void app_status_reset(void);

/** Merge control outputs with sensor/diagnostic fields and publish atomically. */
void app_status_publish_control(const robot_status_t *control_status,
                                bool drv8833_fault_active,
                                uint32_t queue_overflow_count);

/** Record one IMU read result; a NULL sample marks the sensor invalid. */
void app_status_record_imu(const mpu6050_raw_t *sample, int64_t sample_time_us);

/** Copy the current immutable snapshot for HTTP or UART diagnostics. */
bool app_status_get(robot_status_t *status);
