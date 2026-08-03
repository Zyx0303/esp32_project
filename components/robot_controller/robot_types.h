#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    ROBOT_STATE_BOOT = 0,
    ROBOT_STATE_SAFE,
    ROBOT_STATE_READY,
    ROBOT_STATE_RUNNING,
    ROBOT_STATE_FAULT,
    ROBOT_STATE_ESTOP,
} robot_state_t;

typedef enum {
    ROBOT_CMD_ARM = 0,
    ROBOT_CMD_DISARM,
    ROBOT_CMD_ESTOP,
    ROBOT_CMD_CLEAR_FAULT,
    ROBOT_CMD_DRIVE,
    ROBOT_CMD_MOTOR_DIRECT,
    ROBOT_CMD_SERVO,
    ROBOT_CMD_POWER_CONTROL,
} robot_command_type_t;

typedef enum {
    ROBOT_COMMAND_SOURCE_INTERNAL = 0,
    ROBOT_COMMAND_SOURCE_SERIAL,
    ROBOT_COMMAND_SOURCE_WIFI,
    ROBOT_COMMAND_SOURCE_COMPAT,
    ROBOT_COMMAND_SOURCE_COUNT,
} robot_command_source_t;

typedef enum {
    ROBOT_COMMAND_ACCEPTED = 0,
    ROBOT_COMMAND_INVALID_ARGUMENT,
    ROBOT_COMMAND_INVALID_STATE,
    ROBOT_COMMAND_EXPIRED,
    ROBOT_COMMAND_OUT_OF_ORDER,
    ROBOT_COMMAND_HARDWARE_ERROR,
} robot_command_result_t;

typedef uint32_t robot_fault_mask_t;

enum {
    ROBOT_FAULT_NONE = 0,
    ROBOT_FAULT_DRV8833 = 1U << 0,
    ROBOT_FAULT_IMU_INIT = 1U << 1,
    ROBOT_FAULT_IMU_STALE = 1U << 2,
    ROBOT_FAULT_COMMAND_TIMEOUT = 1U << 3,
    ROBOT_FAULT_QUEUE_OVERFLOW = 1U << 4,
    ROBOT_FAULT_CONFIG_INVALID = 1U << 5,
    ROBOT_FAULT_INTERNAL = 1U << 6,
};

typedef struct {
    robot_command_type_t type;
    uint32_t sequence;
    int64_t received_at_us;
    robot_command_source_t source;
    union {
        struct {
            int16_t throttle;
            int16_t steering;
        } drive;
        struct {
            int16_t motor_a;
            int16_t motor_b;
        } motors;
        struct {
            int16_t angle_deg;
        } servo;
        struct {
            bool asserted;
        } power;
    } value;
} robot_command_t;

typedef struct {
    uint32_t command_timeout_ms;
    uint32_t command_max_age_ms;
    uint32_t reverse_deadtime_ms;
    uint8_t motor_ramp_percent_per_10ms;
    bool invert_motor_a;
    bool invert_motor_b;
    int16_t servo_min_deg;
    int16_t servo_max_deg;
    int16_t servo_initial_deg;
    uint16_t servo_slew_deg_per_second;
} robot_control_config_t;

typedef struct {
    robot_state_t state;
    bool armed;
    bool estop_latched;
    robot_fault_mask_t faults;
    int16_t motor_a_target;
    int16_t motor_b_target;
    int16_t motor_a_output;
    int16_t motor_b_output;
    int16_t servo_target_deg;
    int16_t servo_output_deg;
    bool servo_enabled;
    bool power_asserted;
    bool drv8833_fault_active;
    bool imu_valid;
    int16_t imu_ax;
    int16_t imu_ay;
    int16_t imu_az;
    int16_t imu_gx;
    int16_t imu_gy;
    int16_t imu_gz;
    int16_t imu_temp_raw;
    int64_t imu_last_update_us;
    int64_t last_motion_command_us;
    uint32_t uptime_ms;
    uint32_t free_heap_bytes;
    uint32_t imu_sample_count;
    uint32_t imu_error_count;
    uint32_t queue_overflow_count;
    uint32_t commands_received;
    uint32_t commands_rejected;
    uint32_t commands_expired;
    uint32_t watchdog_stops;
} robot_status_t;
