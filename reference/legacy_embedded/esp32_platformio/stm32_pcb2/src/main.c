#include "stm32f4xx_hal.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#ifndef PCB_SERVO_ID
#define PCB_SERVO_ID 6U
#endif

#ifndef CALIBRATE_ZERO_ON_BOOT
#define CALIBRATE_ZERO_ON_BOOT 0
#endif

#ifndef CALIBRATE_SET_ORIGIN_ON_BOOT
#define CALIBRATE_SET_ORIGIN_ON_BOOT 0
#endif

#ifndef SCAN_SERVO_IDS_ON_BOOT
#define SCAN_SERVO_IDS_ON_BOOT 0
#endif

#ifndef READ_SERVO_ANGLE_ON_BOOT
#define READ_SERVO_ANGLE_ON_BOOT 0
#endif

#ifndef SERVO_STARTUP_TEST
#define SERVO_STARTUP_TEST 0
#endif

#define CONTROL_HEAD_1 0xFFU
#define CONTROL_HEAD_2 0xFAU
#define CONTROL_TAIL_1 0x88U
#define CONTROL_TAIL_2 0x77U

#define MODE_STOP_HOLD 0x00U
#define MODE_WORM 0x01U
#define MODE_SNAKE_FORWARD 0x02U
#define MODE_DIRECT_OFFSET 0x10U
#define MODE_GO_CENTER 0x11U

#define SERVO_CENTER_DEG 0.0f
#define SERVO_DIRECTION (-1.0f)
#define MAX_LOGICAL_OFFSET_DEG 45.0f
#define SNAKE_COMMAND_FULL_SCALE_DEG 90.0f
#define SERVO_SPEED_DEG_S 30.0f
#define SERVO_ACCEL_MS 100U
#define SERVO_DECEL_MS 100U
#define SERVO_POWER_MW 6000U

#define CONTROL_PERIOD_MS 50U
#define TWO_PI 6.28318530718f
#define SNAKE_FREQUENCY_HZ 0.2f
#define PCB2_PHASE_OFFSET (TWO_PI / 3.0f)

static UART_HandleTypeDef huart1;
static UART_HandleTypeDef huart2;

static bool snake_enabled = false;
static float snake_amplitude_deg = 0.0f;
static float snake_phase = 0.0f;
static uint32_t last_control_ms = 0U;

volatile uint32_t g_calibration_status = 0U;
volatile int16_t g_calibration_angle_tenths = INT16_MIN;
volatile uint32_t g_found_servo_id = UINT32_MAX;

static void SystemClock_Config(void);
static void GPIO_Init(void);
static void USART1_Init(void);
static void USART2_Init(void);
static void Error_Handler(void);

static void write_u16_le(uint8_t *dst, uint16_t value) {
    dst[0] = (uint8_t)(value & 0xFFU);
    dst[1] = (uint8_t)(value >> 8U);
}

static void servo_send_packet(uint8_t command, const uint8_t *content, uint8_t length) {
    uint8_t packet[20];
    uint8_t checksum = 0U;
    uint8_t index = 0U;

    packet[index++] = 0x12U;
    packet[index++] = 0x4CU;
    packet[index++] = command;
    packet[index++] = length;
    for (uint8_t i = 0U; i < length; ++i) {
        packet[index++] = content[i];
    }
    for (uint8_t i = 0U; i < index; ++i) {
        checksum = (uint8_t)(checksum + packet[i]);
    }
    packet[index++] = checksum;

    if (HAL_UART_Transmit(&huart2, packet, index, 20U) != HAL_OK) {
        Error_Handler();
    }
}

static bool servo_receive_response(uint8_t expected_command, uint8_t *content,
                                   uint8_t expected_length, uint32_t timeout_ms) {
    uint8_t packet[20];
    uint8_t index = 0U;
    uint8_t checksum = 0U;
    uint32_t deadline = HAL_GetTick() + timeout_ms;

    while ((int32_t)(deadline - HAL_GetTick()) > 0) {
        uint8_t byte;
        if (HAL_UART_Receive(&huart2, &byte, 1U, 5U) != HAL_OK) {
            continue;
        }

        if (index == 0U) {
            if (byte == 0x05U) {
                packet[index++] = byte;
            }
            continue;
        }
        if (index == 1U && byte != 0x1CU) {
            index = (byte == 0x05U) ? 1U : 0U;
            continue;
        }

        packet[index++] = byte;
        if (index == 4U && (packet[2] != expected_command || packet[3] != expected_length)) {
            index = 0U;
            continue;
        }
        if (index < (uint8_t)(expected_length + 5U)) {
            continue;
        }

        for (uint8_t i = 0U; i < index - 1U; ++i) {
            checksum = (uint8_t)(checksum + packet[i]);
        }
        if (checksum != packet[index - 1U]) {
            return false;
        }
        for (uint8_t i = 0U; i < expected_length; ++i) {
            content[i] = packet[4U + i];
        }
        return true;
    }
    return false;
}

static bool servo_ping_id(uint8_t servo_id) {
    uint8_t request[1] = {servo_id};
    uint8_t response[1];

    servo_send_packet(0x01U, request, sizeof(request));
    return servo_receive_response(0x01U, response, sizeof(response), 250U) &&
           response[0] == servo_id;
}

static bool servo_ping_broadcast(uint8_t *detected_id) {
    uint8_t request[1] = {0xFFU};
    uint8_t response[1];

    servo_send_packet(0x01U, request, sizeof(request));
    if (!servo_receive_response(0x01U, response, sizeof(response), 300U)) {
        return false;
    }
    *detected_id = response[0];
    return response[0] != 0xFFU;
}

static bool servo_read_angle(int16_t *angle_tenths) {
    uint8_t request[1] = {PCB_SERVO_ID};
    uint8_t response[3];

    servo_send_packet(0x0AU, request, sizeof(request));
    if (!servo_receive_response(0x0AU, response, sizeof(response), 250U) ||
        response[0] != PCB_SERVO_ID) {
        return false;
    }
    *angle_tenths = (int16_t)((uint16_t)response[1] | ((uint16_t)response[2] << 8U));
    return true;
}

static void servo_set_current_as_origin(void) {
    uint8_t content[2] = {PCB_SERVO_ID, 0x00U};
    servo_send_packet(0x17U, content, sizeof(content));
}

static float clamp_offset(float offset_deg) {
    if (offset_deg > MAX_LOGICAL_OFFSET_DEG) {
        return MAX_LOGICAL_OFFSET_DEG;
    }
    if (offset_deg < -MAX_LOGICAL_OFFSET_DEG) {
        return -MAX_LOGICAL_OFFSET_DEG;
    }
    return offset_deg;
}

static void servo_set_logical_offset(float logical_offset_deg) {
    uint8_t content[11];
    float target_deg;
    int16_t position_tenths;

    logical_offset_deg = clamp_offset(logical_offset_deg);
    target_deg = SERVO_CENTER_DEG + SERVO_DIRECTION * logical_offset_deg;
    position_tenths = (int16_t)lroundf(target_deg * 10.0f);

    content[0] = PCB_SERVO_ID;
    write_u16_le(&content[1], (uint16_t)position_tenths);
    write_u16_le(&content[3], (uint16_t)lroundf(SERVO_SPEED_DEG_S * 10.0f));
    write_u16_le(&content[5], SERVO_ACCEL_MS);
    write_u16_le(&content[7], SERVO_DECEL_MS);
    write_u16_le(&content[9], SERVO_POWER_MW);
    servo_send_packet(0x0CU, content, sizeof(content));
}

static void servo_stop_hold(void) {
    uint8_t content[4];

    content[0] = PCB_SERVO_ID;
    content[1] = 0x11U;
    write_u16_le(&content[2], SERVO_POWER_MW);
    servo_send_packet(0x18U, content, sizeof(content));
}

static bool control_frame_push(uint8_t byte, uint8_t *mode, uint8_t *parameter) {
    static uint8_t frame[6];
    static uint8_t index = 0U;

    if (index == 0U) {
        if (byte == CONTROL_HEAD_1) {
            frame[index++] = byte;
        }
        return false;
    }

    if (index == 1U && byte != CONTROL_HEAD_2) {
        index = (byte == CONTROL_HEAD_1) ? 1U : 0U;
        return false;
    }

    frame[index++] = byte;
    if (index < sizeof(frame)) {
        return false;
    }
    index = 0U;

    if (frame[4] != CONTROL_TAIL_1 || frame[5] != CONTROL_TAIL_2) {
        return false;
    }

    *mode = frame[2];
    *parameter = frame[3];
    return true;
}

static void handle_control_command(uint8_t mode, uint8_t parameter) {
    switch (mode) {
        case MODE_STOP_HOLD:
            snake_enabled = false;
            servo_stop_hold();
            break;

        case MODE_WORM:
            /* Existing PC software uses worm gait 8 as its stop button. */
            if (parameter == 8U) {
                snake_enabled = false;
                servo_stop_hold();
            }
            break;

        case MODE_SNAKE_FORWARD:
            if (parameter == 0U) {
                snake_enabled = false;
                servo_stop_hold();
                break;
            }
            snake_amplitude_deg = ((float)parameter / 255.0f) * SNAKE_COMMAND_FULL_SCALE_DEG;
            snake_amplitude_deg = clamp_offset(snake_amplitude_deg);
            snake_phase = -PCB2_PHASE_OFFSET;
            snake_enabled = true;
            servo_set_logical_offset(0.0f);
            last_control_ms = HAL_GetTick();
            break;

        case MODE_DIRECT_OFFSET: {
            float offset_deg = (((float)parameter / 255.0f) * 2.0f - 1.0f) * MAX_LOGICAL_OFFSET_DEG;
            snake_enabled = false;
            servo_set_logical_offset(offset_deg);
            break;
        }

        case MODE_GO_CENTER:
            snake_enabled = false;
            servo_set_logical_offset(0.0f);
            break;

        default:
            break;
    }
}

int main(void) {
    HAL_Init();
    SystemClock_Config();
    GPIO_Init();
    USART1_Init();
    USART2_Init();

    last_control_ms = HAL_GetTick();

#if SERVO_STARTUP_TEST
    HAL_Delay(300U);
    servo_set_logical_offset(20.0f);
    HAL_Delay(2000U);
    servo_set_logical_offset(0.0f);
#endif

#if CALIBRATE_ZERO_ON_BOOT
    HAL_Delay(1500U);
    servo_set_logical_offset(0.0f);
#endif

#if CALIBRATE_SET_ORIGIN_ON_BOOT
    HAL_Delay(1000U);
    if (servo_ping_id(PCB_SERVO_ID)) {
        int16_t angle_tenths;
        g_calibration_status = 1U;
        HAL_Delay(20U);
        servo_set_current_as_origin();
        g_calibration_status = 2U;
        HAL_Delay(150U);
        HAL_Delay(500U);
        if (servo_read_angle(&angle_tenths)) {
            g_calibration_angle_tenths = angle_tenths;
            g_calibration_status = 3U;
            HAL_Delay(20U);
            servo_stop_hold();
        } else {
            g_calibration_status = 0xE2U;
        }
    } else {
        g_calibration_status = 0xE1U;
    }
#endif


#if SCAN_SERVO_IDS_ON_BOOT
    {
        const uint8_t candidate_ids[] = {0U, 3U, 6U, 9U};
        uint8_t detected_id;
        HAL_Delay(1000U);
        if (servo_ping_broadcast(&detected_id)) {
            g_found_servo_id = detected_id;
            g_calibration_status = 0x10U;
        }
        for (uint8_t i = 0U; i < sizeof(candidate_ids); ++i) {
            if (g_found_servo_id != UINT32_MAX) {
                break;
            }
            if (servo_ping_id(candidate_ids[i])) {
                g_found_servo_id = candidate_ids[i];
                g_calibration_status = 0x10U;
                break;
            }
            HAL_Delay(20U);
        }
        if (g_found_servo_id == UINT32_MAX) {
            g_calibration_status = 0xEFU;
        }
    }
#endif

#if READ_SERVO_ANGLE_ON_BOOT
    {
        int16_t angle_tenths;
        if (servo_read_angle(&angle_tenths)) {
            g_calibration_angle_tenths = angle_tenths;
            g_calibration_status = 0x20U;
        } else {
            g_calibration_status = 0xE3U;
        }
    }
#endif

    /* Normal builds deliberately send no servo command at startup. */
    while (1) {
        uint8_t byte;
        uint8_t mode;
        uint8_t parameter;
        uint32_t now = HAL_GetTick();

        if ((huart1.Instance->SR & USART_SR_RXNE) != 0U) {
            byte = (uint8_t)huart1.Instance->DR;
            if (control_frame_push(byte, &mode, &parameter)) {
                handle_control_command(mode, parameter);
            }
        }

        if (snake_enabled && (now - last_control_ms >= CONTROL_PERIOD_MS)) {
            float logical_offset;
            last_control_ms = now;
            snake_phase += TWO_PI * SNAKE_FREQUENCY_HZ * ((float)CONTROL_PERIOD_MS / 1000.0f);
            if (snake_phase >= TWO_PI) {
                snake_phase -= TWO_PI;
            }
            logical_offset = snake_amplitude_deg * sinf(snake_phase + PCB2_PHASE_OFFSET);
            servo_set_logical_offset(logical_offset);
        }

    }
}

static void GPIO_Init(void) {
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    gpio.Pin = GPIO_PIN_9 | GPIO_PIN_10;
    gpio.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    gpio.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &gpio);
}

static void USART1_Init(void) {
    __HAL_RCC_USART1_CLK_ENABLE();
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 921600;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK) {
        Error_Handler();
    }
}

static void USART2_Init(void) {
    __HAL_RCC_USART2_CLK_ENABLE();
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) {
        Error_Handler();
    }
}

static void SystemClock_Config(void) {
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState = RCC_HSE_ON;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM = 25U;
    osc.PLL.PLLN = 336U;
    osc.PLL.PLLP = RCC_PLLP_DIV2;
    osc.PLL.PLLQ = 7U;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        Error_Handler();
    }

    clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                    RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV4;
    clk.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_5) != HAL_OK) {
        Error_Handler();
    }
}

static void Error_Handler(void) {
    __disable_irq();
    while (1) {
    }
}

void SysTick_Handler(void) {
    HAL_IncTick();
}
