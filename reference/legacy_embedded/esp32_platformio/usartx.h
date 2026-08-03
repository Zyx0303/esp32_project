#ifndef __USRATX_H
#define __USRATX_H

#include "stdio.h"
#include "sys.h"
#include "system.h"
#include <stdbool.h>

#define Segment_No 4    // 蠕虫运动中使用的节段数量
#define SNAKE_NO 4  // 当前体节编号(0-4)



#define USART_RECV_BUF_SIZE 2048
#define USART_SEND_BUF_SIZE 2048

#define DATA_STK_SIZE   512
#define DATA_TASK_PRIO  4

#define IMU_STK_SIZE   512
#define IMU_TASK_PRIO  2

#define MTI_HEADER_1 0xFA
#define MTI_HEADER_2 0xFF
#define MTI_PROTOCOL 0x36
#define QUATERNION_ID 0x2010
#define ACCELERATION_ID 0x4020
#define GYROSCOPE_ID 0x8020

#define RING_BUFFER_SIZE 4000

#define RING_BUFFER_SIZE 4000
#define TOTAL_LENGTH 5

#define NUM_SNAKE_JOINTS 4  // 4个蛇形舵机
#define FIRST_SNAKE_JOINT 1  // 第一个蛇形舵机从体节1开始
extern float baseOffset[NUM_SNAKE_JOINTS];  // 每个蛇形舵机的基础偏置角度
extern uint8_t snake_mode_type;  // 当前蛇形运动模式

extern bool Usart1Data_flag;
extern bool Gait_flag;
extern uint8_t Gait_table[8][6];  // 8种步态，每种步态6个参数(5个节段状态 + 1个NP值)
extern uint8_t Now_running_table[5];  // 当前运行的步态状态
extern uint8_t Now_state;  // 当前节段状态
extern uint8_t NP;  // Next Position - 下一步移动的步数
extern uint8_t now_serial;  // 当前执行到的节段序号

extern uint8_t Last_state;  // 上一个状态
extern uint8_t Last_Serial;  // 上一个节段序号

typedef struct {
    float q0, q1, q2, q3;        // Quaternion
    float accX, accY, accZ;     // Acceleration
    float gyrX, gyrY, gyrZ;     // Gyroscope (Rate of Turn)
} MTIData;


typedef struct {
    uint8_t buffer[RING_BUFFER_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} RingBuffer;

void RingBuffer_Init_(RingBuffer *rb);
bool RingBuffer_Put(RingBuffer *rb, uint8_t data);
bool RingBuffer_Get(RingBuffer *rb, uint8_t *data);
bool RingBuffer_IsEmpty_(RingBuffer *rb);

void USART1_SEND(uint8_t* dataArray, int size);
void USART3_SEND(uint8_t* dataArray, int size);

void uart1_init(u32 bound);
void uart3_init(u32 bound);

int USART1_IRQHandler(void);
//int USART3_IRQHandler(void);


void usart1_send(u8 data);
void usart2_send(u8 data);
void usart3_send(u8 data);

float bigEndianToFloat(const uint8_t *data);

//void processUsart3Data(void);

// snake

// 保持原有的通信协议格式
enum Message {
    head1 = 0,
    head2,
    mode,       // 运动模式: 0x01蠕虫步态, 0x02直线, 0x03侧行, 0x04圆周
    parameter,  // 参数: 对于蠕虫是步态号，对于蛇形是振幅
    tail1,
    tail2,
    messageEnd
};

// 定义蛇形运动模式
#define SNAKE_MODE_FORWARD  0x02  // 直线前进模式
#define SNAKE_MODE_LATERAL  0x03  // 侧向运动模式
#define SNAKE_MODE_CIRCULAR 0x04  // 圆周运动模式

#endif
