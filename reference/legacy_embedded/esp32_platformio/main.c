#include "system.h"
#include <math.h>
#include "usart.h"
#include <stdio.h>

//#define PI 3.14159265

//Task priority    //?????????
#define START_TASK_PRIO	1

//Task stack size //????????С
#define START_STK_SIZE 	256

void GAIT_Switch(void *pvParameters);
void USART_ProcessTask(void *pvParameters);
void data_task(void *pvParameters);
void GAIT_Switch2Serpentine(void *pvParameters);
void ModeChangeTask(void *pvParameters);
void updateSnakeGaitAngles(uint8_t swingAngleLimit);
extern bool snake_mode;

// const double frequency = 0.8;
// Motor Base Offset Array
// const float baseOffset[NUM_SNAKE_JOINTS] = {0.0f, 6.0f, 0.0f, 0.0f};  // ????4???????????1-4
// extern float baseOffset[NUM_SNAKE_JOINTS];
// extern uint8_t SNAKE_NO;
extern uint8_t snake_mode_type;
static double ampl = 0.8;

// double spineOffset = -0.3;
//double controlStep = 600;

extern RingBuffer usart1_ringBuffer;
extern RingBuffer usart2_ringBuffer;
bool First = 1;
int segmentIndex = 1;
double snakeTargetAngle = 0; // ?????????????
//Task handle     //???????????????????????
TaskHandle_t StartTask_Handler;

/* IMU stream read/control struct */
static hipnuc_raw_t hipnuc_raw = {0};

/* 0: no new frame arrived, 1: new frame arrived */
static uint8_t decode_succ = 0;

/* the char buffer use to show result */
static char log_buf[256];

//Task function   //??????
void start_task(void *pvParameters);

//Main function //??????
int main(void)
{
  systemInit(); //Hardware initialization //????????
  memset(&hipnuc_raw, 0, sizeof(hipnuc_raw_t));
  decode_succ = 0;
	//Create the start task //???????????
	xTaskCreate((TaskFunction_t )start_task,            //Task function   //??????
							(const char*    )"start_task",          //Task name       //????????
							(uint16_t       )START_STK_SIZE,        //Task stack size //????????С
							(void*          )NULL,                  //Arguments passed to the task function //????????????????
							(UBaseType_t    )START_TASK_PRIO,       //Task priority   //?????????
							(TaskHandle_t*  )&StartTask_Handler);   //Task handle     //??????
	vTaskStartScheduler();  //Enables task scheduling //???????????
}

//Start task task function //?????????????
void start_task(void *pvParameters)
{
    taskENTER_CRITICAL(); //Enter the critical area //?????????

    // ???????????
    modeChangeQueue = xQueueCreate(5, sizeof(ModeChangeMsg));

    //Create the task //????????
    xTaskCreate(data_task,     "DATA_task",     DATA_STK_SIZE,     NULL, 3,     NULL);
    xTaskCreate(USART_ProcessTask, "USART3 Task", 1024, NULL, 2, NULL);
    xTaskCreate(GAIT_Switch, "GAIT_Switch Task", 1024, NULL,  4, NULL);
    xTaskCreate(GAIT_Switch2Serpentine, "GAIT_Switch2Serpentine Task", 1024, NULL,  5, NULL);
    xTaskCreate(ModeChangeTask, "Mode Change Task", 1024, NULL, 3, NULL);  // ???????л?????

    vTaskDelete(StartTask_Handler); //Delete the start task //??????????

    taskEXIT_CRITICAL();            //Exit the critical section//????????
}

void processUsart1Data(void) {
    while (!RingBuffer_IsEmpty_(&usart1_ringBuffer)) {
        uint8_t data;
        RingBuffer_Get(&usart1_ringBuffer, &data);
		if(Usart1Data_flag){
        usart3_send(data);
        usart1_send(data);}
    }
}

void data_task(void *pvParameters)
{
	 u32 lastWakeTime = getSysTickCnt();

   while(1)
    {
			vTaskDelayUntil(&lastWakeTime, F2T(RATE_20_HZ));
			// printf("run here\n");

			processUsart1Data();
		}
}
static int send = 0;
void processUsart2Data(void) {
    static uint8_t Count = 0;
    static uint8_t rxbuf[256];
    uint8_t index = 4;
    static uint8_t size_packet = 59;
		uint8_t	check_sum = 0;
    while(!RingBuffer_IsEmpty_(&usart2_ringBuffer)) {
        uint8_t Usart_Receive;
        RingBuffer_Get(&usart2_ringBuffer, &Usart_Receive);

         decode_succ = hipnuc_input(&hipnuc_raw, Usart_Receive);
		if(decode_succ == 1){
			decode_succ = 0;
			hipnuc_dump_packet(&hipnuc_raw);
		}
		// printf("OK\n");

	}
}


void USART_ProcessTask(void *pvParameters)
{
	 u32 lastWakeTime = getSysTickCnt();

   while(1)
    {

		vTaskDelayUntil(&lastWakeTime, F2T(RATE_100_HZ));
//				printf("%d \n",send++);
		if (send == 100)
			send = 0;
		// printf("run here\n");
        processUsart2Data();
		// printf("%d\n",decode_succ);
//		 if(decode_succ)
//         {
//             decode_succ = 0;
//             /* convert result to strings */
//             hipnuc_dump_packet(&hipnuc_raw, log_buf, sizeof(log_buf));
//		 	//printf("OK\n");
//		 	 printf("%s\r\n", log_buf);
//		 }
	}
}
int USART2_IRQHandler(void) {
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
        uint8_t Usart_Receive = USART_ReceiveData(USART2);
        RingBuffer_Put(&usart2_ringBuffer, Usart_Receive);
//		printf("%c", Usart_Receive);
    }
    return 0;
}


void GAIT_Switch(void *pvParameters)
{
	u32 lastWakeTime = getSysTickCnt();
	static uint8_t last_gait_state = 0xFF;  // ????????????仯

	while(1)
	{
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_1000_HZ));
		if(Gait_flag && F_count != 1000){
			F_count++;
		}
		if(Gait_flag && F_count == 1000){
			F_count = 0; //???

			// ????????л????????
			if(First) //?????????, ?????????
			{
				// ?????????????л?????????????ζ??????
				if(snake_mode) {
					if (SNAKE_NO >= FIRST_SNAKE_JOINT && SNAKE_NO < FIRST_SNAKE_JOINT + NUM_SNAKE_JOINTS) {
						FSUS_SetServoAngleByVelocity(servo_usart, 0, baseOffset[SNAKE_NO - FIRST_SNAKE_JOINT], 1000, 1000, 300, 0, 0);
					}
					vTaskDelay(pdMS_TO_TICKS(1000));
					snake_mode = false;
				}

				// ??????????
				Now_state = Now_running_table[now_serial];
				First = 0;
			}
			else{
				Last_Serial = now_serial;
				Last_state = Now_state; //????????????ε?
				now_serial = (now_serial + NP) % 5;
				Now_state = Now_running_table[now_serial];
			}

			// ??е????????????
			if(Now_state == 0){
				// ?????(0??) - ?????????
				FSUS_SetServoAngleByVelocity(servo_usart, 1, 0, 1000, 1000, 300, 0, 0);
				FSUS_SetServoAngleByVelocity(servo_usart, 2, 0, 1000, 1000, 300, 0, 0);
			}
			if(Now_state == 1){
				// ??????(180??) - ??????????
				FSUS_SetServoAngleByVelocity(servo_usart, 1, 180, 1000, 1000, 300, 0, 0);
				FSUS_SetServoAngleByVelocity(servo_usart, 2, 180, 1000, 1000, 300, 0, 0);
			}
			if(Now_state == 2){
				// ?м??????1 - ?????????????????????
				FSUS_SetServoAngleByVelocity(servo_usart, 1, 0, 1000, 1000, 300, 0, 0);
				FSUS_SetServoAngleByVelocity(servo_usart, 2, 180, 1000, 1000, 300, 0, 0);
			}
			if(Now_state == 3){
				// ?м??????2 - ????2????????
				FSUS_SetServoAngleByVelocity(servo_usart, 1, 180, 1000, 1000, 300, 0, 0);
				FSUS_SetServoAngleByVelocity(servo_usart, 2, 0, 1000, 1000, 300, 0, 0);
			}

			last_gait_state = Now_state;  // ?????????
		}
	}
}


double snake_phase = 0;
extern float swingAngleLimit;  // 声明为float类型
u16 S_count = 0;

// ??????????????????
float turning_radius = 0;  // ????????

// 在文件开头添加运动参数定义
#define PI 3.14159265359f
#define PHASE_DIFF (4.0f * PI / (3.0f * NUM_SNAKE_JOINTS))  // 关节间相位差
// #define GAIT_PERIOD 50  // 将周期长度从100减小到50，这会使频率加倍

// #define PHASE_DIFF (2.0f * PI / NUM_SNAKE_JOINTS)  // 关节间相位差
// #define MAX_VELOCITY 100.0f  // 最大角速度 (度/秒)

// 在文件开头添加控制周期定义
// #define SNAKE_CONTROL_PERIOD 50  // 控制周期(ms)

// // 波形生成函数 - 基于snake_concertina.py的gait函数
// static float generate_gait(uint8_t i) {
//     i = i % (GAIT_PERIOD + 1);
//     float n = GAIT_PERIOD;

//     // 使用纯正弦波，去掉高斯包络
//     float angle = sinf(2.0f * PI * i / n);  // 简单的正弦波
//     return angle;
// }

void GAIT_Switch2Serpentine(void *pvParameters) {
    u32 lastWakeTime = getSysTickCnt();
    static uint8_t last_mode_type = 0xFF;
    static double phase = 0;
    static const double frequency = 0.5;     // 降低到0.5Hz，即2秒一个周期
    static const double controlStep = 100;   // 增加到100ms控制周期
    static double targetAngle = 0;          // 当前舵机的目标角度

    while(1) {
        vTaskDelayUntil(&lastWakeTime, F2T(RATE_20_HZ));

        if(snake_mode) {
            // 模式变化检测和初始化
            if(last_mode_type != snake_mode_type) {
                // 从步态模式切换处理
                if(Gait_flag) {
                    FSUS_SetServoAngleByVelocity(servo_usart, 1, 0, 1000, 1000, 300, 0, 0);
                    FSUS_SetServoAngleByVelocity(servo_usart, 2, 0, 1000, 1000, 300, 0, 0);
                    vTaskDelay(pdMS_TO_TICKS(1000));
                }

                // 到基准位置
                if(SNAKE_NO >= FIRST_SNAKE_JOINT && SNAKE_NO < FIRST_SNAKE_JOINT + NUM_SNAKE_JOINTS) {
                    FSUS_SetServoAngleByVelocity(servo_usart, 0, baseOffset[SNAKE_NO - FIRST_SNAKE_JOINT],
                                                1000, 1000, 300, 0, 0);
                }
                vTaskDelay(pdMS_TO_TICKS(1000));

                last_mode_type = snake_mode_type;
                First = 0;
                phase = 0;
            }

            // 更新相位
            phase -= (double)controlStep / 1000.0 * frequency * 2.0 * PI;

            // 计算当前舵机的相位差和目标角度
            if(SNAKE_NO >= FIRST_SNAKE_JOINT && SNAKE_NO < FIRST_SNAKE_JOINT + NUM_SNAKE_JOINTS) {
                uint8_t joint_index = SNAKE_NO - FIRST_SNAKE_JOINT;
                double phase_offset = 0;

                // 根据不同模式计算目标角度
                switch(snake_mode_type) {
                    case SNAKE_MODE_FORWARD:  // 直线运动
                        phase_offset = (NUM_SNAKE_JOINTS - 1 - joint_index) * (PI/4.0f);
                        // 确保不超过设定的摆动角度
                        float swing = swingAngleLimit * sinf(phase + phase_offset);
                        targetAngle = baseOffset[joint_index] + swing;

                        // 添加角度限制
                        float maxAngle = baseOffset[joint_index] + swingAngleLimit;
                        float minAngle = baseOffset[joint_index] - swingAngleLimit;
                        targetAngle = (targetAngle > maxAngle) ? maxAngle : targetAngle;
                        targetAngle = (targetAngle < minAngle) ? minAngle : targetAngle;

                        // 调试打印
                        printf("Joint %d: base=%.1f swing=%.1f target=%.1f\n",
                               joint_index, baseOffset[joint_index], swing, targetAngle);
                        break;

                    case SNAKE_MODE_LATERAL:  // 侧向运动
                        phase_offset = (NUM_SNAKE_JOINTS - 1 - joint_index) * (PI/3.0f) + PI/2;  // 60度相位差 + 90度
                        targetAngle = baseOffset[joint_index] + swingAngleLimit * sinf(phase + phase_offset);
                        break;

                    case SNAKE_MODE_CIRCULAR:  // 圆周运动
                        {
                            float center_index = (NUM_SNAKE_JOINTS-1)/2.0f;
                            float turn_bias = ((NUM_SNAKE_JOINTS - 1 - joint_index) - center_index) * (20.0f / NUM_SNAKE_JOINTS);
                            phase_offset = (NUM_SNAKE_JOINTS - 1 - joint_index) * (PI/3.0f);  // 60度相位差
                            targetAngle = baseOffset[joint_index] + swingAngleLimit * sinf(phase + phase_offset) + turn_bias;
                        }
                        break;
                }

                // // 限制角度范围
                // targetAngle = (targetAngle > baseOffset[joint_index] + swingAngleLimit) ?
                //              baseOffset[joint_index] + swingAngleLimit : targetAngle;
                // targetAngle = (targetAngle < baseOffset[joint_index] - swingAngleLimit) ?
                //              baseOffset[joint_index] - swingAngleLimit : targetAngle;


                /* // 注释掉平滑过渡部分
                double tempAngle = 0;
                for(int i = 1; i <= 10; i++) {
                    tempAngle = i*(targetAngle[joint_index]-preTargetAngle[joint_index])/10 + preTargetAngle[joint_index];
                    FSUS_SetServoAngleByVelocity(servo_usart, 0, tempAngle, 100, 100, 300, 0, 0);
                    vTaskDelay(pdMS_TO_TICKS(snakeControlPeriod/10));
                }
                */
                // 直接设置目标角度
                FSUS_SetServoAngleByVelocity(servo_usart, 0, targetAngle, 100, 100, 300, 0, 0);

                // 打印当前舵机角度
                // printf("Servo %d: Target Angle = %.2f\n", SNAKE_NO, targetAngle);
                // usart1_send((uint8_t)targetAngle);  // 通过串口发送角度
            }
        } else {
            last_mode_type = 0xFF;
            phase = 0;
        }
    }
}

// ?
void ModeChangeTask(void *pvParameters) {
    ModeChangeMsg msg;
    int gaitNo;  // ?switch??

    while(1) {
        if(xQueueReceive(modeChangeQueue, &msg, portMAX_DELAY) == pdTRUE) {
            switch(msg.mode) {
                case 0x01:  // 沽?
                    // ζ???
                    if(snake_mode) {
                        if (SNAKE_NO >= FIRST_SNAKE_JOINT && SNAKE_NO < FIRST_SNAKE_JOINT + NUM_SNAKE_JOINTS) {
                            FSUS_SetServoAngleByVelocity(servo_usart, 0, baseOffset[SNAKE_NO - FIRST_SNAKE_JOINT], 1000, 1000, 300, 0, 0);
                        } else {
                            FSUS_SetServoAngleByVelocity(servo_usart, 0, 0, 1000, 1000, 300, 0, 0);
                        }
                        delay_ms(1000);
                    }

                    gaitNo = msg.parameter - 1;
                    memcpy(Now_running_table, Gait_table[gaitNo], 5);
                    NP = Gait_table[gaitNo][5];
                    Gait_flag = true;
                    First = 1;
                    snake_mode = false;
                    break;

                case SNAKE_MODE_FORWARD:  // 直线运动
                    if(Gait_flag) {
                        FSUS_SetServoAngleByVelocity(servo_usart, 1, 0, 1000, 1000, 300, 0, 0);
                        FSUS_SetServoAngleByVelocity(servo_usart, 2, 0, 1000, 1000, 300, 0, 0);
                        delay_ms(1000);
                    }

                    snake_mode = true;
                    snake_mode_type = SNAKE_MODE_FORWARD;
                    Gait_flag = false;
                    First = 1;
                    break;

                case SNAKE_MODE_LATERAL:  // 侧向运动
                    if(snake_mode && snake_mode_type != SNAKE_MODE_LATERAL) {
                        if (SNAKE_NO >= FIRST_SNAKE_JOINT && SNAKE_NO < FIRST_SNAKE_JOINT + NUM_SNAKE_JOINTS) {
                            FSUS_SetServoAngleByVelocity(servo_usart, 0, baseOffset[SNAKE_NO - FIRST_SNAKE_JOINT], 1000, 1000, 300, 0, 0);
                        } else {
                            FSUS_SetServoAngleByVelocity(servo_usart, 0, 0, 1000, 1000, 300, 0, 0);
                        }
                        delay_ms(1000);
                    }
                    if(Gait_flag) {
                        FSUS_SetServoAngleByVelocity(servo_usart, 1, 0, 1000, 1000, 300, 0, 0);
                        FSUS_SetServoAngleByVelocity(servo_usart, 2, 0, 1000, 1000, 300, 0, 0);
                        delay_ms(1000);
                    }

                    snake_mode = true;
                    snake_mode_type = SNAKE_MODE_LATERAL;
                    Gait_flag = false;
                    First = 1;
                    break;

                case SNAKE_MODE_CIRCULAR:  // 圆周运动
                    if(snake_mode && snake_mode_type != SNAKE_MODE_CIRCULAR) {
                        if (SNAKE_NO >= FIRST_SNAKE_JOINT && SNAKE_NO < FIRST_SNAKE_JOINT + NUM_SNAKE_JOINTS) {
                            FSUS_SetServoAngleByVelocity(servo_usart, 0, baseOffset[SNAKE_NO - FIRST_SNAKE_JOINT], 1000, 1000, 300, 0, 0);
                        } else {
                            FSUS_SetServoAngleByVelocity(servo_usart, 0, 0, 1000, 1000, 300, 0, 0);
                        }
                        delay_ms(1000);
                    }
                    if(Gait_flag) {
                        FSUS_SetServoAngleByVelocity(servo_usart, 1, 0, 1000, 1000, 300, 0, 0);
                        FSUS_SetServoAngleByVelocity(servo_usart, 2, 0, 1000, 1000, 300, 0, 0);
                        delay_ms(1000);
                    }

                    snake_mode = true;
                    snake_mode_type = SNAKE_MODE_CIRCULAR;
                    Gait_flag = false;
                    First = 1;
                    break;

                default:
                    break;
            }
        }
    }
}
