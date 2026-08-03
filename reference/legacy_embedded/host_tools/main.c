#include "system.h"
#include <math.h>
#include "usart.h"
#include <stdio.h>

//#define PI 3.14159265

//Task priority    //?????????
#define START_TASK_PRIO	1

//Task stack size //????????锟斤拷
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
static uint32_t snake_clamp_count = 0;

static float limitServoTargetStep(float target, float last_target, float max_step)
{
    float delta = target - last_target;

    if (delta > max_step) {
        return last_target + max_step;
    }
    if (delta < -max_step) {
        return last_target - max_step;
    }
    return target;
}

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
							(uint16_t       )START_STK_SIZE,        //Task stack size //????????锟斤拷
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
    xTaskCreate(ModeChangeTask, "Mode Change Task", 1024, NULL, 3, NULL);  // ???????锟斤拷?????

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
	 static uint8_t hb_tick = 0;

   while(1)
    {
			vTaskDelayUntil(&lastWakeTime, F2T(RATE_20_HZ));
			processUsart1Data();
			if (++hb_tick >= 200) {
				hb_tick = 0;
				printf("HB tick=%lu snake=%d mode=%u target=%.1f clamp=%lu\r\n",
					(unsigned long)xTaskGetTickCount(),
					(int)snake_mode,
					(unsigned int)snake_mode_type,
					snakeTargetAngle,
					(unsigned long)snake_clamp_count);
			}
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
        RingBuffer_Push(usart2.recvBuf, Usart_Receive);
    }
    return 0;
}


void GAIT_Switch(void *pvParameters)
{
	u32 lastWakeTime = getSysTickCnt();
	static uint8_t last_gait_state = 0xFF;  // ????????????锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷

	while(1)
	{
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_1000_HZ));
		if(Gait_flag && F_count != 1000){
			F_count++;
		}
		if(Gait_flag && F_count == 1000){
			F_count = 0; //???

			// ????????锟斤拷????????
			if(First) //?????????, ?????????
			{
				// 鍙湪鍒濆鍖栨椂璁剧疆涓€娆″熀纭€鍋忕疆
				if(snake_mode) {
					if (SNAKE_NO >= FIRST_SNAKE_JOINT && SNAKE_NO < FIRST_SNAKE_JOINT + NUM_SNAKE_JOINTS) {
						uint8_t joint_index = SNAKE_NO - FIRST_SNAKE_JOINT;
						FSUS_SetServoAngleByVelocity(servo_usart, 0, baseOffset[joint_index], 1000, 1000, 300, 0, 0);
					}
					vTaskDelay(pdMS_TO_TICKS(1000));
					snake_mode = false;
				}

				Now_state = Now_running_table[now_serial];
				First = 0;
			}
			else{
				Last_Serial = now_serial;
				Last_state = Now_state; //????????????锟斤拷?
				now_serial = (now_serial + NP) % 5;
				Now_state = Now_running_table[now_serial];

				// 鍦ㄦ瘡娆＄姸鎬佹洿鏂版椂涔熶繚鎸佽泧褰㈣埖鏈虹殑鍋忕疆浣嶇疆
				if (SNAKE_NO >= FIRST_SNAKE_JOINT && SNAKE_NO < FIRST_SNAKE_JOINT + NUM_SNAKE_JOINTS) {
					uint8_t joint_index = SNAKE_NO - FIRST_SNAKE_JOINT;
					FSUS_SetServoAngleByVelocity(servo_usart, 0, baseOffset[joint_index], 1000, 1000, 300, 0, 0);
				}
			}

			// ??????????????
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
				// ?????????1 - ?????????????????????
				FSUS_SetServoAngleByVelocity(servo_usart, 1, 0, 1000, 1000, 300, 0, 0);

				FSUS_SetServoAngleByVelocity(servo_usart, 2, 180, 1000, 1000, 300, 0, 0);
			}
			if(Now_state == 3){
				// ?????????2 - ????2????????
				FSUS_SetServoAngleByVelocity(servo_usart, 1, 180, 1000, 1000, 300, 0, 0);
				FSUS_SetServoAngleByVelocity(servo_usart, 2, 0, 1000, 1000, 300, 0, 0);
			}

			last_gait_state = Now_state;  // ?????????
		}
	}
}


double snake_phase = 0;
extern float swingAngleLimit;  // 锟斤拷锟斤拷为float锟斤拷锟斤拷
u16 S_count = 0;

// ??????????????????
float turning_radius = 0;  // ????????

// 锟斤拷锟侥硷拷锟斤拷头锟斤拷锟斤拷锟斤拷锟剿讹拷锟斤拷锟斤拷锟斤拷锟斤拷
#define SNAKE_PI 3.14159265359f

// Tuned straight snake baseline:
// phase diff = 60 deg, interval = 55 ms, frequency = 0.9 Hz.
// The 45 deg amplitude remains provided by the upper computer command.
#define SNAKE_FORWARD_GROUP_A_FREQUENCY_HZ 0.9
#define SNAKE_FORWARD_GROUP_A_PHASE_DIFF_RAD (SNAKE_PI / 3.0f)
#define SNAKE_FORWARD_GROUP_A_INTERVAL_MS 55
#define SNAKE_FORWARD_GROUP_A_T_ACC_MS 20
#define SNAKE_FORWARD_GROUP_A_T_DEC_MS 20
// Fixed right-trim for straight snake mode. Reduce if it drifts right; increase if it drifts left.
#define SNAKE_FORWARD_TRIM_SPAN_DEG 8.0f
// Turning is controlled by a differential angle bias, not by a direct radius.
// Larger values make turns tighter; with 4 joints, 15 gives +/-5.625 and +/-1.875 deg biases.
#define SNAKE_TURN_BIAS_SPAN_DEG 15.0f

// #define GAIT_PERIOD 50  // 锟斤拷锟斤拷锟节筹拷锟饺达拷100锟斤拷小锟斤拷50锟斤拷锟斤拷锟绞蛊碉拷始颖锟?

// #define PHASE_DIFF (2.0f * PI / NUM_SNAKE_JOINTS)  // 锟截节硷拷锟斤拷位锟斤拷
// #define MAX_VELOCITY 100.0f  // 锟斤拷锟斤拷锟劫讹拷 (锟斤拷/锟斤拷)

// 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷头锟斤拷锟接匡拷锟斤拷锟斤拷锟节讹拷锟斤拷
// #define SNAKE_CONTROL_PERIOD 50  // 锟斤拷锟斤拷锟斤拷锟斤拷(ms)

// // 锟斤拷锟斤拷锟斤拷锟缴猴拷锟斤拷 - 锟斤拷锟斤拷snake_concertina.py锟斤拷gait锟斤拷锟斤拷
// static float generate_gait(uint8_t i) {
//     i = i % (GAIT_PERIOD + 1);
//     float n = GAIT_PERIOD;

//     // 使锟矫达拷锟斤拷锟斤拷锟揭诧拷锟斤拷去锟斤拷锟斤拷锟剿癸拷锟斤拷锟?
//     float angle = sinf(2.0f * PI * i / n);  // 锟津单碉拷锟斤拷锟揭诧拷
//     return angle;
// }

void GAIT_Switch2Serpentine(void *pvParameters) {
    u32 lastWakeTime = getSysTickCnt();
    static uint8_t last_mode_type = 0xFF;
    static double phase = 0;
    static float last_servo_target = 0.0f;
    static bool last_servo_target_valid = false;

    // 姝ｇ‘瑙ｈ€鍜孲
    static const int R = RATE_20_HZ;        // 鎻愰珮鍒?0Hz锛屾洿绮剧粏鎺у埗
    static const double S = 1000.0/R;          // 姝ラ暱 = 1000/R = 100
    static const double frequency = SNAKE_FORWARD_GROUP_A_FREQUENCY_HZ;
    static double targetAngle = 0;
    static const double phase_step = (1.0 / RATE_20_HZ) * SNAKE_FORWARD_GROUP_A_FREQUENCY_HZ * 2.0 * SNAKE_PI;

    while(1) {
        vTaskDelayUntil(&lastWakeTime, F2T(R));  // 浣跨敤RATE_10_HZ鎺у埗浠诲姟

        if(snake_mode) {

            // 妯″紡鍒囨崲妫€鏌?
            if(last_mode_type != snake_mode_type) {
                // 硬态模式谢
                if(Gait_flag) {
                    FSUS_SetServoAngleByVelocity(servo_usart, 1, 0, 1000, 1000, 300, 0, 0);
                    FSUS_SetServoAngleByVelocity(servo_usart, 2, 0, 1000, 1000, 300, 0, 0);
                    vTaskDelay(pdMS_TO_TICKS(1000));
                }

                // 准位
                if(SNAKE_NO >= FIRST_SNAKE_JOINT && SNAKE_NO < FIRST_SNAKE_JOINT + NUM_SNAKE_JOINTS) {
                    FSUS_SetServoAngleByVelocity(servo_usart, 0, baseOffset[SNAKE_NO - FIRST_SNAKE_JOINT],
                                                1000, 1000, 300, 0, 0);
                }
                vTaskDelay(pdMS_TO_TICKS(1000));

                last_mode_type = snake_mode_type;
                First = 0;
                phase = 0;
                last_servo_target_valid = false;
            }

            // 浣跨敤姝ｇ‘鐨凷杩涜鐩镐綅鏇存柊
            phase -= phase_step;
            if (phase <= -2.0 * SNAKE_PI) {
                phase += 2.0 * SNAKE_PI;
            } else if (phase >= 2.0 * SNAKE_PI) {
                phase -= 2.0 * SNAKE_PI;
            }

            // 鐩存帴杩涜瑙掑害璁＄畻
            if(SNAKE_NO >= FIRST_SNAKE_JOINT && SNAKE_NO < FIRST_SNAKE_JOINT + NUM_SNAKE_JOINTS) {
                uint8_t joint_index = SNAKE_NO - FIRST_SNAKE_JOINT;
                double phase_offset = 0;
                float swing = 0.0f;
                float straight_trim = 0.0f;

                // 涓嶅悓妯″紡鐨勭洰鏍囪搴?
                switch(snake_mode_type) {
                    case SNAKE_MODE_FORWARD:  // 直
                        // 灏濊瘯鏇村皬鐨勭浉浣嶅樊锛毾€/8 = 22.5搴?
                        phase_offset = (NUM_SNAKE_JOINTS - 1 - joint_index) * SNAKE_FORWARD_GROUP_A_PHASE_DIFF_RAD;

                        // 鎴栬€呮洿灏忥細蟺/10 = 18搴?
                        // phase_offset = (NUM_SNAKE_JOINTS - 1 - joint_index) * (PI/10.0f);

                        // 纭畾鍏宠妭鐨勬憜鍔ㄨ搴?
                        swing = swingAngleLimit * sinf(phase + phase_offset);
                        straight_trim = ((NUM_SNAKE_JOINTS - 1 - joint_index) - ((NUM_SNAKE_JOINTS - 1) / 2.0f))
                                      * (SNAKE_FORWARD_TRIM_SPAN_DEG / NUM_SNAKE_JOINTS);
                        targetAngle = baseOffset[joint_index] + swing + straight_trim;

                        // // 涓?2341浣撹妭锛堢涓€涓叧鑺傦級娣诲姞棰濆鍋忕疆
                        // if (SNAKE_NO == 1) {  // 12341瀵瑰簲鐨勬槸绗竴涓叧鑺?
                        //     targetAngle += 5.0f;  // 娣诲姞5搴︾殑鍙冲亸琛ュ伩锛屽彲浠ユ牴鎹疄闄呮儏鍐佃皟鏁磋繖涓€?
                        // }

                        // if(SNAKE_NO == 4){
                        //     targetAngle += 10.0f;
                        // }

                        // targetAngle += 5.0f;
                        // // 闄愬埗鍏宠妭鐨勮搴﹁寖鍥?
                        // float maxAngle = baseOffset[joint_index] + swingAngleLimit;
                        // float minAngle = baseOffset[joint_index] - swingAngleLimit;
                        // targetAngle = (targetAngle > maxAngle) ? maxAngle : targetAngle;
                        // targetAngle = (targetAngle < minAngle) ? minAngle : targetAngle;

                        // 鎵撳嵃鍏宠妭鐨勮搴?
                        break;

                    case SNAKE_MODE_LATERAL:  // 妯悜杩愬姩
                        phase_offset = (NUM_SNAKE_JOINTS - 1 - joint_index) * (PI/3.0f) + PI/2;  // 60位 + 90
                        targetAngle = baseOffset[joint_index] + swingAngleLimit * sinf(phase + phase_offset);
                        break;

                    case SNAKE_MODE_CIRCULAR:
                    case SNAKE_MODE_TURN_RIGHT:  // 鍦嗗懆杩愬姩
                        {
                            float center_index = (NUM_SNAKE_JOINTS-1)/2.0f;
                            float turn_direction = (snake_mode_type == SNAKE_MODE_TURN_RIGHT) ? 1.0f : -1.0f;
                            float turn_bias = turn_direction * ((NUM_SNAKE_JOINTS - 1 - joint_index) - center_index) * (SNAKE_TURN_BIAS_SPAN_DEG / NUM_SNAKE_JOINTS);
                            phase_offset = (NUM_SNAKE_JOINTS - 1 - joint_index) * (PI/3.0f);  // 60位
                            targetAngle = baseOffset[joint_index] + swingAngleLimit * sinf(phase + phase_offset) + turn_bias;
                        }
                        break;
                }

                // 鐩存帴璁剧疆鐩爣瑙掑害
                if (last_servo_target_valid) {
                    float rawTargetAngle = (float)targetAngle;
                    float max_target_step = 2.0f * swingAngleLimit * sinf((float)(phase_step * 0.5));
                    if (max_target_step < 6.0f) {
                        max_target_step = 6.0f;
                    }
                    max_target_step += 3.0f;
                    targetAngle = limitServoTargetStep((float)targetAngle, last_servo_target, max_target_step);
                    if (targetAngle != rawTargetAngle) {
                        snake_clamp_count++;
                        printf("CLAMP raw=%.1f limited=%.1f last=%.1f max=%.1f\r\n",
                            rawTargetAngle,
                            (float)targetAngle,
                            last_servo_target,
                            max_target_step);
                    }
                }
                snakeTargetAngle = targetAngle;
                last_servo_target = (float)targetAngle;
                last_servo_target_valid = true;

                FSUS_SetServoAngleByInterval(servo_usart, 0, targetAngle,
                        SNAKE_FORWARD_GROUP_A_INTERVAL_MS,
                        SNAKE_FORWARD_GROUP_A_T_ACC_MS,
                        SNAKE_FORWARD_GROUP_A_T_DEC_MS,
                        0, 0);

                // 鎵撳嵃鍏宠妭鐨勭洰鏍囪搴?
                // printf("Servo %d: Target Angle = %.2f\n", SNAKE_NO, targetAngle);
                // usart1_send((uint8_t)targetAngle);  // 鍙戦€佺洰鏍囪搴?
            }
        } else {
            last_mode_type = 0xFF;
            phase = 0;
            snakeTargetAngle = 0;
            last_servo_target_valid = false;
        }
    }
}

// ?
void ModeChangeTask(void *pvParameters) {
    ModeChangeMsg msg;
    int gaitNo;  // switch妯″紡

    while(1) {
        if(xQueueReceive(modeChangeQueue, &msg, portMAX_DELAY) == pdTRUE) {
            switch(msg.mode) {
                case 0x01:  // 鍋滄
                    // 鍋滄杩愬姩
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

                case SNAKE_MODE_FORWARD:  // 鐩寸嚎杩愬姩
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

                case SNAKE_MODE_LATERAL:  // 妯悜杩愬姩
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

                case SNAKE_MODE_CIRCULAR:
                    case SNAKE_MODE_TURN_RIGHT:  // 鍦嗗懆杩愬姩
                    if(snake_mode && snake_mode_type != msg.mode) {
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
                    snake_mode_type = msg.mode;
                    Gait_flag = false;
                    First = 1;
                    break;

                default:
                    break;
            }
        }
    }
}
