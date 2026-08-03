#include "usartx.h"

#define TOTAL_LENGTH 5

uint8_t Gait_table[8][6] = {
    {1,1,0,0,0,1},  // 步态0: 基础蠕虫步态 - 前两节收缩(1)，后三节伸展(0)，每次前进1步(NP=1)
                     // 适用于稳定爬行，类似毛毛虫的基本运动模式

    {1,1,1,0,0,1},  // 步态1: 增强推进步态 - 前三节收缩，后两节伸展，每次前进1步
                     // 比步态0提供更强的推进力，但保持相同的移动速度

    {1,1,1,1,0,1},  // 步态2: 最大收缩步态 - 前四节收缩，仅末节伸展，每次前进1步
                     // 提供最大推进力，适用于需要强力推进的场景（如上坡）

    {1,1,1,0,0,2},  // 步态3: 快速运动步态 - 前三节收缩，后两节伸展，每次前进2步(NP=2)
                     // 通过增加步进距离(NP=2)实现更快的移动速度，但可能降低稳定性

    {1,1,2,3,0,1},  // 步态4: 渐进过渡步态 - 从前到后逐渐过渡(1->1->2->3->0)，每次前进1步
                     // 使用中间状态(2,3)实现平滑运动，减少机械冲击

    {1,1,2,2,0,1},  // 步态5: 双级过渡步态 - 前两节完全收缩，中间两节半收缩，末节伸展
                     // 提供更平缓的运动过渡，适合需要稳定性的场景

    {1,1,3,3,0,1},  // 步态6: 另一种过渡步态 - 类似步态5但使用不同的中间状态值
                     // 通过调整中间状态值(3)提供不同的运动特性

    {0,0,0,0,0,0}   // 步态7: 停止/复位步态 - 所有节段伸展，停止运动(NP=0)
                     // 用于系统初始化或停止运动
};

uint8_t Now_running_table[5];
uint8_t Now_state = 0;
uint8_t NP = 0;
uint8_t now_serial = Segment_No;

bool snake_mode = false;

extern double snakeTargetAngle;


uint8_t Last_state;
uint8_t Last_Serial;

bool Usart1Data_flag = 0;
bool Gait_flag = 0;

float bigEndianToFloat(const uint8_t *data) {
    union {
        uint8_t bytes[4];
        float value;
    } converter;
    converter.bytes[3] = data[0];
    converter.bytes[2] = data[1];
    converter.bytes[1] = data[2];
    converter.bytes[0] = data[3];

    return converter.value;
}

void RingBuffer_Init_(RingBuffer *rb) {
    rb->head = 0;
    rb->tail = 0;
}

bool RingBuffer_Put(RingBuffer *rb, uint8_t data) {
    if (((rb->head + 1) % RING_BUFFER_SIZE) == rb->tail) {
        // Buffer is full
        return false;
    }
    rb->buffer[rb->head] = data;
    rb->head = (rb->head + 1) % RING_BUFFER_SIZE;
    return true;
}

bool RingBuffer_Get(RingBuffer *rb, uint8_t *data) {
    if (rb->head == rb->tail) {
        // Buffer is empty
        return false;
    }
    *data = rb->buffer[rb->tail];
    rb->tail = (rb->tail + 1) % RING_BUFFER_SIZE;
    return true;
}

bool RingBuffer_IsEmpty_(RingBuffer *rb) {
    return rb->head == rb->tail;
}


RingBuffer usart1_ringBuffer;
RingBuffer usart2_ringBuffer;


extern int Time_count;




void USART1_SEND(uint8_t* dataArray, int size) {
    for(int i = 0; i < size; i++) {
        usart1_send(dataArray[i]);
    }
}


void USART3_SEND(uint8_t* dataArray, int size) {
    for(int i = 0; i < size; i++) {
        usart3_send(dataArray[i]);
    }
}

/**************************************************************************
Function: Serial port 1 initialization
Input   : none
Output  : none
函数功能：串口1初始化
入口参数：无
返 回 值：无
**************************************************************************/
void uart1_init(u32 bound)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	 //Enable the gpio clock //使能GPIO时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); //Enable the Usart clock //使能USART时钟

	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10 ,GPIO_AF_USART1);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;            //输出模式
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //推挽输出
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;       //高速50MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //上拉
	GPIO_Init(GPIOA, &GPIO_InitStructure);  		          //初始化

  //UsartNVIC configuration //UsartNVIC配置
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	//Preempt priority //抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1 ;
	//Subpriority //子优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	//Enable the IRQ channel //IRQ通道使能
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  //Initialize the VIC register with the specified parameters
	//根据指定的参数初始化VIC寄存器
	NVIC_Init(&NVIC_InitStructure);

  //USART Initialization Settings 初始化设置
	USART_InitStructure.USART_BaudRate = bound; //Port rate //串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; //The word length is 8 bit data format //字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1; //A stop bit //一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No; //Prosaic parity bits //无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //No hardware data flow control //无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//Sending and receiving mode //收发模式
	USART_Init(USART1, &USART_InitStructure); //Initialize serial port 1 //初始化串口1

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); //Open the serial port to accept interrupts //开启串口接受中断
	USART_Cmd(USART1, ENABLE);                     //Enable serial port 1 //使能串口1
}
/**************************************************************************
Function: Serial port 3 initialization
Input   : none
Output  : none
函数功能：串口3初始化
入口参数：无
返回  值：无
**************************************************************************/
void uart3_init(u32 bound)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);	 //Enable the gpio clock  //使能GPIO时钟
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); //Enable the Usart clock //使能USART时钟

	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;            //输出模式
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //推挽输出
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;       //高速50MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //上拉
	GPIO_Init(GPIOB, &GPIO_InitStructure);  		          //初始化

  //UsartNVIC configuration //UsartNVIC配置
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	//Preempt priority //抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;
	//Preempt priority //抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	//Enable the IRQ channel //IRQ通道使能
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  //Initialize the VIC register with the specified parameters
	//根据指定的参数初始化VIC寄存器
	NVIC_Init(&NVIC_InitStructure);

  //USART Initialization Settings 初始化设置
	USART_InitStructure.USART_BaudRate = bound; //Port rate //串口??特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; //The word length is 8 bit data format //字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1; //A stop bit //一个停止
	USART_InitStructure.USART_Parity = USART_Parity_No; //Prosaic parity bits //无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //No hardware data flow control //无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//Sending and receiving mode //收发模式
  USART_Init(USART3, &USART_InitStructure);      //Initialize serial port 3 //初始化串口3

  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE); //Open the serial port to accept interrupts //开启串口接受中断
  USART_Cmd(USART3, ENABLE);                     //Enable serial port 3 //使能串口3
}

extern u16 F_count;
extern bool First;
#define flag_start			1
#define notStart(flag) 		(!(flag & flag_start))
#define flag_imu			(1<<1)
#define flag_worm			(1<<2)
#define isWormMode(flag)	(flag & flag_worm)
#define flag_snake			(1<<3)
#define isSnakeMode(flag)	(flag & flag_snake)
// int USART1_IRQHandler(void)
// {
// 	static uint8_t cxx_out = 0;
// 	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) //Check if data is received //判断是否接收到数据
// 	{
// 		uint8_t data;
// 			uint8_t Usart_Receive = USART_ReceiveData(USART1);
// 			RingBuffer_Put(&usart1_ringBuffer, Usart_Receive);


// 			if(Usart_Receive == 0x88)
// 			{

// 				RingBuffer_Get(&usart1_ringBuffer, &data);

// 					cxx_out++;
// 			}
// 			else if(Usart_Receive == 0x77 && cxx_out == 1)
// 			{ //开启IMU配置
// 				Usart1Data_flag = !Usart1Data_flag;
// 				RingBuffer_Get(&usart1_ringBuffer, &data);
// //				printf("Usart1\2 start: %d\n",Usart1Data_flag);
// 				cxx_out = 0;
// 			}
// 			else if(Usart_Receive == 0x78 && cxx_out == 1)
// 			{
// 				RingBuffer_Get(&usart1_ringBuffer, &data);
// 				Gait_flag = false;
// 				F_count = 1000;
// 				cxx_out++;
// 			} //开启步态配置
// 			if(cxx_out == 2 && Usart_Receive != 0x78)
// 			{//根据第三帧的数据来选择进行第几个步
// 				cxx_out = 0;
// 				RingBuffer_Get(&usart1_ringBuffer, &data);
// 				First = 1;
// 				int gaitNo = (int)Usart_Receive;
// 				for(uint8_t i = 0; i < TOTAL_LENGTH; i++){
// 					Now_running_table[i] =  Gait_table[gaitNo][i];}
// 				now_serial = Segment_No;
// 				NP = Gait_table[Usart_Receive][TOTAL_LENGTH];
// 				Now_state = Now_running_table[Segment_No];
// 				Gait_flag = true;
// 			}
// 			if() //定义受指令
// 			{
// 				snakeTargetAngle
// 			}
//   }
//   return 0;
// }

#define FrameHead1 0xFF
#define FrameHead2 0xFA
#define FrameTail1 0x88
#define FrameTail2 0x77



float swingAngleLimit = 0;  // 定义为float类型
//指令
int USART1_IRQHandler(void) {
    static uint8_t frame_buffer[messageEnd] = {0};
    static uint8_t frame_index = 0;

    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        uint8_t Usart_Receive = USART_ReceiveData(USART1);
        RingBuffer_Put(&usart1_ringBuffer, Usart_Receive);

        frame_buffer[frame_index++] = Usart_Receive;

        if (frame_buffer[head1] != FrameHead1) {
            frame_index = 0;
            return 0;
        }

        if (frame_buffer[head2] != FrameHead2) {
            frame_index = 1;
            return 0;
        }

        if (frame_buffer[tail1] != FrameTail1 || frame_buffer[tail2] != FrameTail2) {
            return 0;
        }

        switch (frame_buffer[mode]) {
            case 0x01:  // 蠕虫步态
                {
                    if(snake_mode) {
                        if (SNAKE_NO >= FIRST_SNAKE_JOINT && SNAKE_NO < FIRST_SNAKE_JOINT + NUM_SNAKE_JOINTS) {
                            FSUS_SetServoAngleByVelocity(servo_usart, 0, baseOffset[SNAKE_NO - FIRST_SNAKE_JOINT], 1000, 1000, 300, 0, 0);
                        }
                    }

                    int gaitNo = frame_buffer[parameter] - 1;
                    memcpy(Now_running_table, Gait_table[gaitNo], 5);
                    NP = Gait_table[gaitNo][5];
                    Gait_flag = true;
                    First = 1;
                    snake_mode = false;
                }
                break;

            case SNAKE_MODE_FORWARD:  // 0x02 直线运动
                snake_mode = true;
                snake_mode_type = SNAKE_MODE_FORWARD;
                swingAngleLimit = (frame_buffer[parameter] * 90.0f) / 255.0f;  // 需要转换回角度值
                Gait_flag = false;
                First = 1;
                break;

            case SNAKE_MODE_LATERAL:  // 0x03 侧向运动
                snake_mode = true;
                snake_mode_type = SNAKE_MODE_LATERAL;
                swingAngleLimit = (frame_buffer[parameter] * 90.0f) / 255.0f;  // 直接使用角度值
                Gait_flag = false;
                First = 1;
                break;

            case SNAKE_MODE_CIRCULAR:  // 0x04 圆周运动
                snake_mode = true;
                snake_mode_type = SNAKE_MODE_CIRCULAR;
                swingAngleLimit = (frame_buffer[parameter] * 90.0f) / 255.0f;  // 直接使用角度值
                Gait_flag = false;
                First = 1;
                break;
        }

        // 清空缓冲区
        for (int i = 0; i < messageEnd; i++) {
            frame_buffer[i] = 0;
        }
        frame_index = 0;
    }
    return 0;
}


MTIData result;


/**************************************************************************
Function: Serial port 1 sends data
Input   : The data to send
Output  : none
函数功能：串口1发送数据
入口参数：要发送的数据
返回  值：无
**************************************************************************/
void usart1_send(uint8_t data) {
    USART1->DR = data;
    while((USART1->SR & 0x40) == 0);  // 等待发送完成
}
/**************************************************************************
Function: Serial port 2 sends data
Input   : The data to send
Output  : none
函数功能：串口2发送数据
入口参数：要发送的数据
返回  值：无
**************************************************************************/
void usart2_send(u8 data)
{
	USART2->DR = data;
	while((USART2->SR&0x40)==0);
}
/**************************************************************************
Function: Serial port 3 sends data
Input   : The data to send
Output  : none
函数功能：串口3发送数据
入口参数：要发送的数据
返回  值：无
**************************************************************************/
void usart3_send(u8 data)
{
	USART3->DR = data;
	while((USART3->SR&0x40)==0);
}

// 在源文件中定义全局量
// uint8_t SNAKE_NO = 4;  // 删除这行
uint8_t snake_mode_type = SNAKE_MODE_FORWARD;  // 当前蛇形运动模式
float baseOffset[NUM_SNAKE_JOINTS] = {3.0f, 2.0f, 28.0f, -40.0f};  // 体节1-4的偏置

// 创建消息队列句柄
QueueHandle_t modeChangeQueue = NULL;
