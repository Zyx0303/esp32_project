# STM32F407 四路AD采集硬件设计指南

## 一、项目背景

- **芯片**: STM32F407ZGT6 @ 168MHz
- **现有外设**: UART1/UART2/UART3, TIM1/2/3/4/8 (PWM输出)
- **需求**: 添加四路AD采集（通道待定）

---

## 二、GPIO选择

### 2.1 ADC可用通道（STM32F407）

STM32F407具有3个ADC单元，共16个外部通道。建议选择**同一ADC单元**的通道，便于同步采样。

| ADC1 通道 | GPIO      | 复用功能 |
|-----------|-----------|----------|
| ADC1_IN0  | PA0       | ADC123_IN0 |
| ADC1_IN1  | PA1       | ADC123_IN1 |
| ADC1_IN2  | PA2       | ADC123_IN2 |
| ADC1_IN3  | PA3       | ADC123_IN3 |
| ADC1_IN4  | PA4       | ADC12_IN4 |
| ADC1_IN5  | PA5       | ADC12_IN5 |
| ADC1_IN6  | PA6       | ADC12_IN6 |
| ADC1_IN7  | PA7       | ADC12_IN7 |
| ADC1_IN8  | PB0       | ADC12_IN8 |
| ADC1_IN9  | PB1       | ADC12_IN9 |
| ADC1_IN10 | PC0       | ADC123_IN10 |
| ADC1_IN11 | PC1       | ADC123_IN11 |
| ADC1_IN12 | PC2       | ADC123_IN12 |
| ADC1_IN13 | PC3       | ADC123_IN13 |
| ADC1_IN14 | PC4       | ADC12_IN14 |
| ADC1_IN15 | PC5       | ADC12_IN15 |

### 2.2 推荐方案

**推荐使用 ADC1_IN10~IN13 (PC0-PC3)**，原因：
- 均为ADC123共享通道，同一ADC单元
- PC0-PC3分布在同一个GPIO端口，便于PCB布局
- 不与现有PWM引脚冲突（PA0-PA7已用于TIM2/TIM3 PWM）
- 采样时间可统一配置

### 2.3 GPIO配置要点

```c
// GPIO初始化配置
GPIO_InitTypeDef GPIO_InitStruct = {0};
// PC0/PC1/PC2/PC3 作为模拟输入
GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;      // 模拟输入模式
GPIO_InitStruct.Pull = GPIO_NOPULL;           // 禁用上下拉
HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
```

**注意**：确保这些引脚不与其他功能复用冲突。

---

## 三、DMA vs 中断 对比

### 3.1 方案对比

| 特性 | DMA模式 | 中断模式 |
|------|---------|----------|
| CPU占用 | 极低 | 中等 |
| 数据实时性 | 整批传输 | 每次转换即处理 |
| 内存占用 | 双缓冲建议 | 单变量即可 |
| 复杂度 | 中等 | 简单 |
| 最高采样率 | 可达1MSPS+ | 受中断开销限制 |
| 可靠性 | 高（硬件传输） | 中（可能丢中断） |

### 3.2 推荐方案

**对于四路AD采集，推荐DMA + 定时器触发模式**：

- 使用TIM3作为触发源（定时器触发ADC转换）
- DMA2_stream0 channel0 用于ADC1
- DMA循环模式，自动将转换结果搬运到内存

```c
// 典型配置
// ADC: 4通道扫描模式，TIM3触发，DMA循环传输
// DMA: Memory半字/全字，循环模式
// TIM3: 定时溢出触发ADC
```

### 3.3 何时用中断

若采样率低于**10kHz**且CPU负载不敏感，可使用中断模式简化实现：

```c
// 中断模式简单示例
HAL_ADC_Start_IT(&hadc1);  // 启动中断模式
// 在HAL_ADC_ConvCpltCallback中处理数据
```

---

## 四、采样率配置

### 4.1 STM32F407 ADC时钟

- ADC时钟源：APB2总线（84MHz最大值）
- 分频系数：ADC_PRESSCALER_PCLK2_DIV2 (42MHz) 或 DIV4/DIV6/DIV8
- 建议使用 **DIV4** 或 **DIV6**，平衡采样速度与精度

### 4.2 采样时间计算

ADC采样时间 = (采样周期 + 12) / ADC时钟频率

| 采样周期 | 总周期 @42MHz | 最大采样率 |
|----------|---------------|------------|
| 3 cycles | 15 | 2.8 MSPS |
| 15 cycles | 27 | 1.5 MSPS |
| 28 cycles | 40 | 1.05 MSPS |
| 84 cycles | 96 | 437 ksPS |
| 480 cycles | 492 | 85 ksPS |

### 4.3 采样率配置示例

**需求：四路AD，每路50kHz采样率**

```
总采样率 = 4 x 50kHz = 200kHz
建议ADC时钟 = 21MHz (DIV4)
采样周期 = 15 cycles
转换时间 = (15+12)/21MHz ≈ 1.29μs
每个通道转换周期 = 21 (15+12-6)  // 6是12-bit转换固定周期
实际采样率 ≈ 1/1.29μs ≈ 775kHz (远满足需求)
```

**定时器触发间隔**：
- 若使用TIM3触发：ARR寄存器设置决定触发频率
- 建议采样率 ≤ 500kHz 以确保数据准确性

### 4.4 推荐配置

```c
// ADC配置
hadc1.Instance = ADC1;
hadc1.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV4;  // 42MHz
hadc1.Init.Resolution = ADC_RESOLUTION_12B;
hadc1.Init.ScanConvMode = ENABLE;        // 扫描模式（多通道）
hadc1.Init.ContinuousConvMode = DISABLE; // 禁用连续转换（定时器触发）
hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO; // TIM3触发
hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
hadc1.Init.NbrOfConversion = 4;           // 4个通道
hadc1.Init.DMAContinuousRequests = ENABLE; // DMA连续请求

// DMA配置
hdma_adc1.Instance = DMA2_Stream0;
hdma_adc1.Init.Channel = DMA_CHANNEL_0;
hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;       // 内存地址递增
hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
hdma_adc1.Init.Mode = DMA_CIRCULAR;            // 循环模式
hdma_adc1.Init.Priority = DMA_PRIORITY_HIGH;
```

---

## 五、滤波电路建议

### 5.1 典型前端电路

```
信号源 --> [R1] --> [C1] --> ADC输入
                |
               GND
```

### 5.2 电阻电容选择

| 信号频率 | R1 | C1 | 用途 |
|----------|----|----|------|
| <1kHz | 10kΩ | 100nF | 低频传感器（电位器等） |
| 1-10kHz | 4.7kΩ | 10nF | 音频/动态信号 |
| >10kHz | 1kΩ | 1nF | 高速信号 |

**ADC输入阻抗**：STM32F407 ADC输入阻抗约 6kΩ，前端电阻不宜过大。

### 5.3 典型应用场景推荐

**场景1：传感器信号（如电位器、压力传感器）**
- R1 = 10kΩ, C1 = 100nF → 截止频率 ~160Hz
- 适合直流或低频信号

**场景2：电机电流采样**
- 使用专用的电流传感芯片（如ACS712）
- 输出已滤波，可直接接入ADC

**场景3：需要快速响应的信号**
- R1 = 1kΩ, C1 = 4.7nF → 截止频率 ~34kHz
- 配合较高的采样率

### 5.4 PCB布局建议

1. ADC输入走线远离高速数字信号
2. 滤波电容尽量靠近MCU引脚
3. 地平面完整，避免走环路
4. 模拟地(AGND)和数字地(DGND)单点连接

---

## 六、需要用户提供的信息清单

### 6.1 必填信息

| 序号 | 问题 | 说明 |
|------|------|------|
| 1 | **四路AD信号类型** | 传感器类型（电位器/电流/电压/其他） |
| 2 | **信号源输出阻抗** | 高阻抗信号需要缓冲电路 |
| 3 | **所需采样率** | 每通道采样频率（Hz） |
| 4 | **信号幅度范围** | 输入电压范围（如0-3.3V、0-5V、±10V） |
| 5 | **是否需要信号调理** | 放大/偏移/滤波等 |
| 6 | **四路信号是否同步** | 同步采样或独立采样 |
| 7 | **ADC数据用途** | 控制回路/数据记录/显示 |

### 6.2 选填信息

| 序号 | 问题 | 说明 |
|------|------|------|
| 8 | **信号带宽** | 最高信号频率 |
| 9 | **精度要求** | 分辨率需求（12bit是否足够） |
| 10 | **硬件接口偏好** | 杜邦线/排针/焊盘 |
| 11 | **是否需要校准** | 零点/满量程校准 |
| 12 | **电源噪声情况** | 是否有开关电源干扰 |

### 6.3 示例反馈格式

```
AD通道信息反馈：
- CH1: 电位器，0-3.3V，阻抗10kΩ，采样率1kHz
- CH2: 压力传感器，0-3.3V，阻抗1kΩ，采样率1kHz
- CH3: 电机电流，0-2.5V（ACS712），采样率2kHz
- CH4: 电池电压监测，0-15V（分压后），采样率100Hz
信号同步：是（同时采样）
数据用途：运动控制闭环
```

---

## 七、软件架构建议

### 7.1 建议目录结构

```
stm32_work/
├── adc/
│   ├── adc.c          # ADC初始化和DMA配置
│   ├── adc.h
│   └── filter.c       # 数字滤波（可选）
├── main.c             # 主循环
└── main.h
```

### 7.2 关键变量

```c
// DMA缓冲区（四路数据循环存储）
#define ADC_BUF_SIZE 64
volatile uint16_t adc_dma_buf[ADC_BUF_SIZE][4];

// 过滤后的数据
float ad_value[4];

// 当前正在处理的半缓冲区索引
volatile uint8_t adc_half_buf_idx = 0;
```

---

## 八、总结

1. **GPIO**: 推荐 PC0-PC3 (ADC123_IN10~IN13)，避免与现有PWM冲突
2. **传输方式**: 推荐DMA+定时器触发，兼顾性能与CPU占用
3. **采样率**: 42MHz ADCclk下可达1MSPS+，满足大多数需求
4. **滤波**: 根据信号带宽选择RC组合，建议10kΩ+100nF起步
5. **关键反馈**: 请用户提供四路信号的具体类型和采样率需求
