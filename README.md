# ESP32-S3 机器人控制器

## 硬件配置

### 开发板
- **芯片**: ESP32-S3 (QFN56, revision v0.2)
- **型号**: S3-N16R8 (WiFi + BT)
- **Flash**: 16MB
- **PSRAM**: 8MB
- **晶振**: 40MHz
- **蓝牙**: 内置 BLE 5.0
- **MAC**: 3c:0f:02:d6:e1:c0

### 引脚定义

| 功能 | GPIO | 说明 |
|------|------|------|
| I2C SCL | 11 | MPU6050 时钟 |
| I2C SDA | 12 | MPU6050 数据 |
| MPU6050 INT | 8 | 数据就绪中断 |
| DRV8833 nSLEEP | 3 | 电机驱动睡眠控制 |
| DRV8833 nFAULT | 4 | 故障检测输入 |
| DRV8833 AIN1 | 46 | 电机A控制；GPIO46仅输入，需硬件改线 |
| DRV8833 AIN2 | 9 | 电机A控制/PWM |
| DRV8833 BIN1 | 5 | 电机B控制/PWM |
| DRV8833 BIN2 | 6 | 电机B控制/PWM |
| 舵机 PWM | 45 | 50Hz PWM 输出 |

### I2C 配置
- **地址**: MPU6050 = **0x68** (AD0 接地)
- **频率**: 400kHz

## 功能模块

### 1. BLE 蓝牙控制
- **设备名称**: ESP32-Robot
- **Service UUID**: 0xFF00
- **Characteristics**:
  - 0xFF01: 电机速度 (0-100%)
  - 0xFF02: 舵机角度 (0-180°)

### 2. 电机驱动 (DRV8833)
- **PWM频率**: 16kHz
- **PWM分辨率**: 10-bit (1024级)
- **控制方式**: 双H桥

### 3. 舵机控制
- **频率**: 50Hz
- **脉宽范围**: 500μs - 2500μs
- **角度范围**: 0° - 180°

### 4. IMU (MPU6050)
- **加速度计**: ±2g
- **陀螺仪**: ±250°/s
- **中断模式**: 下降沿触发

## 构建和烧录

### 构建
```powershell
# 先激活 ESP-IDF 环境，再在项目目录执行
idf.py set-target esp32s3
idf.py build
```

### 烧录
```powershell
idf.py -p COM7 flash
```

## 文件结构

```
esp32_project/
├── main/
│   ├── app_main.c      # 主程序入口
│   └── CMakeLists.txt  # 主组件配置
├── esp32_s3/          # 板级驱动组件
│   ├── board_pins.c/h  # 引脚定义
│   ├── drv8833.c/h    # 电机驱动
│   ├── servo_pwm.c/h   # 舵机控制
│   ├── mpu6050.c/h    # IMU驱动
│   ├── flash_storage.c/h # Flash存储
│   ├── ble_control.c/h  # 蓝牙控制
│   └── CMakeLists.txt
├── partitions.csv       # 分区表
├── sdkconfig          # ESP-IDF 配置
└── sdkconfig.defaults # 默认配置
```

## 组件配置

### esp32_s3/CMakeLists.txt
```cmake
idf_component_register(
    SRCS
        "board_pins.c"
        "mpu6050.c"
        "drv8833.c"
        "servo_pwm.c"
        "flash_storage.c"
        "ble_control.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_gpio esp_driver_i2c esp_driver_ledc esp_timer nvs_flash bt
)
```

### main/CMakeLists.txt (完整功能)
```cmake
idf_component_register(
    SRCS "app_main.c"
    INCLUDE_DIRS "."
    REQUIRES nvs_flash esp_driver_ledc esp_driver_gpio esp_driver_i2c esp_timer bt esp32_s3
)
```

## 串口监视

```powershell
idf.py -p COM7 monitor
```

## 测试检查清单

- [ ] 串口输出正常 (115200 baud)
- [ ] BLE 设备可被发现 (ESP32-Robot)
- [ ] 电机 PWM 输出正常
- [ ] 舵机角度控制正常
- [ ] I2C 设备响应
- [ ] MPU6050 数据读取

## 已知硬件限制

最新原理图将 DRV8833 的 AIN1 连接到了 ESP32-S3 的 GPIO46。GPIO46
仅支持输入，不能输出电机控制信号。固件检测到这一映射后会保持
DRV8833 的 nSLEEP 为低电平，防止电机误动作；需要将 AIN1 飞线到一个
可输出 GPIO，并同步修改 `board_pins.h`，才能启用双路电机。

## 技术栈

- ESP-IDF v5.3.1
- FreeRTOS
- Bluedroid (BLE)
- LEDC (PWM)
- I2C Master

## 版本

- 构建时间: 2026-05-26
- ESP-IDF: v5.3.1-dirty
