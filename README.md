# ESP32-S3 机器人控制器

基于 ESP-IDF 的双电机、舵机和 MPU6050 控制固件。无线控制使用 Wi-Fi，固件不再启用蓝牙。

## 硬件

- 主控：ESP32-S3-WROOM-1-N16R8（16 MB Flash、8 MB PSRAM）
- 电机驱动：DRV8833
- 舵机：50 Hz PWM
- IMU：MPU6050，I2C 地址 `0x68`
- USB 转串口：CH340，连接 ESP32-S3 的 UART0

### 最新原理图引脚

| 功能 | GPIO | 协议/方向 |
|---|---:|---|
| DRV8833 nSLEEP | 3 | 数字输出 |
| DRV8833 nFAULT | 4 | 数字输入 |
| DRV8833 AIN1 | 46 | PWM；见下方硬件限制 |
| DRV8833 AIN2 | 9 | PWM |
| DRV8833 BIN1 | 5 | PWM |
| DRV8833 BIN2 | 6 | PWM |
| 舵机 PWM | 45 | LEDC，50 Hz |
| MPU6050 INT | 8 | 数字输入 |
| MPU6050 SCL | 11 | I2C，400 kHz |
| MPU6050 SDA | 12 | I2C，400 kHz |

## Wi-Fi 控制

上电后 ESP32-S3 自己创建热点，无需家庭路由器：

- SSID：`ESP32-Robot`
- 密码：`esp32robot`
- 控制页：[http://192.168.4.1](http://192.168.4.1)

手机连接热点后打开控制页，可以按住前进/后退、紧急停止和调节舵机角度。非零电机指令需要持续刷新；超过 1 秒没有收到新指令时，固件会自动停车。

### HTTP API

| 请求 | 作用 |
|---|---|
| `GET /api/status` | 查询当前控制状态 |
| `GET /api/motor?speed=60` | 电机前进，范围 `-100..100` |
| `GET /api/motor?speed=-60` | 电机后退 |
| `GET /api/servo?angle=90` | 舵机角度，范围 `0..180` |
| `GET /api/stop` | 立即停车 |

## 串口控制

UART0 波特率为 115200，仍保留以下调试命令：

```text
motor <-100..100>
servo <0..180>
i2c scan
imu
help
```

## 构建与烧录

先进入 ESP-IDF 环境，再执行：

```powershell
idf.py set-target esp32s3
idf.py build
idf.py -p COMx flash monitor
```

也可在已激活 ESP-IDF 环境的终端运行：

```powershell
.\build.bat
.\build.bat COM7
```

编译不需要连接开发板；只有烧录和串口监视需要开发板与有效 COM 端口。

## 已知硬件限制

最新原理图把 DRV8833 `AIN1` 接到了 ESP32-S3 的 `GPIO46`。GPIO46 只能输入，不能输出 PWM。固件检测到该映射后会保持 `nSLEEP` 为低电平，避免电机误动作。

必须把 `AIN1` 飞线到可输出的 GPIO，并同步修改 [`esp32_s3/board_pins.h`](esp32_s3/board_pins.h)，双路电机控制才能启用。这是 PCB 引脚问题，无法仅靠固件修复。

## 主要文件

```text
main/app_main.c                 启动、串口命令和外设整合
esp32_s3/board_pins.h          最新原理图引脚映射
esp32_s3/wifi_control.c/.h     Wi-Fi SoftAP、网页和 HTTP API
esp32_s3/drv8833.c/.h          电机驱动
esp32_s3/servo_pwm.c/.h        舵机 PWM
esp32_s3/mpu6050.c/.h          IMU 驱动
sdkconfig.defaults             ESP32-S3、16 MB Flash、Wi-Fi 配置
```
