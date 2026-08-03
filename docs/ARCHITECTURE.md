# ESP32-S3 机器人控制器概要

## 目标

- 使用 Wi-Fi 而不是蓝牙进行无线控制
- 控制 DRV8833 双路电机和一个 50 Hz 舵机
- 通过 I2C 读取 MPU6050
- 保留 UART0 调试命令和 Flash 数据存储模块

## 软件结构

```text
手机浏览器
   │  Wi-Fi SoftAP + HTTP
   ▼
wifi_control.c
   └── HTTP 命令 ──► FreeRTOS 命令队列
                         │
                         ▼
                    app_control.c
                         │
                         ▼
                  control_manager.c
                    ├── 状态机/ARM/看门狗
                    ├── 斜坡与换向死区 ──► app_actuators.c ──► DRV8833
                    └── 舵机速率限制 ─────► app_actuators.c ──► 舵机 PWM

MPU6050 ── I2C ──► app_imu.c ──► app_status.c
UART0  ── app_console.c ───────► 同一命令队列
```

`app_main.c` 只装配上述模块并规定安全启动顺序。运行时只有 `app_control.c`
中的 `control_task` 可以通过执行器适配层写硬件；HTTP、UART 和 IMU 任务只能提交
命令或发布/读取状态快照。

## Wi-Fi 接口

- 热点：`ESP32-Robot`
- 地址：`192.168.4.1`
- 局域网：STA 自动连接本地网络，DHCP 地址由串口 `status` 输出；SoftAP 始终保留为救援入口
- 网页控制：`GET /`
- 状态：`GET /api/v1/status`
- 设备信息：`GET /api/v1/device`
- 安全控制：`POST /api/v1/arm|disarm|estop|fault/clear`
- 差速/独立电机：`POST /api/v1/drive|motors`
- 舵机/GPIO35：`POST /api/v1/servo|power`
- 安全策略：上电保持 SAFE，ARM 后才允许运动；超时、DISARM、ESTOP、nFAULT 均执行同一硬件安全停止

完整接口见 [`CONTROL_PROTOCOL_V1.md`](CONTROL_PROTOCOL_V1.md)。

## 硬件阻塞项

原理图将 DRV8833 AIN1 接到 GPIO46。GPIO46 在 ESP32-S3 上支持输入和输出，但属于启动绑带引脚，外部电路不得在复位采样期间强制错误电平。
