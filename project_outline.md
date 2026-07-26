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
   ├── 电机速度回调 ──► LEDC ──► DRV8833
   └── 舵机角度回调 ──► LEDC ──► 舵机

MPU6050 ── I2C ──► ESP32-S3
UART0  ── 串口命令 ──► 同一套控制回调
```

## Wi-Fi 接口

- 热点：`ESP32-Robot`
- 地址：`192.168.4.1`
- 网页控制：`GET /`
- 状态：`GET /api/status`
- 电机：`GET /api/motor?speed=-100..100`
- 舵机：`GET /api/servo?angle=0..180`
- 停车：`GET /api/stop`
- 安全策略：非零电机命令 1 秒超时自动停车

## 硬件阻塞项

原理图将 DRV8833 AIN1 接到输入专用 GPIO46。固件保持驱动器休眠，直到硬件改线并更新 `board_pins.h`。
