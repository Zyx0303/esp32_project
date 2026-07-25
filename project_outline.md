# ESP32-S3 机器人控制器

## 第1页：项目概述

### 标题：ESP32-S3 机器人控制器

### 内容
- **项目背景**: 基于 ESP-IDF 5.3.x 的机器人控制固件
- **目标硬件**: ESP32-S3 开发板
- **核心功能**:
  - BLE 无线控制（手机/电脑）
  - IMU 数据采集（MPU6050）
  - Flash 数据存储（DMA批量写入）
  - 电机 PWM 控制（DRV8833）
  - 舵机控制

---

## 第2页：系统架构

### 标题：系统架构与任务设计

### 内容
- **FreeRTOS 任务架构**:

| 任务 | 优先级 | 功能 |
|------|--------|------|
| ble_host | max-1 | NimBLE 蓝牙主机 |
| imu_task | max-1 | IMU中断采集→Flash |
| motor_ctrl | 2 | 电机/舵机控制队列 |
| flush_task | 2 | Flash批量写入 |

- **数据流**:
  - BLE → 队列 → motor_ctrl_task → PWM输出
  - GPIO15中断 → imu_task → flash_storage

- **外设驱动**:
  - `drv8833.c` - 双路电机驱动
  - `servo_pwm.c` - 舵机PWM
  - `mpu6050.c` - IMU驱动
  - `ble_control.c` - BLE GATT服务
  - `flash_storage.c` - Flash批量存储

---

## 第3页：功能与接口

### 标题：BLE 控制接口

### 内容
- **BLE设备名**: MSRR-1
- **GATT Service**: 0xFF00
- **控制特性**:

| UUID | 名称 | 范围 |
|------|------|------|
| 0xFF01 | 电机速度 | 0-100% |
| 0xFF02 | 舵机角度 | 0-180° |

- **使用方法**: nRF Connect / BLE Scanner 连接后写入值

### 已知限制
- GPIO46 为输入专用，电机A无法工作（需改硬件到GPIO4~10）
- Flash 循环写入，wrap时旧数据丢失

---

## 附录：更新记录

| 日期 | 更新内容 |
|------|----------|
| - | 初始化项目，添加 MPU6050、DRV8833 驱动 |
| - | 重构 app_main.c，删除旧测试代码 |
| - | 新增 flash_storage.c，Flash DMA 批量存储 |
| - | 新增 ble_control.c，BLE GATT 服务 |
| - | 修复 flash_storage.c 扇区擦除逻辑 |
| - | BLE 回调改用队列实现线程安全 |
| - | 自定义分区表 (512KB imu_data) |
