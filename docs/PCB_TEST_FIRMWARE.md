# ESP32-S3 最小系统板 PCB 信号测试固件

`testforpcb` 分支只构建板级测试程序，不启动 Wi-Fi、IMU、NVS 或正式机器人控制栈。
它用于把 ESP32-S3 最小系统板接到机器人 PCB 后，逐路检查控制信号和执行器响应。

## 引脚与预期信号

| 对象 | ESP32-S3 引脚 | 测试信号 |
|---|---:|---|
| DRV8833 nSLEEP | GPIO3 | 高电平唤醒，停止/故障时为低 |
| DRV8833 nFAULT | GPIO4 | 输入，低电平表示故障 |
| Motor A | GPIO46 / GPIO9 | 16 kHz PWM，正反转分别使用 AIN1/AIN2 |
| Motor B | GPIO5 / GPIO6 | 16 kHz PWM，正反转分别使用 BIN1/BIN2 |
| 舵机 | GPIO45 | 50 Hz PWM，0–180° 对应 500–2500 us |
| 电磁铁接口 | GPIO35 | GPIO 高时 2N7002 导通，外部接口被拉低 |

GPIO46 同时是 ESP32-S3 启动绑带引脚。复位采样期间不要由外部电路强行驱动它。

## 烧录和串口

在 ESP-IDF 5.3 环境中执行：

```powershell
idf.py set-target esp32s3
idf.py build
idf.py -p COMx flash monitor
```

串口为 UART0、115200 baud。上电后所有输出都关闭；先输入 `arm`，再发送测试命令。

```text
help
status
arm
motor a 10
motor a -10
motor b 10
motors 10 -10
servo 45
servo 90
servo off
magnet on
magnet off
stop
```

输入 `test` 会依次执行 Motor A ±10%、Motor B ±10%、电磁铁短时置位以及舵机
45°/90°/135°，最后自动关闭全部输出并退出 ARM。

## 安全约束

- 所有会激活负载的命令必须先执行 `arm`。
- 活动输出 5 秒没有收到新命令会自动停止并退出 ARM。
- `nFAULT` 变低会立即停止电机、舵机和电磁铁并锁存故障。
- 故障原因消失后输入 `clear`，再重新 `arm`。
- 任意时刻输入 `stop` 都会让 DRV8833 休眠、舵机停止 PWM、电磁铁释放。
- 第一次接实际执行器时使用限流电源，并从 `10%` 电机输出开始。

