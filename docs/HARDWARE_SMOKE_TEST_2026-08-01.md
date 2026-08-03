# 旧 PCB 无运动冒烟测试记录（2026-08-01）

## 测试范围

本次只验证 ESP32-S3 主控、Flash、串口、FreeRTOS 任务、Wi-Fi SoftAP 和旧板上可识别的 IMU。
由于旧 PCB 的外设引脚可能与 2026-07-31 最新原理图不同，全程未发送 `ARM`，未驱动电机、舵机或 GPIO35。

## 被测设备与固件

- 串口：`COM7`（CH340）
- 芯片：ESP32-S3 QFN56 revision v0.2
- 晶振：40 MHz
- Flash 配置：16 MB，DIO 80 MHz
- PSRAM：8 MB
- MAC：`3c:0f:02:cb:9c:24`
- 固件：Debug `esp32_s3.bin`，844800 字节
- 应用版本：`cfa5123-dirty`
- ESP-IDF：v5.2.6

## 验收结果

| 项目 | 结果 | 实测证据 |
|---|---|---|
| UART 下载连接 | PASS | esptool 成功识别 ESP32-S3 revision v0.2 |
| Flash 擦写与校验 | PASS | Bootloader、分区表和应用均完成写入，三个镜像哈希校验通过 |
| 正常启动 | PASS | 从 `SPI_FAST_FLASH_BOOT` 加载 factory app，进入 `app_main()` |
| FreeRTOS 主任务 | PASS | `main_task` 在 CPU0 启动并持续响应串口命令 |
| 控制任务 | PASS | SAFE 下依次拒绝 DRIVE、MOTOR_DIRECT、SERVO；ESTOP 和 CLEAR 被及时执行 |
| 传感任务 | PASS | 5 个连续窗口采样率均为 100.00 Hz |
| Wi-Fi 驱动任务 | PASS | Wi-Fi driver task 创建成功，SoftAP 在 2.4 GHz channel 1 启动 |
| Wi-Fi 关联 | PASS | Windows 成功关联 `ESP32-Robot`，WPA2-Personal/CCMP，信号 98% |
| HTTP/DHCP 端到端 | 未完成 | 电脑只有一张 Wi-Fi 网卡；为避免中断互联网，没有继续保持 SoftAP 连接执行 HTTP 请求 |
| MPU6050 探测 | PASS | 当前旧板返回 `WHO_AM_I=0x68` |
| MPU6050 连续采样 | PASS | 5.410 秒增加 541 帧；实测 100.00 Hz |
| SAFE 上电状态 | PASS | `armed=0`、`estop=0`、`faults=0`、Motor A/B 目标与输出全为 0、舵机 disabled、power=0 |
| ARM 门禁 | PASS | SAFE 下非零电机、双电机和舵机命令均被控制任务拒绝 |
| 参数校验 | PASS | 超范围 drive、servo、power 均返回 usage 警告，未知命令被拒绝 |
| ESTOP 锁存与恢复 | PASS | SAFE → ESTOP 后仍为零输出；CLEAR 后返回 SAFE，未自动 ARM |
| 25 秒串口浸泡 | PASS | 6 次状态均安全，IMU 五段均 100.00 Hz，无 reboot、panic、assert 或 watchdog 日志 |

浸泡测试覆盖的设备运行时间为 `489022..514432 ms`；后续短测时仍维持 SAFE。该记录证明的是
旧板上的无运动主控基础，不证明最新 PCB 的引脚、电气、电机方向、舵机行程或负载稳定性。

## 已发现的板级问题

旧 PCB 的自动下载 DTR/RTS 时序不可靠：自动探测曾得到 `Wrong boot mode (0x8)`，烧录需要手动执行
“按住 BOOT → 短按 EN/RST → 松开 EN/RST → 松开 BOOT”。烧录完成后还需要在不按 BOOT 的情况下
短按 EN/RST，才能从 Flash 正常启动。数据线、CH340、UART 下载和 Flash 本身均正常。

## 后续验证

- 增加 STA 优先、SoftAP 回退的网络模式后，在不切断电脑互联网的情况下验证 DHCP、控制页和全部 HTTP v1 接口。
- 使用最新 PCB 按分阶段流程验证 nFAULT、GPIO35、舵机、单电机和双电机；首次执行器测试仍需显式 ARM。
- 对旧 PCB 不进行外设运动测试，除非先取得旧板原理图并逐项确认引脚与供电。
