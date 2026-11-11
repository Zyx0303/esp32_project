# ESP32-S3 光板手工测试记录（2026-08-18）

> 本文是人工实验原始记录。测试人必须亲自执行命令、观察现象并填写“实测现象”；
> 未执行的项目填写 `NOT RUN`，条件不具备填写 `BLOCKED`，不得用自动化测试结果代替真机现象。

## 1. 被测对象

| 字段 | 实际信息 |
|---|---|
| 测试日期与时间 | 2026-08-18 `__:__` 至 `__:__` |
| 测试人 | `<姓名>` |
| PCB 编号/版本 | `<编号或丝印>` |
| 供电方式 | `<USB / 限流电源，电压与限流值>` |
| ESP32-S3 串口 | `COM10`（测试前确认） |
| Wi-Fi SSID | `BSRL` |
| 电脑 IP | `<填写>` |
| ESP32 IP | `<填写>` |
| ESP-IDF | `v5.2.6` |
| 固件 build_date/build_time | `<从 /api/v1/device 填写>` |
| Git commit / 工作区状态 | `<sha>` / `<clean 或 dirty>` |
| 环境温度 | `<可选>` |
| 使用仪器 | `<无 / 万用表 / 示波器 / 逻辑分析仪>` |

## 2. 证据管理

建议创建本次测试的证据目录：

```powershell
New-Item -ItemType Directory -Force docs\evidence\2026-08-18
```

证据统一命名：

```text
T01_boot_serial.txt
T02_uart_baseline.txt
T03_wifi_baseline.json
T04_imu_cross_channel.md
T05_imu_six_faces.csv
T06_imu_rotation.md
T07_imu_soak_start.json
T07_imu_soak_end.json
T08_imu_zero_before.json
T08_imu_zero_after.json
T09_safety_states.md
T10_wifi_reconnect.md
```

串口监视使用时间戳且禁止自动复位：

```powershell
$env:ESP_IDF_MONITOR_NO_RESET = "1"
idf.py -p COM10 monitor --timestamps
```

在串口窗口中输入命令后，把相关输出复制到对应证据文件。Wi-Fi JSON 可直接保存：

```powershell
$device = 'http://<ESP32-IP>'
Invoke-RestMethod "$device/api/v1/status" |
    ConvertTo-Json -Depth 8 |
    Set-Content -Encoding utf8 docs\evidence\2026-08-18\T03_wifi_baseline.json
```

## 3. 判定约定

- `PASS`：实测现象满足通过标准，并已经填写证据。
- `FAIL`：能够执行，但任一必要标准不满足。
- `BLOCKED`：当前硬件、固件或仪器不具备测试条件。
- `NOT RUN`：尚未执行。
- 串口和 Wi-Fi 读取的是持续更新的状态快照，不要求每个瞬间的数值完全相同。
- MPU6050 当前配置为加速度 ±2g、陀螺仪 ±250°/s：约 `16384 LSB/g`、`131 LSB/(°/s)`。
- “加速度归零”不能把重力轴清零；静止时加速度合量应保留约 1g。

## 4. T01：USB、复位与正常启动

### 操作

1. 退出所有串口工具，确认没有按住 `BOOT`。
2. 短按 `EN/RST`，等待固件正常启动。
3. 使用 `--no-reset` monitor 接入并记录启动信息。
4. 重复正常复位 3 次。

### 记录

| 次数 | boot 模式 | 是否进入 app_main | panic/assert/watchdog | Wi-Fi 是否恢复 | 结果 |
|---:|---|---|---|---|---|
| 1 | `<应为 SPI_FAST_FLASH_BOOT>` |  |  |  | NOT RUN |
| 2 |  |  |  |  | NOT RUN |
| 3 |  |  |  |  | NOT RUN |

实测现象：

```text
<粘贴关键启动日志，记录是否再次误入 DOWNLOAD(USB/UART0)>
```

通过标准：3 次均从 Flash 正常启动；无 panic、assert、watchdog reset；Wi-Fi 能恢复。

证据：`docs/evidence/2026-08-18/T01_boot_serial.txt`

最终判定：`NOT RUN`

## 5. T02：串口基线状态

### 操作

在串口依次输入：

```text
disarm
status
imu
```

### 记录

| 观测项 | 预期 | 实测 |
|---|---|---|
| 安全状态 | 未解锁，非 ESTOP |  |
| Motor A target/output | `0 / 0` |  |
| Motor B target/output | `0 / 0` |  |
| 舵机 | disabled |  |
| GPIO35 外部接口 | released |  |
| IMU valid | `true` |  |
| IMU samples | 持续增加 |  |
| IMU errors | 不持续增加 |  |

实测现象：

```text
<粘贴 status 与 imu 输出>
```

证据：`docs/evidence/2026-08-18/T02_uart_baseline.txt`

最终判定：`NOT RUN`

## 6. T03：Wi-Fi/HTTP 基线状态

### 操作

```powershell
$device = 'http://<ESP32-IP>'
Invoke-RestMethod "$device/api/v1/device" | ConvertTo-Json
Invoke-RestMethod "$device/api/v1/status" | ConvertTo-Json -Depth 8
```

### 记录

| 观测项 | 预期 | 实测 |
|---|---|---|
| HTTP `/device` | 200；`target=esp32s3`、`api_version=1` |  |
| HTTP `/status` | 200；JSON 完整 |  |
| STA connected | `true` |  |
| STA IP | 与实际访问地址一致 |  |
| faults | `0`，或对非零位逐项解释 |  |
| free_heap | 大于 0 且后续不持续下降 |  |

实测现象：

```text
<填写响应时间、是否超时、页面/PowerShell 现象>
```

证据：`docs/evidence/2026-08-18/T03_wifi_baseline.json`

最终判定：`NOT RUN`

## 7. T04：IMU 串口与 Wi-Fi 双通道一致性

### 操作

1. 把板子放稳，保持静止至少 3 秒。
2. 串口输入 `imu` 并立即记录。
3. 在另一 PowerShell 立即读取：

```powershell
(Invoke-RestMethod "$device/api/v1/status").imu | ConvertTo-Json
```

4. 重复 5 组。不要要求两条链路的 sample 编号完全相同。

### 原始记录

| 组 | 通道 | accel X/Y/Z | gyro X/Y/Z | samples | errors | 时间 |
|---:|---|---|---|---:|---:|---|
| 1 | UART |  |  |  |  |  |
| 1 | Wi-Fi |  |  |  |  |  |
| 2 | UART |  |  |  |  |  |
| 2 | Wi-Fi |  |  |  |  |  |
| 3 | UART |  |  |  |  |  |
| 3 | Wi-Fi |  |  |  |  |  |
| 4 | UART |  |  |  |  |  |
| 4 | Wi-Fi |  |  |  |  |  |
| 5 | UART |  |  |  |  |  |
| 5 | Wi-Fi |  |  |  |  |  |

通过标准：同一静止姿态下，两通道的主重力轴、正负号和数量级一致；sample 持续前进；error 不增加。

实测现象：

```text
<记录是否有跳变、冻结、Wi-Fi 超时或串口刷 read raw 错误>
```

证据：`docs/evidence/2026-08-18/T04_imu_cross_channel.md`

最终判定：`NOT RUN`

## 8. T05：IMU 六面静态加速度测试

每个姿态稳定 3 秒后读取 5 次，填写平均值。`+X/-X` 等方向以 PCB 丝印或测试人定义为准，
开始前必须写明坐标定义并拍照留档。

坐标定义：`<填写 PCB 哪一边是 +X、+Y、+Z>`

| 放置面 | 平均 ax | 平均 ay | 平均 az | 加速度合量 | 主轴期望 | 实测现象 | 结果 |
|---|---:|---:|---:|---:|---|---|---|
| +X 向上 |  |  |  |  | X 约 `+16384` |  | NOT RUN |
| -X 向上 |  |  |  |  | X 约 `-16384` |  | NOT RUN |
| +Y 向上 |  |  |  |  | Y 约 `+16384` |  | NOT RUN |
| -Y 向上 |  |  |  |  | Y 约 `-16384` |  | NOT RUN |
| +Z 向上 |  |  |  |  | Z 约 `+16384` |  | NOT RUN |
| -Z 向上 |  |  |  |  | Z 约 `-16384` |  | NOT RUN |

通过标准（基础板测）：主轴符号随翻面反转；合量约为 `16384`，建议初验范围 `13900..18800`
（约 0.85g..1.15g）；无固定值、饱和值或突然归零。

证据：`docs/evidence/2026-08-18/T05_imu_six_faces.csv`，以及六个姿态照片。

最终判定：`NOT RUN`

## 9. T06：IMU 三轴旋转响应

保持其他轴尽量不动，分别绕 X、Y、Z 做一次正向和一次反向缓慢旋转，观察对应 gyro。

| 轴 | 正向峰值/符号 | 反向峰值/符号 | 静止后平均值 | 串口/Wi-Fi趋势一致 | 结果 |
|---|---|---|---|---|---|
| X |  |  |  |  | NOT RUN |
| Y |  |  |  |  | NOT RUN |
| Z |  |  |  |  | NOT RUN |

通过标准：正反旋转符号相反；主要响应出现在对应轴；停止后返回稳定零偏附近；无长时间冻结。

实测现象：

```text
<记录响应方向、延迟、噪声、是否出现 read raw>
```

证据：`docs/evidence/2026-08-18/T06_imu_rotation.md`

最终判定：`NOT RUN`

## 10. T07：IMU 与系统静置浸泡

### 操作

1. 保持 DISARM，记录起始 `/status`。
2. 板子静置运行至少 10 分钟，不发送运动命令。
3. 记录结束 `/status` 和串口异常日志。

| 指标 | 起始 | 结束 | 差值/结果 |
|---|---:|---:|---|
| uptime_ms |  |  |  |
| imu samples |  |  |  |
| imu errors |  |  |  |
| free_heap |  |  |  |
| Wi-Fi reconnects |  |  |  |
| reboot/panic/assert/watchdog |  |  |  |

计算采样率：

```text
(结束 samples - 起始 samples) / (结束 uptime_ms - 起始 uptime_ms) × 1000 = ____ Hz
```

通过标准：采样率建议 `95..105 Hz`；errors 不增加；无连续 `read raw`；无异常复位；free_heap
没有持续单向下降；Wi-Fi 查询始终可用。

证据：`T07_imu_soak_start.json`、`T07_imu_soak_end.json` 和串口日志。

最终判定：`NOT RUN`

## 11. T08：IMU 归零/零偏校准

当前固件仅发布 MPU6050 原始值，没有实现串口 `imu zero` 或 HTTP 校准接口。因此本项当前必须记录为：

```text
BLOCKED：固件缺少明确的 IMU 零偏校准命令、校准状态和校准前后数据字段。
```

以后实现后，必须按以下方式测试：

1. 板子固定在定义好的水平基准面，静置至少 5 秒。
2. 保存校准前 500 帧统计：gyro 三轴均值/标准差、accel 三轴均值/合量。
3. 分别通过串口和 Wi-Fi触发同一个校准流程。
4. 保存校准偏置、采样帧数、运动拒绝原因及校准后 500 帧统计。
5. 陀螺仪静止均值应接近 0；加速度 X/Y 接近 0，重力轴必须保留约 ±1g。
6. 复位后检查偏置是否按设计保留；若仅做 RAM 校准，应明确记录“复位后不保留”。

| 项目 | 校准前 | 校准偏置 | 校准后 | 通过标准 | 结果 |
|---|---:|---:|---:|---|---|
| gyro X mean |  |  |  | 接近 0 | BLOCKED |
| gyro Y mean |  |  |  | 接近 0 | BLOCKED |
| gyro Z mean |  |  |  | 接近 0 | BLOCKED |
| accel X mean |  |  |  | 水平时接近 0 | BLOCKED |
| accel Y mean |  |  |  | 水平时接近 0 | BLOCKED |
| accel Z mean |  |  |  | 保留约 ±16384 | BLOCKED |

最终判定：`BLOCKED`

## 12. T09：安全状态机手工测试（无运动输出）

保持没有连接电机和舵机。每一步在串口执行命令，并用 Wi-Fi `/status` 复核。

| 步骤 | 串口操作 | 预期 | 串口实测 | Wi-Fi 实测 | 结果 |
|---:|---|---|---|---|---|
| 1 | `disarm` | 未解锁，A/B 输出为 0 |  |  | NOT RUN |
| 2 | `arm` | armed=true，尚无电机输出 |  |  | NOT RUN |
| 3 | `disarm` | armed=false，所有执行器安全停止 |  |  | NOT RUN |
| 4 | `estop` | estop=true，输出为 0 |  |  | NOT RUN |
| 5 | `arm` | 在 ESTOP 下应拒绝 |  |  | NOT RUN |
| 6 | `clear` | 原因消失后返回安全态，不自动 ARM |  |  | NOT RUN |

通过标准：两通道状态一致；安全命令均被执行；ESTOP/CLEAR 不产生输出；CLEAR 后仍未解锁。

证据：`docs/evidence/2026-08-18/T09_safety_states.md`

最终判定：`NOT RUN`

## 13. T10：Wi-Fi 掉线与恢复

### 操作

1. 保持 DISARM，保存起始 `/status`。
2. 暂时关闭或离开 `BSRL`，观察串口断线原因与重连退避。
3. 恢复 `BSRL`，等待 ESP32 重新获取 IP。
4. 重新扫描设备，保存结束 `/status`。

| 观测项 | 实测现象 |
|---|---|
| 掉线 reason |  |
| reconnect_delay_ms 变化 |  |
| 救援热点 ESP32-Robot 是否仍存在 |  |
| 恢复耗时 |  |
| 恢复后 IP |  |
| reconnects 增量 |  |
| 是否重启/卡死 |  |

通过标准：网络恢复后无需重新烧录或复位；HTTP 可再次访问；全程保持安全输出。

证据：`docs/evidence/2026-08-18/T10_wifi_reconnect.md`

最终判定：`NOT RUN`

## 14. 可选电气测试（有仪器时）

| ID | 项目 | 仪器 | 操作摘要 | 实测现象 | 证据 | 结果 |
|---|---|---|---|---|---|---|
| E01 | 3.3V 与静态电流 | 万用表/限流电源 | USB 上电、保持 DISARM |  |  | NOT RUN |
| E02 | DRV8833 nFAULT 空闲电平 | 万用表/示波器 | 不接电机、读取 GUI 同步状态 |  |  | NOT RUN |
| E03 | GPIO35/2N7002 电平关系 | 万用表 | 手动置位/释放，各测一次 |  |  | NOT RUN |
| E04 | 舵机 PWM | 示波器 | 不接舵机，测 0/90/180° 脉宽 |  |  | NOT RUN |
| E05 | Motor A/B PWM | 示波器 | 不接电机、限流、10% 短测 |  |  | NOT RUN |
| E06 | I²C 波形 | 逻辑分析仪 | GPIO11 SCL、GPIO12 SDA |  |  | NOT RUN |

## 15. 异常与问题单

| 问题 ID | 时间 | 操作步骤 | 实际现象 | 是否可复现 | 临时恢复方式 | 关联证据 | 状态 |
|---|---|---|---|---|---|---|---|
| ISSUE-01 |  |  |  |  |  |  | OPEN |

已知需重点复核：

- 默认 `idf.py monitor` 可能通过 DTR/RTS 让旧板误入 `DOWNLOAD(USB/UART0)`；本次必须使用 `--no-reset`。
- 已观察到 MPU6050 运行中连续 `read raw`；浸泡测试必须记录首次出现时间和复位后的恢复情况。

## 16. 测试结论

| 范围 | 结论 | 备注 |
|---|---|---|
| USB/启动 | NOT RUN |  |
| 串口通信 | NOT RUN |  |
| Wi-Fi/HTTP | NOT RUN |  |
| IMU 双通道输出 | NOT RUN |  |
| IMU 六面与旋转响应 | NOT RUN |  |
| IMU 浸泡稳定性 | NOT RUN |  |
| IMU 归零 | BLOCKED | 当前固件未实现校准入口 |
| 安全状态机 | NOT RUN |  |
| Wi-Fi 重连 | NOT RUN |  |
| 可选电气测试 | NOT RUN |  |

总体结论：`NOT RUN`

遗留风险：

```text
<填写未测项目、失败项以及是否允许进入下一阶段>
```

测试人签名：`____________`　日期：`____________`

