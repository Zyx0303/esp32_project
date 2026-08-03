# 接 PCB 前软件基线记录（2026-08-03）

## 目的

在最新机器人 PCB 到手前，冻结一份可以烧录、联网、诊断和安全分阶段板测的软件基线。
本记录证明软件与 ESP32-S3 最小系统板链路，不替代最新 PCB 的电气和负载验收。

## 已验证环境

- ESP32-S3 QFN56 revision v0.2；
- 16 MB Flash、8 MB PSRAM；
- USB Serial/JTAG：COM8；
- ESP-IDF v5.2.6；
- STA：2.4 GHz 手机热点；
- ESP32 DHCP 地址：`172.26.96.61`（动态地址，仅为本次记录）；
- 电脑同时连接有线网和手机热点。

## 真机结果

| 项目 | 结果 | 证据 |
|---|---|---|
| 编译与烧录 | PASS | Bootloader、分区表和应用写入并完成哈希校验 |
| 固件启动 | PASS | ESP-IDF 进入 `app_main()`，无 panic/assert |
| 外设缺失降级 | PASS | 最小系统板没有 MPU6050，记录不可用后继续启动网络 |
| STA | PASS | WPA2 关联成功并取得 DHCP 地址 |
| 救援 AP 初始化 | PASS | DHCP 服务在 `192.168.4.1` 启动 |
| 互联网并存 | PASS | 电脑通过手机热点访问互联网，同时访问 ESP32 |
| 控制页 | PASS | `GET /` 返回 HTTP 200 |
| 设备/状态 API | PASS | `/api/v1/device` 与 `/api/v1/status` 返回有效 JSON |
| 自动发现 | PASS | 双网卡环境自动扫描到 `172.26.96.61` |
| 无运动冒烟 | PASS | `SAFE -> READY -> SAFE -> ESTOP -> SAFE` |
| 最终输出 | PASS | Motor A/B 为 0、舵机关闭、GPIO35 释放 |
| Wi-Fi 诊断 | PASS | 状态包含断开原因、重连计数和退避延迟 |
| Debug 干净构建 | PASS | 独立目录镜像 849392 字节，应用分区剩余约 19% |
| Release 干净构建 | PASS | 独立目录镜像 703968 字节，应用分区剩余约 32% |

## 安全工具

```powershell
# 自动发现并启动 GUI
.\start_robot_control.bat

# 只发现设备
python tools\start_robot_control.py --find-only

# 默认无运动冒烟
python tools\robot_smoke_test.py

# 最新 PCB 上的单阶段低输出测试（仍需输入 MOVE）
python tools\robot_smoke_test.py --allow-motion --test motor-a
```

运动阶段不会自动串联。必须依次完成 Motor A、停止检查、Motor B、停止检查，再考虑双电机
人工测试。舵机测试前必须先确认独立供电和机械行程。

## 最新 PCB 到手后的后置项

- 电源轨、静态电流和上电时序；
- CH340 下载与复位时序；
- MPU6050 WHO_AM_I、DATA_READY、采样抖动和零偏；
- DRV8833 nFAULT 实际电平、电机方向、电流和噪声；
- GPIO35/2N7002 实际逻辑；
- 舵机供电、真实角度和机械干涉；
- 电机带载时的 Wi-Fi、IMU 和 ESP32 复位稳定性。
