# 新人快速上手

本指南介绍如何在 Windows 上完成配置环境 → 编译烧录 → 连接 Wi-Fi → 打开板载实验台 →
无运动验证。编译本身不需要连接开发板。

> 安全提示：设备上电默认处于 `SAFE`。首次使用只做无运动验证；接电机、舵机或新 PCB
> 前，必须使用限流电源、悬空轮子并准备独立断电手段。

## 1. 准备环境

需要：

- ESP-IDF 5.2.6；
- Python 3；
- 支持数据传输的 Type-C 线；
- ESP32-S3 开发板或项目 PCB；
- 2.4 GHz Wi-Fi 或手机热点。

打开 **ESP-IDF PowerShell**，进入项目并检查环境：

```powershell
cd D:\esp32_project
idf.py --version
python --version
```

新电脑可先克隆仓库：

```powershell
git clone git@github.com:Zyx0303/esp32_project.git
cd esp32_project
```

## 2. 配置 Wi-Fi

首次使用时复制模板：

```powershell
Copy-Item components\robot_controller\wifi_credentials.example.h `
          components\robot_controller\wifi_credentials.h
```

编辑 `components/robot_controller/wifi_credentials.h`：

```c
#pragma once
#define ROBOT_WIFI_STA_SSID     "你的 2.4 GHz Wi-Fi 名称"
#define ROBOT_WIFI_STA_PASSWORD "你的 Wi-Fi 密码"
```

该文件已被 Git 忽略，不要把真实密码提交到 GitHub。修改 Wi-Fi 后需要重新编译和烧录。

## 3. 编译和烧录

编译：

```powershell
idf.py set-target esp32s3
idf.py build
```

连接开发板并查看串口：

```powershell
[System.IO.Ports.SerialPort]::GetPortNames()
```

以下以 `COM8` 为例，请替换为实际端口：

```powershell
idf.py -p COM8 flash monitor
```

使用 `Ctrl+]` 退出监视器。日志会显示 ESP32 获得的局域网 IP。

如果无法进入下载模式：按住 `BOOT`，短按并松开 `EN/RST`，再松开 `BOOT` 后重新烧录。
如果没有 COM 端口，依次检查数据线、USB 接口、CH340 驱动和设备管理器。

## 4. 打开板载实验台

电脑或手机连接 `ESP32-Robot` 热点（密码 `esp32robot`），在浏览器打开
[http://192.168.4.1/](http://192.168.4.1/)。页面由控制板提供，无需运行 Python，也不依赖互联网。
如果 ESP32 已连到局域网，也可从同一局域网打开串口日志显示的设备地址。

- 上电保持 `SAFE`。先看设备状态和 IMU，再按需解锁电机、舵机和推拉杆控制。
- 在「IMU 实验」中可查看实时原始数据。点击「开始录制」后由板端以 100 Hz 保存，
  点击「结束录制」后下载封存的 `.log` 文件。重连可查看板上记录状态并重试下载。
- 若显示「日志状态不可用」，仍可查看实时 IMU 数据。请检查串口中的存储挂载日志。

操作及断线保护见 [板载 HTTP 控制](PHONE_HTTP_CONTROL.md)。原有 Python 客户端仍可通过
`start_robot_control.bat` 或 `python tools/start_robot_control.py` 启动。

## 5. 首次安全验证

运行无运动冒烟测试：

```powershell
python tools\robot_smoke_test.py
```

它会验证设备身份、HTTP API 和以下状态切换：

```text
SAFE → READY → SAFE → ESTOP → SAFE
```

测试不会输出电机、舵机或 推拉杆，结束时应显示 `[PASS]` 并回到 `SAFE`。

提交代码前运行开发门禁：

```powershell
python tools\p0_preflight.py
idf.py build
```

## 6. 常见问题

- **找不到设备：**确认电脑与 ESP32 在同一局域网，从串口日志读取 IP，再使用
  `python tools\start_robot_control.py --url http://设备IP --find-only`。
- **连不上 Wi-Fi：**检查名称、密码和 2.4 GHz 是否开启，修改后重新编译烧录。
- **有电但没有 COM：**板子供电不代表 USB 串口枚举成功，重点检查数据线、CH340、
  D+/D- 和驱动。
- **按钮能点但电机不动：**`SAFE`、`FAULT` 或 `ESTOP` 状态会拒绝运动命令，先排除硬件
  问题再 ARM，不要移除安全状态机。
- **停止发送命令约三秒后停车：**这是断链宽限期结束后的正常保护行为；状态回到 `SAFE`，重连后需要重新 ARM。

## 7. 下一步

接入真实 PCB 或开发新功能前，根据需要阅读：

- [项目完整说明](../README.md)
- [软件架构](ARCHITECTURE.md)
- [HTTP 控制协议](CONTROL_PROTOCOL_V1.md)
- [接 PCB 前基线](PRE_PCB_BASELINE_2026-08-03.md)
- [P0 验收标准](P0_ACCEPTANCE_MATRIX.md)

真实运动测试必须分阶段进行；不要在旧 PCB 引脚未核对、轮子落地或舵机行程未知时发送
运动命令。
