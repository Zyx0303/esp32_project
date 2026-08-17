# 新人快速上手

本指南用于在 Windows 上完成：配置环境 → 编译烧录 → 连接 Wi-Fi → 启动上位机 →
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

## 4. 启动上位机

让电脑与 ESP32 连接同一个局域网，然后双击项目根目录中的：

```text
start_robot_control.bat
```

也可以使用终端：

```powershell
# 自动发现并启动上位机
python tools\start_robot_control.py

# 只查找设备，不打开界面
python tools\start_robot_control.py --find-only

# 已知设备 IP
python tools\start_robot_control.py --url http://192.168.1.123
```

自动发现只读取设备信息，不会 ARM 或发送运动命令。

局域网不可用时，可连接 ESP32 的救援热点：

```text
SSID：ESP32-Robot
密码：esp32robot
地址：http://192.168.4.1
```

连接救援热点后，单 Wi-Fi 网卡的电脑通常会暂时失去互联网。

## 5. 首次安全验证

运行无运动冒烟测试：

```powershell
python tools\robot_smoke_test.py
```

它会验证设备身份、HTTP API 和以下状态切换：

```text
SAFE → READY → SAFE → ESTOP → SAFE
```

测试不会输出电机、舵机或 GPIO35，结束时应显示 `[PASS]` 并回到 `SAFE`。

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
- **电机约一秒后停车：**这是通信看门狗的正常保护行为。

## 7. 下一步

接入真实 PCB 或开发新功能前，根据需要阅读：

- [项目完整说明](../README.md)
- [软件架构](ARCHITECTURE.md)
- [HTTP 控制协议](CONTROL_PROTOCOL_V1.md)
- [接 PCB 前基线](PRE_PCB_BASELINE_2026-08-03.md)
- [P0 验收标准](P0_ACCEPTANCE_MATRIX.md)

真实运动测试必须分阶段进行；不要在旧 PCB 引脚未核对、轮子落地或舵机行程未知时发送
运动命令。
