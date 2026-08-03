# ESP32-S3 机器人控制器

这是一个基于 ESP-IDF 的单板机器人控制项目。ESP32-S3 直接控制 DRV8833 双路直流
电机、一个 PWM 舵机、MPU6050 IMU 和一路经 2N7002 输出的外部低有效接口，并通过
Wi-Fi 向浏览器及桌面上位机提供控制能力。

项目当前使用 **Wi-Fi，而不是蓝牙**。正常使用时，ESP32 和电脑连接同一个局域网；
局域网不可用时，仍可连接 ESP32 自己建立的救援热点。

> 安全提示：电机、舵机和外接电源可能造成机械运动、过流或夹伤。首次接入新 PCB 时，
> 必须按照本文的分阶段板测流程操作。固件上电默认进入 `SAFE`，但软件保护不能替代
> 限流电源、保险措施和机械急停。

## 1. 当前状态

目前已经具备：

- ESP32-S3、16 MB Flash 和 8 MB PSRAM 工程配置；
- DRV8833 双电机独立控制及差速混控；
- 电机斜坡、反转换向死区、命令超时停车和 nFAULT 故障处理；
- 50 Hz 舵机 PWM、角度限制、脉宽配置和速率限制；
- MPU6050 初始化、WHO_AM_I 检查和 100 Hz 原始数据采样；
- GPIO35 经 2N7002 控制外部低有效接口；
- Wi-Fi APSTA、HTTP v1 API、浏览器页面和 Python 桌面上位机；
- UART0 调试控制；
- `BOOT / SAFE / READY / RUNNING / FAULT / ESTOP` 安全状态机；
- 统一命令队列、状态快照和模块化 FreeRTOS 任务；
- 无硬件自动化测试和 ESP-IDF 编译验证。

最近一次模块化修改验证结果：22 项主机测试通过，ESP-IDF 5.2.6 成功生成 ESP32-S3
固件，应用镜像约 848 KB，1 MB 应用分区剩余约 19%。

旧 PCB 已验证烧录、启动、串口、FreeRTOS、Wi-Fi 关联、ARM 门禁和 MPU6050 100 Hz
采样；最新 PCB 的电机方向、舵机行程、电源稳定性和实际引脚仍需真机验证。详细记录见
[`docs/HARDWARE_SMOKE_TEST_2026-08-01.md`](docs/HARDWARE_SMOKE_TEST_2026-08-01.md)。

## 2. 硬件组成与引脚

- 主控：ESP32-S3-WROOM-1-N16R8
- 电机驱动：DRV8833 双路 H 桥
- 舵机：标准 50 Hz PWM 舵机
- IMU：MPU6050，I2C 地址 `0x68`
- USB 转串口：CH340，连接 ESP32-S3 UART0
- 当前原理图：
  [`hardware/schematics/robot_controller_2026-07-31.svg`](hardware/schematics/robot_controller_2026-07-31.svg)

最新原理图对应的引脚如下。代码中的唯一引脚来源是
[`components/robot_controller/board_pins.h`](components/robot_controller/board_pins.h)。

| 功能 | GPIO | 协议/方向 | 说明 |
|---|---:|---|---|
| DRV8833 nSLEEP | 3 | 数字输出 | 低电平使 H 桥休眠 |
| DRV8833 nFAULT | 4 | 数字输入 | 低有效故障输入 |
| DRV8833 AIN1 | 46 | LEDC PWM | Motor A |
| DRV8833 AIN2 | 9 | LEDC PWM | Motor A |
| DRV8833 BIN1 | 5 | LEDC PWM | Motor B |
| DRV8833 BIN2 | 6 | LEDC PWM | Motor B |
| 舵机 PWM | 45 | LEDC，50 Hz | 初始化阶段不输出脉冲 |
| MPU6050 INT | 8 | 数字输入 | 预留 DATA_READY 中断 |
| MPU6050 SCL | 11 | I2C，400 kHz | 需要合适的外部上拉 |
| MPU6050 SDA | 12 | I2C，400 kHz | 需要合适的外部上拉 |
| 外部接口控制 | 35 | 数字输出 | 高电平导通 2N7002，使外部接口拉低 |
| BOOT | 0 | 启动绑带输入 | 手动进入下载模式 |

GPIO3、GPIO45 和 GPIO46 是 ESP32-S3 启动绑带引脚。复位采样完成后可以作为普通
GPIO 使用，但外部电路不能在上电或复位采样期间强制这些引脚进入错误电平。

## 3. 软件架构

应用层已经从单个大文件拆分成职责明确的模块：

```text
浏览器 / 桌面上位机                 UART0
          │ HTTP                       │ 文本命令
          ▼                            ▼
     wifi_control                 app_console
          │                            │
          └──── robot_command_t ───────┘
                         │
                         ▼
                  app_control 命令队列
                         │
                         ▼
               control_task（唯一执行者）
                         │
                         ▼
                   control_manager
                 状态机 / 看门狗 / 斜坡
                         │
                         ▼
                    app_actuators
                 电机 / 舵机 / GPIO35

MPU6050 ── I2C ──► app_imu ──► app_status ◄── control_task
                                      │
                                      └── HTTP/UART 只读快照
```

### `main/` 应用编排层

| 文件 | 职责 |
|---|---|
| `app_main.c` | 只负责安全启动顺序和模块装配 |
| `app_actuators.c` | 将控制管理器的抽象操作适配到 DRV8833、舵机和 GPIO35 |
| `app_control.c` | 统一命令队列、队列溢出处理和 100 Hz 控制任务 |
| `app_imu.c` | I2C 初始化、MPU6050 探测和 100 Hz 采样任务 |
| `app_console.c` | UART0 初始化、命令解析和诊断输出 |
| `app_status.c` | 在短临界区内维护跨任务状态快照 |

### `components/robot_controller/` 可复用核心

这里包含板级驱动、机器人状态机、控制管理器、NVS 配置和 Wi-Fi/HTTP 服务。核心控制
逻辑通过执行器回调注入，因此可以在电脑上使用假硬件运行 C 测试。

并发规则如下：

1. HTTP 和 UART 只能生成 `robot_command_t` 并投递队列；
2. 只有 `control_task` 可以改变状态机并调用执行器；
3. `sensor_task` 只访问 MPU6050 并发布传感器状态；
4. HTTP 和 UART 查询只复制状态快照，不直接读取 I2C 或执行器；
5. 状态锁内禁止 I2C、网络、日志和其他可能阻塞的操作。

更完整的设计说明见 [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md)。

## 4. 安全状态机

```text
上电
 │
 ▼
BOOT ──初始化完成──► SAFE ──ARM──► READY ──运动命令──► RUNNING
                       ▲              │                    │
                       │              └──── DISARM ────────┘
                       │
                CLEAR FAULT
                       │
                  FAULT / ESTOP
```

- `BOOT`：驱动和任务正在初始化；
- `SAFE`：默认状态，运动命令会被拒绝；
- `READY`：已经 ARM，但当前没有运动输出；
- `RUNNING`：存在有效运动目标；
- `FAULT`：硬件或内部故障锁存；
- `ESTOP`：急停锁存，清除后只返回 `SAFE`，不会自动 ARM。

上电时固件按以下顺序建立安全状态：

1. GPIO35 外部接口释放；
2. DRV8833 休眠且电机目标为零；
3. 舵机 PWM 保持关闭，不会自动转到 90°；
4. 状态机进入 `SAFE`；
5. 最后才开放 Wi-Fi/HTTP 控制入口。

`ESTOP` 和 `DISARM` 是队列高优先级命令。普通队列即使已满，也会清除未执行的普通命令，
优先保留安全命令。运动命令默认需要在 1000 ms 内持续刷新；超时后执行统一硬件安全
停止。故障清除不会恢复之前的运动目标。

STA 断线不会立即无限循环扫描，而是按照 2、4、8、16、30 秒逐步退避并在 30 秒封顶。
断开原因、累计重连次数和下一次重连延迟会出现在日志及 `/api/v1/status` 中；连接成功后
退避清零。这样即使目标路由器长期离线，救援 SoftAP 仍有稳定的广播和接入窗口。

## 5. Wi-Fi 工作方式

固件使用 APSTA 模式，同时提供两条连接路径。

### 5.1 局域网 STA（推荐）

ESP32 连接本地 2.4 GHz Wi-Fi，电脑也连接同一局域网。这样电脑可以同时访问互联网和
ESP32，不需要在机器人热点与正常网络之间切换。

局域网账号保存在本地文件：

```text
components/robot_controller/wifi_credentials.h
```

首次配置时复制模板：

```powershell
Copy-Item components\robot_controller\wifi_credentials.example.h `
          components\robot_controller\wifi_credentials.h
```

然后只编辑本地文件：

```c
#pragma once
#define ROBOT_WIFI_STA_SSID     "你的局域网名称"
#define ROBOT_WIFI_STA_PASSWORD "你的局域网密码"
```

`wifi_credentials.h` 已被 Git 忽略，**不要把真实密码提交到 GitHub**。启动日志和串口
`status` 命令会显示 DHCP 分配的 IP，例如 `192.168.1.123`。电脑随后访问：

```text
http://192.168.1.123/
```

### 5.2 救援 SoftAP

STA 未配置或局域网不可用时，SoftAP 仍然启动：

- SSID：`ESP32-Robot`
- 密码：`esp32robot`
- 地址：[http://192.168.4.1](http://192.168.4.1)

电脑只有一张 Wi-Fi 网卡时，连接该热点通常会暂时失去互联网，因此日常开发优先使用
STA 局域网方式。

## 6. 开发环境

推荐环境：

- Windows 10/11；
- ESP-IDF 5.2.6；
- Python 3；
- Git；
- CH340 驱动；
- 支持数据传输的 USB Type-C 线。

先打开 ESP-IDF PowerShell，或者执行 ESP-IDF 安装目录中的 `export.ps1`。确认：

```powershell
idf.py --version
python --version
```

编译不需要连接开发板。只有查找端口、烧录和串口监视需要开发板。

## 7. 编译

首次构建：

```powershell
cd D:\esp32_project
idf.py set-target esp32s3
idf.py build
```

仓库也提供批处理入口：

```powershell
.\build.bat
```

主要产物位于 `build/`：

```text
bootloader/bootloader.bin
partition_table/partition-table.bin
esp32_robot_controller.bin
esp32_robot_controller.elf
```

需要隔离 Debug 和 Release 构建时使用独立目录：

```powershell
idf.py -B build/debug -D SDKCONFIG=build/debug/sdkconfig `
  -D SDKCONFIG_DEFAULTS=sdkconfig.defaults build

idf.py -B build/release -D SDKCONFIG=build/release/sdkconfig `
  -D "SDKCONFIG_DEFAULTS=sdkconfig.defaults;sdkconfig.release.defaults" build
```

## 8. 烧录和串口监视

查看 Windows 当前串口：

```powershell
[System.IO.Ports.SerialPort]::GetPortNames()
```

假设开发板为 `COM7`：

```powershell
idf.py -p COM7 flash monitor
```

或者：

```powershell
.\build.bat COM7
idf.py -p COM7 monitor
```

退出监视器通常使用 `Ctrl+]`。

旧 PCB 的自动下载 DTR/RTS 时序不稳定。如果出现 `Wrong boot mode` 或无法进入下载模式，
可以手动操作：

1. 按住 `BOOT`；
2. 短按并松开 `EN/RST`；
3. 松开 `BOOT`；
4. 再执行烧录；
5. 烧录结束后，在不按 BOOT 的情况下短按 `EN/RST` 正常启动。

端口不存在时先确认：开发板供电、Type-C 线具备数据能力、CH340 驱动正常、设备管理器
出现对应 COM 端口。编译成功与电脑是否识别串口无关。

## 9. 串口命令

UART0 参数为 `115200 8N1`。可用命令：

```text
help
status
imu

arm
disarm
estop
clear

drive <throttle -100..100> <steering -100..100>
motors <motor_a -100..100> <motor_b -100..100>
servo <0..180>
power <0|1>
```

示例：

```text
status
arm
drive 15 0
drive 0 0
disarm
```

首次接新 PCB 时不要直接照抄运动示例。应先断开电机/舵机负载，完成分阶段检查。

## 10. HTTP API v1

所有改变状态的 v1 请求使用 `POST`，查询使用 `GET`。HTTP 回调只负责参数校验和命令
入队，不会直接操作硬件。

| 请求 | 作用 | JSON 请求体 |
|---|---|---|
| `GET /api/v1/status` | 状态、执行器、IMU、网络和诊断计数 | 无 |
| `GET /api/v1/device` | 固件、芯片和构建信息 | 无 |
| `POST /api/v1/arm` | 从 SAFE 解锁 | 无 |
| `POST /api/v1/disarm` | 锁定并安全停止 | 无 |
| `POST /api/v1/estop` | 锁存急停并安全停止 | 无 |
| `POST /api/v1/fault/clear` | 故障消失后返回 SAFE | 无 |
| `POST /api/v1/drive` | 油门与转向差速混控 | `{"throttle":20,"steering":-10}` |
| `POST /api/v1/motors` | Motor A/B 独立控制 | `{"motor_a":20,"motor_b":-20}` |
| `POST /api/v1/servo` | 设置舵机目标角度 | `{"angle":90}` |
| `POST /api/v1/power` | 控制 GPIO35 外部接口 | `{"asserted":true}` |

PowerShell 调用示例：

```powershell
$device = 'http://192.168.1.123'

Invoke-RestMethod "$device/api/v1/status"
Invoke-RestMethod -Method Post "$device/api/v1/arm"
Invoke-RestMethod -Method Post -ContentType 'application/json' `
  -Body '{"throttle":15,"steering":0}' "$device/api/v1/drive"
Invoke-RestMethod -Method Post "$device/api/v1/disarm"
```

差速混控规则：

```text
motor_a = clamp(throttle + steering, -100, 100)
motor_b = clamp(throttle - steering, -100, 100)
```

完整响应字段、错误语义和兼容接口见
[`docs/CONTROL_PROTOCOL_V1.md`](docs/CONTROL_PROTOCOL_V1.md)。

## 11. 桌面上位机

最简单的启动方式是在仓库根目录双击：

```text
start_robot_control.bat
```

它会先验证已知地址和救援地址，再并发扫描电脑当前 `/24` 局域网中的只读
`GET /api/v1/device` 接口。识别到本项目的 ESP32-S3 后，会自动把地址传给图形上位机。
发现过程不会 ARM，也不会发送任何运动命令。

也可以从终端一键发现并启动：

```powershell
python tools\start_robot_control.py
```

仅检查设备发现、不打开窗口：

```powershell
python tools\start_robot_control.py --find-only
```

如果自动扫描受防火墙或特殊子网影响，可以指定地址：

```powershell
python tools\start_robot_control.py --url http://172.26.96.61
```

上位机只使用 Python 标准库和 Tkinter：

```powershell
python tools\robot_control_gui.py --url http://192.168.1.123
```

使用救援热点时可以省略地址，默认连接 `http://192.168.4.1`：

```powershell
python tools\robot_control_gui.py
```

上位机提供 ARM、停止、急停、状态轮询和基础运动控制。它不能绕过固件的安全状态机；
即使上位机异常退出，固件的命令超时看门狗仍会停止电机。

## 12. 无硬件测试

只运行 Python 静态和协议测试：

```powershell
python -m unittest discover -s tests -p "test_*.py"
```

运行完整 P0 预检，包括 Python 测试、主机原生 C 测试和 Git whitespace 检查：

```powershell
python tools\p0_preflight.py
```

同时执行 ESP-IDF 构建：

```powershell
python tools\p0_preflight.py --idf-build
```

验收规则和每项证据见
[`docs/P0_ACCEPTANCE_MATRIX.md`](docs/P0_ACCEPTANCE_MATRIX.md)。

### 联网后的无运动冒烟测试

以下命令会自动发现 ESP32，验证设备信息、状态查询、ARM、DISARM、ESTOP 和 CLEAR，
但不会发送电机、舵机或 GPIO35 输出：

```powershell
python tools\robot_smoke_test.py
```

结束时工具强制确认设备回到 `SAFE`。真正的输出测试必须显式指定唯一阶段，例如：

```powershell
python tools\robot_smoke_test.py --allow-motion --test motor-a
```

随后还必须在终端输入大写 `MOVE`。Motor A/B 阶段限制为 10% 输出、0.6 秒，并在
`finally` 路径中归零和 DISARM。`--yes` 只允许在具有独立急停和限流电源的自动化台架使用。

## 13. 最新 PCB 首次上电流程

建议使用限流电源，并让轮子悬空或拆除机械负载。测试顺序：

1. 不接电机和舵机，检查短路、3.3 V/电机电源轨和静态电流；
2. 只连接 USB，验证 CH340、下载、复位和串口启动；
3. 在 `SAFE` 状态验证 MPU6050、I2C 和 100 Hz 采样；
4. 验证 DRV8833 nFAULT 的空闲电平和故障上报；
5. 验证 GPIO35/2N7002 的实际高低电平关系；
6. 单独给舵机供电，限制机械行程后验证 PWM 和方向；
7. 显式 ARM，以低占空比测试 Motor A；
8. 停止并 DISARM，再以同样方法测试 Motor B；
9. 低速测试双电机、方向、斜坡和反转换向死区；
10. 验证 ESTOP、DISARM、拔掉网络和停止刷新命令都能立即或超时停车；
11. 电机带载时观察电源跌落、ESP32 复位、Wi-Fi 掉线和 IMU 噪声。

旧 PCB 引脚可能与最新原理图不同，不应使用旧板进行未确认的外设运动测试。

## 14. 常见问题

### 编译需要连接开发板吗？

不需要。编译只需要 ESP-IDF 工具链和源码。烧录、串口监视和真机测试才需要开发板。

### 为什么连接 `ESP32-Robot` 后电脑不能上网？

该热点由 ESP32 提供，通常没有互联网出口。请配置 STA，让 ESP32 和电脑同时连接同一个
局域网，然后通过 ESP32 的 DHCP 地址访问。

### 为什么插上电源和 Type-C 仍没有 COM 端口？

外部 3.9 V 电源只解决板级供电，不代表 CH340 已经通过 USB 枚举。依次检查数据线、
Type-C 接口焊接、CH340 供电/晶振/USB D+/D−、驱动和 Windows 设备管理器。用另一块最小
系统板测试成功，只能证明电脑端口和数据线正常。

### 为什么发送电机或舵机命令没有动作？

先执行 `status`。如果状态是 `SAFE`、`FAULT` 或 `ESTOP`，运动命令会被拒绝。正常流程是
先确认硬件安全，再发送 `arm`。若存在 nFAULT 或超时锁存，需要排除原因后 `clear`，然后
重新 ARM。

### 为什么电机运行一秒左右就停？

这是预期的通信看门狗行为。非零运动命令必须持续刷新，默认超时为 1000 ms。不要通过
简单增大超时掩盖网络或上位机问题。

### 为什么舵机启动后没有自动回中？

这是安全设计。启动阶段不输出舵机 PWM，避免机械结构突然动作。只有 ARM 后收到合法的
舵机命令，PWM 才会显式启用。

## 15. 目录结构

```text
main/                            应用编排层和 FreeRTOS 任务
components/robot_controller/     板级驱动、状态机、控制核心、配置和 Wi-Fi
hardware/schematics/             当前原理图
hardware/manufacturing/          制造资料和版本说明
docs/                            架构、协议、路线图和验收记录
tools/                           桌面上位机及自动化工具
tests/                           无硬件 Python/C 测试
reference/legacy_embedded/       精选旧工程参考，不参与当前构建
build/                           本机构建产物，不提交 Git
sdkconfig.defaults               公共 ESP-IDF 默认配置
sdkconfig.release.defaults       Release 附加配置
```

## 16. 后续 TODO

当前优先级：

- [ ] 烧录最新模块化固件，通过串口确认 STA 成功连接局域网；
- [ ] 在电脑保持联网的情况下验证控制页及全部 HTTP v1 接口；
- [ ] 使用最新 PCB 完成 USB、IMU、GPIO35、舵机、单电机和双电机分阶段测试；
- [ ] 使用 GPIO8 DATA_READY 中断，并增加 I2C 恢复和丢帧统计；
- [ ] 完成 IMU 单位换算、零偏校准、低通滤波和 NVS 校准参数；
- [ ] 完善上位机中文提示、实时状态和 Xbox/XInput 控制；
- [ ] 增加默认禁止运动、显式 `--allow-motion` 的硬件测试工具；
- [ ] 模块化版本重新执行 Debug/Release 独立干净构建；
- [ ] OTA、故障记录持久化和姿态融合留到 P2。

完整路线图见 [`docs/ROADMAP.md`](docs/ROADMAP.md)。

## 17. 文档索引

- [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md)：软件架构概要；
- [`docs/CONTROL_PROTOCOL_V1.md`](docs/CONTROL_PROTOCOL_V1.md)：冻结的 HTTP v1 协议；
- [`docs/PRE_HARDWARE_SOFTWARE_FOUNDATION.md`](docs/PRE_HARDWARE_SOFTWARE_FOUNDATION.md)：接板前软件基础设计；
- [`docs/P0_ACCEPTANCE_MATRIX.md`](docs/P0_ACCEPTANCE_MATRIX.md)：P0 验收证据和命令；
- [`docs/HARDWARE_SMOKE_TEST_2026-08-01.md`](docs/HARDWARE_SMOKE_TEST_2026-08-01.md)：旧 PCB 冒烟测试记录；
- [`docs/ROADMAP.md`](docs/ROADMAP.md)：P0/P1/P2 开发计划。
