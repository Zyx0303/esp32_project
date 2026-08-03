# P0 软件基础验收矩阵

本矩阵把 [`PRE_HARDWARE_SOFTWARE_FOUNDATION.md`](PRE_HARDWARE_SOFTWARE_FOUNDATION.md)
中的“接板前完成”要求转为可重复执行的证据。P0 验收只证明无硬件的软件基础；它不证明
GPIO 电气连接、电机方向、舵机机械行程或 IMU 噪声已经通过真机验证。

## 一键预检

在仓库根目录执行：

```powershell
python tools/p0_preflight.py
```

激活 ESP-IDF 5.2.6 环境后，可同时编译：

```powershell
python tools/p0_preflight.py --idf-build
```

任意必要门禁失败时脚本返回非零退出码。Python 测试只使用标准库；控制核心行为测试另需
本机 `gcc`、`clang` 或 `tcc`，可用 `--cc <编译器路径>` 指定。全部测试均不需要连接开发板。

## 验收等级

- **自动**：预检脚本可直接判定，适合每次修改运行。
- **构建**：由 ESP-IDF 编译产物和命令退出码证明。
- **审查**：需要对实现和测试覆盖进行人工核对；不得用“存在某个文件”代替。
- **真机后置**：P0 不宣称完成，留到接板测试。

## P0 要求、证据与判定

| ID | 要求 | 自动化证据 | 补充证据/命令 | 通过标准 |
|---|---|---|---|---|
| P0-01 | BOOT、SAFE、READY、RUNNING、FAULT、ESTOP 状态齐全 | Python 静态测试；原生 C 控制核心测试 | 审查 `robot_state_machine.c` | 非法转换返回错误；只有 SAFE 可 ARM |
| P0-02 | 上电默认电机休眠、双电机为零、舵机不输出、GPIO35 释放 | `test_safe_stop_neutralizes_all_motion_outputs`、`test_boot_does_not_command_servo_center`、`test_power_defaults_to_released` | 审查 `app_main` 初始化调用顺序 | 启动路径没有 90° 舵机命令；`control_manager_init` 首次动作是安全停止 |
| P0-03 | 所有运动经过 ARM、范围校验和状态机 | Python 静态测试；原生 C 的 ARM 前拒绝/ARM 测试 | 审查 HTTP/串口只投递 `robot_command_t` | SAFE/FAULT/ESTOP 下 DRIVE、MOTOR_DIRECT、SERVO 均拒绝 |
| P0-04 | ESTOP、DISARM、通信超时、nFAULT 共用硬件安全停止 | 静态 fan-in 测试；原生 C 的 ESTOP、watchdog、硬件故障测试 | 审查 `safe_stop` 的执行器操作 | 四条路径均清零电机、休眠 DRV8833、关闭舵机 PWM |
| P0-05 | ESTOP/DISARM 不会被过期、乱序或满队列挡住 | `test_safety_commands_precede_stale_and_sequence_rejection`、`test_full_queue_cannot_discard_estop_or_disarm` | 审查命令队列优先级 | 即使时间戳/序号无效或队列已满，安全命令仍先执行 |
| P0-06 | 双电机独立控制、差速混控、限幅、斜坡、反转换向死区 | `tools/run_p0_c_tests.py` 实际编译并执行 manager | `idf.py build`；审查 `control_manager_tick` | 边界 `-100/100`、斜坡和正反转过零死区测试通过 |
| P0-07 | 舵机限幅、脉宽换算、速率限制以及禁用输出 | 安全静态测试覆盖禁用路径 | C 单元测试/审查 `servo_pwm` 与 manager | 未 ARM 无脉冲；命令限幅；输出按配置速率变化 |
| P0-08 | 命令包含类型、来源、序号、时间戳且拒绝过期/乱序命令 | `test_unified_command_carries_sequence_time_and_source`、`test_protocol_exposes_required_command_types` | 审查 `control_manager_submit` | 每个来源独立递增；过期计数可查询 |
| P0-09 | HTTP 和串口使用同一内部命令模型，由单一控制任务执行 | `test_http_uses_unified_command_submission`、`test_single_control_task_owns_command_execution` | 审查串口适配器和 `control_task` | 两入口只入队；只有控制任务调用 manager 和执行器抽象 |
| P0-10 | v1 HTTP API 固定；改变状态只允许 POST | `test_v1_route_methods_are_frozen`、`test_mutating_v1_routes_never_use_get` | 检查协议文档与 GUI 调用 | 10 个 v1 路由方法完全匹配协议表 |
| P0-11 | 状态能解释不可运动原因 | 审查 `robot_status_t` 与 `/api/v1/status` JSON | 对 SAFE、FAULT、ESTOP 快照做离线/接口测试 | 至少包含 state、armed、estop、faults、目标/输出、拒绝与超时计数 |
| P0-12 | 配置有版本、逐字段校验和安全默认回退 | `test_config_has_version_validation_and_safe_fallback` | 审查 NVS load/save；无效 blob 测试 | 缺失、旧版本、错误大小、越界字段均使用默认值并可报告 |
| P0-13 | 新 P0 模块实际参与构建 | `test_required_p0_modules_are_registered` | `idf.py build` 链接成功 | CMake 列出 state machine、manager、config，且无未解析符号 |
| P0-14 | Debug/Release 可从干净目录构建且无新增警告 | 无硬件编译门禁 | 分别执行下方两条干净构建命令 | 两次退出码均为 0；日志没有新增 warning |
| P0-15 | 代码无 whitespace 错误 | `git diff --check`（预检自动运行） | 无 | 退出码为 0 |
| P0-16 | 固件、引脚、协议及验收命令已文档化 | 本文、`board_pins.h`、项目 README | 人工比对 2026-07-31 SVG | 文档与代码一致，不含已废弃蓝牙控制说明 |

## 干净构建命令

不要删除正在使用的构建目录；为两个配置使用独立目录：

```powershell
idf.py -B build/debug -D SDKCONFIG=build/debug/sdkconfig -D SDKCONFIG_DEFAULTS=sdkconfig.defaults build
idf.py -B build/release -D SDKCONFIG=build/release/sdkconfig -D "SDKCONFIG_DEFAULTS=sdkconfig.defaults;sdkconfig.release.defaults" build
```

Release 构建必须同时加载公共默认值和 `sdkconfig.release.defaults`，不能用已有增量 Debug 构建替代。

## P0 与真机验收边界

下列项目必须接硬件后验证，不作为本矩阵绿色状态的依据：

- CH340/USB 枚举、下载和复位时序；
- DRV8833 电机 A/B 实际方向、电流、nFAULT 电平与噪声；
- 舵机供电能力、真实角度范围和机械干涉；
- MPU6050 WHO_AM_I、中断、量程、零偏和采样抖动；
- Wi-Fi 射频距离以及电机运行时的掉线/复位情况。

## 验收记录模板

| 日期 | Git commit | 主机测试 | `diff --check` | Debug 构建 | Release 构建 | 固件大小 | 审查人 |
|---|---|---|---|---|---|---|---|
| YYYY-MM-DD | `<sha>` | PASS/FAIL | PASS/FAIL | PASS/FAIL | PASS/FAIL | `<bytes>` | `<name>` |
| 2026-08-01 | working tree @ `cfa5123` | PASS（18 Python + 原生 C） | PASS | PASS | PASS | Debug 844800；Release 699712 | Codex + 多代理交叉审查 |
| 2026-08-03 | pre-PCB working tree | PASS（34 Python + 原生 C + HTTP 真机冒烟） | PASS | PASS | PASS | Debug 849392；Release 703968 | Codex |

只有 P0-01 至 P0-16 都有充分证据时，才能把 P0 标记为完成。静态测试通过不等价于全部 P0 完成。
