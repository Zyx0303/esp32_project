# 项目待办

更新日期：2026-09-14

## 振动电机闭环控制（待扫频测试后实现）

当前保留策略接口，没有注册具体闭环算法。手动直走、转向和双电机独立控制保持现状。
待扫频测试及控制方案确定后，再接入基于 IMU 的反馈控制。

### 接口位置

| 文件 | 接口及作用 |
| --- | --- |
| [control_strategy.h](components/robot_controller/control_strategy.h) | 定义 `robot_control_strategy_ops_t`，其中 `step()` 负责根据状态计算下一周期目标；`init()`、`reset()` 为可选回调。 |
| [app_control.h](main/app_control.h) | `app_control_register_strategy(ops, context)`：注册策略，必须在 `app_control_start()` 之前调用。 |
| [app_control.c](main/app_control.c) | `run_registered_strategy()`：ARM 后在控制任务中执行策略，名义周期为 10 ms（100 Hz）。 |
| [app_main.c](main/app_main.c) | 应用启动入口；后续在调用 `app_control_start(&stored_config)` 之前注册策略。 |

### 输入与输出

- 输入 `input->status`：包含 IMU 原始值 `imu_ax/ay/az`、`imu_gx/gy/gz`、`imu_temp_raw`，以及 `imu_valid`、`imu_last_update_us` 和电机状态。
- 时间输入：`input->now_us` 和 `input->delta_us`，单位为微秒。
- 独立电机输出：设置 `output->motor_mode = ROBOT_STRATEGY_MOTOR_DIRECT`，填写 `output->motor.motors.motor_a`、`motor_b`，范围为 −100～100% 的占空比目标。
- 混控输出：设置 `ROBOT_STRATEGY_MOTOR_DRIVE`，填写 `output->motor.drive.throttle`、`steering`。
- 停止输出：设置 `ROBOT_STRATEGY_MOTOR_STOP`；输出结构在每次回调前清零，默认即为停止。

策略输出由控制任务转成命令，经过现有安全状态机、范围检查、斜坡、换向等待和看门狗后送往执行器。
策略不能直接操作 PWM、GPIO 或 I²C；`step()` 必须快速返回，不得阻塞或延时。

### 后续工作

- [ ] 完成双电机扫频测试，记录占空比与 IMU 响应，检查采样时间间隔、丢帧及饱和情况。
- [ ] 确认 IMU 安装坐标系，完成单位转换、零偏校准及所需滤波。
- [ ] 根据测试结果确定反馈量、直走与转向控制方案及参数。
- [ ] 明确手动命令与闭环策略的切换方式；当前注册策略会在 ARM 后每周期提交电机目标，需要避免覆盖手动控制意图。
- [ ] 实现策略 `init()`、`reset()`、`step()`，在启动控制任务之前注册。
- [ ] 在策略中检查 `imu_valid` 和时间戳，定义 IMU 失效或数据过期时的停止行为；不要假定传感器失效已自动触发电机停车。
- [ ] 验证输出限幅、反转换向、解除 ARM、急停及策略错误处理。
- [ ] 完成离线算法验证后，再进行低占空比真机测试。

## 桌面 GUI

- [x] 背景改为学术风浅天蓝色。
- [x] 左上角放置用户提供的复旦校徽，资源位于 `tools/assets/fudan_logo.png`。
- [x] 右上角显示加粗 `BSRL`，下方显示“仿生结构与机器人实验室”。
- [x] 两侧标识放大一倍：校徽显示尺寸翻倍，BSRL 字号 26 → 52，实验室名称字号 9 → 18；标题下移，优先保留标识空间。
- [x] 检查中文字体和最小窗口宽度下的页眉布局。

界面修改后需要关闭旧窗口并重新运行 `python tools/start_robot_control.py`。

- [x] 左上角复旦校徽显示区域调整为固定 640 × 640 像素，等比例居中显示；右侧 BSRL 保持原放大尺寸。

- [x] 协调大校徽布局：左侧独立品牌栏保留 640 × 640 校徽，右侧集中展示 BSRL、设备连接和实验控制；默认窗口按内容尺寸展开，并完成实窗截图检查。
- [x] 将 640 × 640 校徽改成与天蓝页面背景融合的低对比度水印，移除左侧白色海报卡、装饰标题和分隔线，使控制区成为视觉主体。
