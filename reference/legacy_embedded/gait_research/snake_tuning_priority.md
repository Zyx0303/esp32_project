# 蛇形步态调参优先级表

## 1. 适用前提

这份表基于当前 `STM32` 工程源码整理：

- [main.c](/E:/Fudan/worms/codes/USER/main.c)
- [usartx.c](/E:/Fudan/worms/codes/HARDWARE/usartx.c)
- [usartx.h](/E:/Fudan/worms/codes/HARDWARE/usartx.h)
- [system.h](/E:/Fudan/worms/codes/BALANCE/system.h)

并采用你刚刚补充的前提：

- `baseOffset = {0, 2, 28, -40}` 是已经修正过的
- 机器人在不运动时是直的

因此，这份表里会把 `baseOffset` 放到较低优先级，不再把它当作首要怀疑对象。

---

## 2. 当前生效参数

### 2.1 蛇形参与体节

| 项目 | 当前值 | 位置 | 说明 |
| --- | --- | --- | --- |
| 蛇形参与关节数 | `NUM_SNAKE_JOINTS = 4` | [usartx.h](/E:/Fudan/worms/codes/HARDWARE/usartx.h#L35) | 当前蛇形波只作用在 4 个蛇形关节上 |
| 起始体节号 | `FIRST_SNAKE_JOINT = 1` | [usartx.h](/E:/Fudan/worms/codes/HARDWARE/usartx.h#L36) | 表示蛇形从体节 `1` 开始 |
| 参与范围 | 体节 `1-4` | [main.c](/E:/Fudan/worms/codes/USER/main.c#L338) | 体节 `0` 不参与蛇形波 |

### 2.2 波形参数

| 项目 | 当前值 | 位置 | 说明 |
| --- | --- | --- | --- |
| 控制循环频率 | `20 Hz` | [system.h](/E:/Fudan/worms/codes/BALANCE/system.h#L41), [main.c](/E:/Fudan/worms/codes/USER/main.c#L296) | 每 `50 ms` 更新一次目标 |
| 步态频率 | `0.8 Hz` | [main.c](/E:/Fudan/worms/codes/USER/main.c#L298) | 当前偏稳，速度偏保守 |
| 每周期相位步进 | `0.8 * 2pi / 20 = 0.2513 rad` | [main.c](/E:/Fudan/worms/codes/USER/main.c#L300) | 约 `14.4°/tick` |
| 直线蛇形相邻相位差 | `pi/3 = 60°` | [main.c](/E:/Fudan/worms/codes/USER/main.c#L347) | 当前不是 `90°` |
| 摆角公式 | `target = baseOffset + swingAngleLimit * sin(phase + phase_offset)` | [main.c](/E:/Fudan/worms/codes/USER/main.c#L353), [main.c](/E:/Fudan/worms/codes/USER/main.c#L354) | 没有单独的方向符号表 |

### 2.3 舵机命令参数

| 项目 | 当前值 | 位置 | 说明 |
| --- | --- | --- | --- |
| 输出接口 | `FSUS_SetServoAngleByInterval` | [main.c](/E:/Fudan/worms/codes/USER/main.c#L412) | 当前已经不用 `ByVelocity` 跑连续蛇形 |
| `interval` | `60 ms` | [main.c](/E:/Fudan/worms/codes/USER/main.c#L413) | 比 `50 ms` 控制周期还长，偏保守 |
| `t_acc` | `20 ms` | [main.c](/E:/Fudan/worms/codes/USER/main.c#L414) | 已经是库允许的最小值 |
| `t_dec` | `20 ms` | [main.c](/E:/Fudan/worms/codes/USER/main.c#L415) | 已经是库允许的最小值 |
| `power` | `0` | [main.c](/E:/Fudan/worms/codes/USER/main.c#L416) | 使用默认保护设置 |
| 库最小加减速时间 | `20 ms` | [fashion_star_uart_servo.c](/E:/Fudan/worms/codes/HARDWARE/fashion_star_uart_servo.c#L412), [fashion_star_uart_servo.c](/E:/Fudan/worms/codes/HARDWARE/fashion_star_uart_servo.c#L415) | 再小也会被夹到 `20 ms` |

### 2.4 振幅参数来源

| 项目 | 当前值 | 位置 | 说明 |
| --- | --- | --- | --- |
| 协议换算 | `parameter * 90 / 255` | [usartx.c](/E:/Fudan/worms/codes/HARDWARE/usartx.c#L347) | 上位机发 `0-255`，板上还原成 `0-90°` |
| 手柄默认直线蛇形振幅 | `45°` | [xbox_robot_control.py](/E:/Fudan/worms/codes/xbox_robot_control.py#L420) | 当前手柄 `A` 键固定发 `45°` |
| 图形上位机振幅范围 | `0-90°` | [robot_v2_no_imu.py](/E:/Fudan/worms/codes/robot_v2_no_imu.py#L64), [robot_v2_no_imu.py](/E:/Fudan/worms/codes/robot_v2_no_imu.py#L67) | 可以手动试更大振幅 |

### 2.5 当前偏置

| 项目 | 当前值 | 位置 | 说明 |
| --- | --- | --- | --- |
| `baseOffset` | `{0, 2, 28, -40}` | [usartx.c](/E:/Fudan/worms/codes/HARDWARE/usartx.c#L434) | 你已确认静止时机器人是直的 |

---

## 3. 调参优先级

### 优先级 1: 直线蛇形相位差

| 项目 | 当前值 | 建议尝试 | 主要改善 | 原因 |
| --- | --- | --- | --- | --- |
| 相邻关节相位差 | `60°` | `75° -> 90°` | 前进速度、走直性 | 当前 `60°` 更容易形成“姿态像蛇，但推进波弱” |

建议顺序：

1. 先试 `75°`
2. 如果推进还是弱，再试 `90°`
3. 每次只改这一项，其他不动

说明：

- 当前仓库说明里的直线推进模型本来就是按 `90°` 相邻相位差理解的，见 [README.md](/E:/Fudan/worms/codes/README.md#L25)
- 这项是我认为最优先的，因为它同时影响 `速度` 和 `直线性`

### 优先级 2: 舵机 `interval`

| 项目 | 当前值 | 建议尝试 | 主要改善 | 原因 |
| --- | --- | --- | --- | --- |
| `FSUS_SetServoAngleByInterval.interval` | `60 ms` | `50 ms -> 45 ms -> 40 ms` | 前进速度、波形跟踪一致性 | 当前舵机到位时间比控制周期 `50 ms` 还长，存在持续滞后 |

建议顺序：

1. 先试 `50 ms`
2. 再试 `45 ms`
3. 如果舵机仍然跟得上且不抖，再试 `40 ms`

说明：

- `t_acc/t_dec` 现在已经是最小 `20 ms`，优先改 `interval`
- 这项会直接影响“机器人看起来摆得出来，但整体推进慢”的问题

### 优先级 3: 步态频率

| 项目 | 当前值 | 建议尝试 | 主要改善 | 风险 |
| --- | --- | --- | --- | --- |
| `frequency` | `0.8 Hz` | `1.0 Hz -> 1.2 Hz` | 前进速度 | 频率上去后，如果舵机跟踪不一致，跑偏可能反而更明显 |

建议顺序：

1. 在前两项基本稳定后，再把频率从 `0.8` 提到 `1.0`
2. 如果推进明显改善且不散，再试 `1.2`
3. 不建议一开始就提得太高

说明：

- 频率调高通常最直接地带来“更快”
- 但如果相位差和舵机跟踪还没调顺，频率越高越容易把左右不一致放大

### 优先级 4: 振幅

| 项目 | 当前值 | 建议尝试 | 主要改善 | 风险 |
| --- | --- | --- | --- | --- |
| `swingAngleLimit` | 手柄默认 `45°` | `50° -> 55° -> 60°` | 前进速度、推进力 | 振幅过大后容易磨地、侧滑、跑偏加剧 |

建议顺序：

1. 先保留 `45°` 作为基线
2. 再试 `50°`
3. 如果地面摩擦够、舵机也跟得上，再试 `55-60°`

说明：

- 这项对速度有效，但对“走直”不一定天然有利
- 所以它排在相位差和舵机 `interval` 后面

### 优先级 5: 参与体节范围

| 项目 | 当前值 | 可考虑方向 | 主要改善 | 代价 |
| --- | --- | --- | --- | --- |
| 蛇形只作用体节 `1-4` | 是 | 评估是否让第 `0` 节也参与 | 推进效率、直线性 | 需要改逻辑，不属于纯调参 |

说明：

- 这不是简单参数，而是控制结构问题
- 但如果前 4 项都调完还觉得推进弱，这一项值得重新评估

### 优先级 6: `baseOffset`

| 项目 | 当前值 | 当前判断 | 何时再动 |
| --- | --- | --- | --- |
| `baseOffset = {0, 2, 28, -40}` | 已修正 | 暂时降级处理 | 只有在“静止直，但运动一开就稳定偏同一侧”时，再做动态微调 |

说明：

- 你已经确认“静止时机器人是直的”，所以它不再是第一优先级
- 但如果所有波形参数都调完，仍然总是向固定一侧偏，最后还是可能要给某一节做 `1-5°` 级别的小补偿

---

## 4. 推荐调参顺序

建议按下面顺序做，避免多变量一起改：

1. 先固定振幅 `45°`，频率 `0.8 Hz` 不动
2. 只改相位差：`60° -> 75° -> 90°`
3. 确定一个相位差后，只改 `interval`：`60 -> 50 -> 45 -> 40 ms`
4. 确定上面两项后，再改频率：`0.8 -> 1.0 -> 1.2 Hz`
5. 最后再试振幅：`45 -> 50 -> 55 -> 60°`
6. 如果仍然总向固定一侧偏，再回头微调 `baseOffset`

---

## 5. 我建议的第一轮实验矩阵

为了省时间，第一轮不要做全排列，直接试这 4 组：

| 组别 | 相位差 | `interval` | 频率 | 振幅 |
| --- | --- | --- | --- | --- |
| A | `60°` | `60 ms` | `0.8 Hz` | `45°` |
| B | `75°` | `60 ms` | `0.8 Hz` | `45°` |
| C | `75°` | `50 ms` | `0.8 Hz` | `45°` |
| D | `90°` | `50 ms` | `0.8 Hz` | `45°` |

判断标准只看两件事：

- 哪一组前进速度明显更快
- 哪一组从上方看传播波最像“沿身体向后传”，而不是整体左右甩

如果 `C` 或 `D` 明显优于 `A`，说明主问题基本就在：

- 相位差偏小
- 舵机 `interval` 偏长

---

## 6. 简化结论

在你已经确认 `baseOffset` 校正过的前提下，我建议当前按这个优先级处理：

1. `相位差`
2. `舵机 interval`
3. `频率`
4. `振幅`
5. `参与体节范围`
6. `baseOffset`

如果只允许先动一个参数，我建议先动：

- `直线蛇形相位差: 60° -> 75°`

如果允许同时动两个参数，我建议先动：

- `相位差: 60° -> 75°`
- `interval: 60 ms -> 50 ms`
