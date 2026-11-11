# ESP32-S3 光板手工核对 TODO

使用方式：完成一项后把 `[ ]` 改为 `[x]`；发现问题直接在该项后补一句现象。

## 启动与串口

- [ ] COM10 能被电脑稳定识别
- [ ] 未按 BOOT 时，按 EN/RST 能正常从 Flash 启动
- [ ] 正常启动显示 `SPI_FAST_FLASH_BOOT`，不会停在 `waiting for download`
- [ ] 使用 `monitor --no-reset` 不会让板子进入下载模式
- [ ] 串口波特率 `115200`，日志清晰且没有持续乱码
- [ ] 连续复位 3 次都能正常进入应用
- [ ] 启动日志没有 panic、assert 或 watchdog reset
- [ ] 串口 `help` 命令正常
- [ ] 串口 `status` 命令正常

## Wi-Fi 与 HTTP

- [ ] ESP32 能连接 BSRL
- [ ] ESP32 能获取局域网 IP
- [ ] 电脑端扫描器能发现 ESP32
- [ ] `/api/v1/device` 能正常返回设备信息
- [ ] `/api/v1/status` 能正常返回完整 JSON
- [ ] Python GUI 能连接并持续刷新状态
- [ ] 关闭并重新打开 GUI 后仍能连接
- [ ] ESP32 断开 BSRL 后会自动尝试重连
- [ ] BSRL 恢复后 ESP32 无需复位即可重新连接
- [ ] 救援热点 `ESP32-Robot` 能正常出现
- [ ] 偶发 `httpd_sock_err: 104` 不影响下一次 HTTP 请求
- [ ] HTTP 请求不会连续超时或导致 ESP32 重启

## IMU 基础通信

- [ ] MPU6050 初始化成功，地址为 `0x68`
- [ ] 串口 `imu` 能返回数据
- [ ] Wi-Fi `/status` 能返回 IMU 数据
- [ ] `imu.valid` 为 `true`
- [ ] `imu.samples` 持续增加
- [ ] `imu.errors` 不持续增加
- [ ] 不会连续出现 `mpu6050_read_raw: read raw`
- [ ] 串口与 Wi-Fi 返回的轴方向、正负号和数量级一致

## IMU 静态与动态响应

- [ ] 板子静止时，gyro X 接近固定零偏
- [ ] 板子静止时，gyro Y 接近固定零偏
- [ ] 板子静止时，gyro Z 接近固定零偏
- [ ] 板子静止时，加速度合量接近 1g
- [ ] +X/-X 翻面时，X 轴符号正确反转
- [ ] +Y/-Y 翻面时，Y 轴符号正确反转
- [ ] +Z/-Z 翻面时，Z 轴符号正确反转
- [ ] 绕 X 正反旋转时，gyro X 响应方向相反
- [ ] 绕 Y 正反旋转时，gyro Y 响应方向相反
- [ ] 绕 Z 正反旋转时，gyro Z 响应方向相反
- [ ] 停止旋转后，gyro 能回到原零偏附近
- [ ] IMU 输出没有冻结、饱和或突然全部归零

## IMU 稳定性与归零

- [ ] 静置运行 10 分钟没有 I2C 连续读取错误
- [ ] 10 分钟采样率约为 100 Hz
- [ ] 静置期间 `imu.errors` 不增加
- [ ] 静置期间 ESP32 不重启
- [ ] 静置期间 Wi-Fi 查询保持可用
- [ ] TODO：实现串口 IMU 归零命令
- [ ] TODO：实现 Wi-Fi IMU 归零接口
- [ ] TODO：归零时检测板子是否保持静止
- [ ] TODO：归零后 gyro 三轴静止均值接近 0
- [ ] TODO：归零后加速度重力轴仍保留约 ±1g
- [ ] TODO：明确校准值仅存 RAM 还是保存到 NVS
- [ ] TODO：验证复位后的校准值保留策略

## 安全状态机

- [ ] 上电默认未 ARM
- [ ] 上电时 Motor A/B target 和 output 都为 0
- [ ] 上电时舵机输出关闭
- [ ] 上电时 GPIO35 外部接口处于释放状态
- [ ] SAFE 状态可以 ARM
- [ ] 已经 ARM 时重复 ARM 会被明确拒绝
- [ ] DISARM 后所有运动输出归零
- [ ] ESTOP 后所有运动输出归零
- [ ] ESTOP 状态不能直接 ARM
- [ ] CLEAR 后回到安全状态，不会自动 ARM
- [ ] 串口与 Wi-Fi 显示的安全状态一致
- [ ] GUI 关闭时会请求 DISARM

## 系统稳定性

- [ ] 连续运行 10 分钟没有异常复位
- [ ] 连续运行 30 分钟没有异常复位
- [ ] `free_heap` 没有持续单向下降
- [ ] `queue_overflow` 保持为 0
- [ ] `watchdog_stops` 的变化都能解释
- [ ] `commands_rejected` 的变化都能对应到手工测试命令
- [ ] 串口、Wi-Fi 和 GUI 同时查询时系统仍正常响应

## 有万用表或示波器后再核对

- [ ] 3.3V 电源稳定
- [ ] 光板静态电流合理
- [ ] DRV8833 nFAULT 空闲电平正确
- [ ] GPIO35/2N7002 置位与释放电平正确
- [ ] GPIO11/GPIO12 的 I2C 波形正常
- [ ] 舵机 PWM 周期和 0°/90°/180°脉宽正确
- [ ] Motor A 低占空比 PWM 正确
- [ ] Motor B 低占空比 PWM 正确
- [ ] 电机正反转控制逻辑正确

## 接执行器前的门槛

- [ ] 启动、串口、Wi-Fi 和 HTTP 项全部完成
- [ ] IMU 基础通信和 10 分钟稳定性通过
- [ ] 安全状态机项目通过
- [ ] 电源轨、nFAULT 和静态电流确认正常
- [ ] 已准备限流电源和独立急停手段
- [ ] 首次运动测试计划从 10% 输出开始

