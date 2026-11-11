# 手机浏览器 HTTP 控制

手机无需安装 App。固件内置中文控制页，不依赖互联网、电脑上位机或外部网页资源。

## 1. 编译和烧录

在 ESP-IDF 5.2.6 环境中执行：

```powershell
cd D:\esp32_project
idf.py build
idf.py -p COMx flash monitor
```

将 `COMx` 替换成板子的实际端口。此次修改需要重新烧录，旧固件不会自动更新页面。

## 2. 手机连接

两种方式任选一种：

- **设备热点：**手机连接 `ESP32-Robot`，密码 `esp32robot`，浏览器打开
  `http://192.168.4.1/`。手机提示热点没有互联网时，选择继续连接该网络。
- **同一局域网：**手机和 ESP32 连接同一个 Wi-Fi，浏览器打开串口日志或 `status`
  中的 ESP32 STA 地址，例如 `http://192.168.1.123/`。路由器若开启客户端隔离，设备之间可能无法访问。

输入完整的 `http://` 地址。建议在 Safari、Chrome 等完整浏览器中打开；不要依赖热点登录弹窗的小窗口。

## 3. 操作

1. 等待「已连接」，确认安全状态为 `SAFE`，然后点击「解锁 ARM」。
2. 行驶力度默认 `10%`。按住前进、后退、左转或右转按钮发送运动目标；松开即发送零目标。
3. 独立电机滑块只设置 Motor A/B 的目标，点击「开始输出」后持续运行，无需按住。
   点击本区「停止双电机」或方向键区「停车」停止。运行中调整滑块后，点击「更新输出」才应用新目标。
4. 舵机滑块只设置角度，点击「发送角度」才执行。GPIO35 置位/释放需要显式点击。
5. 「停车」将双电机目标归零，仍保留 ARM，执行固件原有斜坡停止。
   「停止并锁定」关闭电机、舵机 PWM 和外部接口；「急停」还会锁存 ESTOP。
6. 排除急停/故障原因后点击「清除故障 / 急停」，确认 `SAFE` 后重新解锁。

页面每 500 ms 读取状态，按住方向键或启动独立电机输出后，每 200 ms 刷新已提交的电机目标，同一时间最多有一个运动请求。
HTTP 的 `accepted` 只表示命令入队；ARM 后页面确认实际 `armed` 状态才开放执行器按钮。

切后台、锁屏、关闭页面或网络异常时，页面取消本地运动并尽力发送 DISARM。
恢复后不会自动继续旧运动，需要重新 ARM。手机被系统立即挂起或已经断网时，DISARM
可能无法送达；运动中的电机继续受固件断链超时保护，默认 3 秒后回到 SAFE。
舵机和 GPIO35 不具有独立的断链超时，操作结束应显式「停止并锁定」。

同一时刻使用一个控制端。若另一控制端已 ARM，新打开的手机页面会要求先锁定，
再从本页解锁，避免接管已有运动目标。

## 4. 验证

无硬件检查：

```powershell
python -m unittest discover -s tests -p "test_*.py"
node --test tests/test_phone_control.cjs
idf.py build
```

手机真机验收先断开机械负载，验证连接、状态、ARM/DISARM 和 ESTOP/CLEAR；再按项目的
分阶段板测流程检查按住输出、松手停车、触摸取消、切后台、锁屏及断网。首次实际输出从 10% 开始。

## 5. 页面显示 Header fields are too long

这是 ESP-IDF HTTP 服务器的 HTTP 431 错误：设备已经收到请求，但手机浏览器发送的
请求头总长度超过固件上限。旧固件使用默认的 512 字节，本项目现设为 4096 字节。

`sdkconfig.defaults` 已保存新的默认值；已有的 `sdkconfig` 不会自动被默认值覆盖。
若编译提示请求头上限不足，在 `idf.py menuconfig` 的
`Component config → HTTP Server → Max HTTP Request Header Length` 中设为 `4096`，
或把项目根目录 `sdkconfig` 的 `CONFIG_HTTPD_MAX_REQ_HDR_LEN` 改为 `4096`。
重新编译并烧录后，启动日志应出现 `HTTP request header limit: 4096 bytes`。
手机重新打开完整地址 `http://192.168.4.1/`，必要时关闭原标签页或使用无痕窗口。

配置含义见 [ESP-IDF 请求头上限说明](https://docs.espressif.com/projects/esp-idf/en/v5.2.6/esp32s3/api-reference/kconfig.html#config-httpd-max-req-hdr-len)。

页面源文件是 `components/robot_controller/control.html`，通过 ESP-IDF 的
`EMBED_TXTFILES` 写入固件，由原有 `GET /` 返回。HTTP v1 接口和桌面控制器保持兼容。
嵌入机制参考 [ESP-IDF 构建文档](https://docs.espressif.com/projects/esp-idf/en/v5.2.6/esp32s3/api-guides/build-system.html#embedding-binary-data)。
