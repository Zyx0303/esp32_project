# 工具

- `robot_control_gui.py`：使用 HTTP v1 的桌面控制器。
- `start_robot_control.py`：自动发现同一局域网中的 ESP32 并启动桌面控制器。
- `robot_smoke_test.py`：默认无运动的 HTTP 冒烟测试；运动阶段需要双重显式授权。
- `p0_preflight.py`：运行 Python、原生 C、Git whitespace 和可选 ESP-IDF 构建门禁。
- `run_p0_c_tests.py`：用主机 C 编译器编译并执行控制核心测试。

生成的 `__pycache__`、构建目录和临时编译器不属于项目内容。
