# 测试

- `test_p0_protocol_contract.py`：HTTP v1 与上位机协议契约。
- `test_p0_static_safety.py`：安全启动、队列、状态和构建接线静态检查。
- `test_wifi_apsta_contract.py`：局域网 STA、救援 AP、IP 状态和本地凭据隔离。
- `host/test_control_core.c`：可执行的状态机、斜坡、死区、看门狗和舵机行为测试。

所有测试均不需要连接开发板。

- `test_phone_control.cjs`：手机网页的按住/松手、急停、后台锁定、断线恢复和 ARM 状态确认行为。
  使用 Node.js 运行：`node --test tests/test_phone_control.cjs`；测试模拟 HTTP 响应，不连接机器人。
