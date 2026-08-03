"""ESP32-S3 robot desktop controller.

Connect the computer to the ``ESP32-Robot`` Wi-Fi network, then run:

    python tools/robot_control_gui.py

Only Python's standard library is required.  The default device address is
http://192.168.4.1 and can also be supplied with ``--url``.
"""

from __future__ import annotations

import argparse
import json
import queue
import threading
import time
import tkinter as tk
from dataclasses import dataclass
from tkinter import scrolledtext, ttk
from typing import Any, Callable
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


DEFAULT_DEVICE_URL = "http://192.168.4.1"
REQUEST_TIMEOUT_SECONDS = 0.8
MOTOR_HEARTBEAT_MS = 300
STATUS_POLL_MS = 1500
MAX_PARALLEL_REQUESTS = 2
MAX_LOG_LINES = 300


@dataclass(frozen=True)
class ApiResult:
    operation: str
    ok: bool
    payload: dict[str, Any] | None = None
    error: str = ""


def normalize_base_url(value: str) -> str:
    value = value.strip().rstrip("/")
    if not value:
        raise ValueError("设备地址不能为空")
    if not value.startswith(("http://", "https://")):
        value = "http://" + value
    return value


class RobotApi:
    def __init__(self, base_url: str) -> None:
        self.base_url = normalize_base_url(base_url)

    def set_base_url(self, base_url: str) -> None:
        self.base_url = normalize_base_url(base_url)

    def request(self, method: str, path: str, payload: dict[str, Any] | None = None) -> dict[str, Any]:
        body = None if payload is None else json.dumps(payload).encode("utf-8")
        headers = {"Accept": "application/json", "Connection": "close"}
        if body is not None:
            headers["Content-Type"] = "application/json"
        request = Request(
            self.base_url + path,
            data=body,
            headers=headers,
            method=method,
        )
        try:
            with urlopen(request, timeout=REQUEST_TIMEOUT_SECONDS) as response:
                body = response.read().decode("utf-8", errors="replace")
        except HTTPError as exc:
            detail = exc.read().decode("utf-8", errors="replace").strip()
            raise RuntimeError(f"HTTP {exc.code}: {detail or exc.reason}") from exc
        except (URLError, TimeoutError, OSError) as exc:
            reason = getattr(exc, "reason", exc)
            raise RuntimeError(f"无法连接设备：{reason}") from exc

        try:
            payload = json.loads(body)
        except json.JSONDecodeError as exc:
            raise RuntimeError(f"设备返回的不是有效 JSON：{body[:120]}") from exc
        if not isinstance(payload, dict):
            raise RuntimeError("设备返回格式错误")
        return payload

    def get(self, path: str) -> dict[str, Any]:
        return self.request("GET", path)

    def post(self, path: str, payload: dict[str, Any] | None = None) -> dict[str, Any]:
        return self.request("POST", path, payload)


class RobotControllerApp(tk.Tk):
    def __init__(self, base_url: str) -> None:
        super().__init__()
        self.title("ESP32-S3 机器人控制器")
        self.geometry("920x820")
        self.minsize(820, 700)

        self.api = RobotApi(base_url)
        self.results: queue.Queue[ApiResult] = queue.Queue()
        self.request_slots = threading.BoundedSemaphore(MAX_PARALLEL_REQUESTS)
        self.closing = False
        self.drive_direction = 0
        self.drive_after_id: str | None = None
        self.direct_after_id: str | None = None
        self.direct_active = False
        self.servo_after_id: str | None = None

        self.url_var = tk.StringVar(value=self.api.base_url)
        self.connection_var = tk.StringVar(value="尚未连接")
        self.safety_status_var = tk.StringVar(value="状态：--（未解锁）")
        self.motor_status_var = tk.StringVar(value="电机：--")
        self.servo_status_var = tk.StringVar(value="舵机：--")
        self.network_status_var = tk.StringVar(value="网络：--")
        self.fault_status_var = tk.StringVar(value="诊断：--")
        self.imu_status_var = tk.StringVar(value="IMU：--")
        self.counter_status_var = tk.StringVar(value="计数：--")
        self.speed_var = tk.IntVar(value=60)
        self.speed_text_var = tk.StringVar(value="60%")
        self.steering_var = tk.IntVar(value=0)
        self.steering_text_var = tk.StringVar(value="0%")
        self.motor_a_var = tk.IntVar(value=0)
        self.motor_b_var = tk.IntVar(value=0)
        self.motor_a_text_var = tk.StringVar(value="0%")
        self.motor_b_text_var = tk.StringVar(value="0%")
        self.power_status_var = tk.StringVar(value="GPIO35：--")
        self.angle_var = tk.IntVar(value=90)
        self.angle_text_var = tk.StringVar(value="90°")

        self._build_ui()
        self.protocol("WM_DELETE_WINDOW", self.on_close)
        self.after(100, self._drain_results)
        self.after(250, self.poll_status)

    def _build_ui(self) -> None:
        self.columnconfigure(0, weight=1)
        self.rowconfigure(4, weight=1)

        header = ttk.Frame(self, padding=14)
        header.grid(row=0, column=0, sticky="ew")
        header.columnconfigure(1, weight=1)
        ttk.Label(header, text="ESP32-S3 机器人控制器", font=("Microsoft YaHei UI", 18, "bold")).grid(
            row=0, column=0, columnspan=3, sticky="w", pady=(0, 10)
        )
        ttk.Label(header, text="设备地址").grid(row=1, column=0, sticky="w")
        ttk.Entry(header, textvariable=self.url_var).grid(row=1, column=1, sticky="ew", padx=8)
        ttk.Button(header, text="连接 / 刷新", command=self.apply_url_and_refresh).grid(row=1, column=2)

        status = ttk.LabelFrame(self, text="设备状态", padding=12)
        status.grid(row=1, column=0, sticky="ew", padx=14, pady=(0, 10))
        status.columnconfigure((0, 1, 2, 3), weight=1)
        ttk.Label(status, textvariable=self.connection_var).grid(row=0, column=0, sticky="w")
        ttk.Label(status, textvariable=self.safety_status_var).grid(row=0, column=1)
        ttk.Label(status, textvariable=self.motor_status_var).grid(row=0, column=2)
        ttk.Label(status, textvariable=self.servo_status_var).grid(row=0, column=3, sticky="e")
        safety_buttons = ttk.Frame(status)
        safety_buttons.grid(row=1, column=0, columnspan=4, sticky="ew", pady=(10, 0))
        safety_buttons.columnconfigure((0, 1, 2, 3), weight=1)
        ttk.Button(safety_buttons, text="ARM 解锁", command=self.arm).grid(row=0, column=0, sticky="ew", padx=3)
        ttk.Button(safety_buttons, text="DISARM 锁定", command=self.disarm).grid(row=0, column=1, sticky="ew", padx=3)
        ttk.Button(safety_buttons, text="急停", command=self.estop).grid(row=0, column=2, sticky="ew", padx=3)
        ttk.Button(safety_buttons, text="清除故障", command=self.clear_fault).grid(row=0, column=3, sticky="ew", padx=3)

        controls = ttk.Frame(self, padding=(14, 0, 14, 10))
        controls.grid(row=2, column=0, sticky="ew")
        controls.columnconfigure((0, 1), weight=1)

        motor = ttk.LabelFrame(controls, text="电机（按住运动，松开停止）", padding=12)
        motor.grid(row=0, column=0, sticky="nsew", padx=(0, 6))
        motor.columnconfigure((0, 1, 2), weight=1)
        ttk.Label(motor, text="速度").grid(row=0, column=0, sticky="w")
        ttk.Scale(
            motor,
            from_=0,
            to=100,
            variable=self.speed_var,
            command=self._on_speed_change,
        ).grid(row=0, column=1, sticky="ew", padx=8)
        ttk.Label(motor, textvariable=self.speed_text_var, width=5).grid(row=0, column=2)

        ttk.Label(motor, text="转向").grid(row=1, column=0, sticky="w")
        ttk.Scale(
            motor,
            from_=-100,
            to=100,
            variable=self.steering_var,
            command=self._on_steering_change,
        ).grid(row=1, column=1, sticky="ew", padx=8)
        ttk.Label(motor, textvariable=self.steering_text_var, width=5).grid(row=1, column=2)

        reverse = ttk.Button(motor, text="◀ 后退")
        reverse.grid(row=2, column=0, sticky="ew", pady=(14, 0), padx=(0, 5))
        stop = ttk.Button(motor, text="■ 停止", command=self.stop_motor)
        stop.grid(row=2, column=1, sticky="ew", pady=(14, 0), padx=5)
        forward = ttk.Button(motor, text="前进 ▶")
        forward.grid(row=2, column=2, sticky="ew", pady=(14, 0), padx=(5, 0))
        self._bind_hold_button(reverse, -1)
        self._bind_hold_button(forward, 1)

        servo = ttk.LabelFrame(controls, text="舵机", padding=12)
        servo.grid(row=0, column=1, sticky="nsew", padx=(6, 0))
        servo.columnconfigure(0, weight=1)
        ttk.Label(servo, textvariable=self.angle_text_var, font=("Microsoft YaHei UI", 16, "bold")).grid(
            row=0, column=0, pady=(0, 4)
        )
        ttk.Scale(
            servo,
            from_=0,
            to=180,
            variable=self.angle_var,
            command=self._on_angle_change,
        ).grid(row=1, column=0, sticky="ew")
        angle_buttons = ttk.Frame(servo)
        angle_buttons.grid(row=2, column=0, sticky="ew", pady=(12, 0))
        angle_buttons.columnconfigure((0, 1, 2), weight=1)
        for column, angle in enumerate((0, 90, 180)):
            ttk.Button(angle_buttons, text=f"{angle}°", command=lambda value=angle: self.set_servo(value)).grid(
                row=0, column=column, sticky="ew", padx=3
            )

        direct = ttk.LabelFrame(controls, text="双电机独立板测（持续刷新）", padding=12)
        direct.grid(row=1, column=0, sticky="nsew", padx=(0, 6), pady=(10, 0))
        direct.columnconfigure(1, weight=1)
        ttk.Label(direct, text="Motor A").grid(row=0, column=0, sticky="w")
        ttk.Scale(
            direct, from_=-100, to=100, variable=self.motor_a_var,
            command=lambda value: self._on_direct_change("a", value),
        ).grid(row=0, column=1, sticky="ew", padx=8)
        ttk.Label(direct, textvariable=self.motor_a_text_var, width=5).grid(row=0, column=2)
        ttk.Label(direct, text="Motor B").grid(row=1, column=0, sticky="w")
        ttk.Scale(
            direct, from_=-100, to=100, variable=self.motor_b_var,
            command=lambda value: self._on_direct_change("b", value),
        ).grid(row=1, column=1, sticky="ew", padx=8)
        ttk.Label(direct, textvariable=self.motor_b_text_var, width=5).grid(row=1, column=2)
        direct_buttons = ttk.Frame(direct)
        direct_buttons.grid(row=2, column=0, columnspan=3, sticky="ew", pady=(10, 0))
        direct_buttons.columnconfigure((0, 1), weight=1)
        ttk.Button(direct_buttons, text="开始独立输出", command=self.start_direct_motors).grid(
            row=0, column=0, sticky="ew", padx=(0, 4)
        )
        ttk.Button(direct_buttons, text="双电机归零", command=self.stop_motor).grid(
            row=0, column=1, sticky="ew", padx=(4, 0)
        )

        auxiliary = ttk.LabelFrame(controls, text="外部接口与传感器", padding=12)
        auxiliary.grid(row=1, column=1, sticky="nsew", padx=(6, 0), pady=(10, 0))
        auxiliary.columnconfigure((0, 1), weight=1)
        ttk.Label(auxiliary, textvariable=self.power_status_var).grid(
            row=0, column=0, columnspan=2, sticky="w"
        )
        ttk.Button(auxiliary, text="GPIO35 置位", command=lambda: self.set_power(True)).grid(
            row=1, column=0, sticky="ew", padx=(0, 4), pady=(8, 0)
        )
        ttk.Button(auxiliary, text="GPIO35 释放", command=lambda: self.set_power(False)).grid(
            row=1, column=1, sticky="ew", padx=(4, 0), pady=(8, 0)
        )
        ttk.Label(auxiliary, textvariable=self.imu_status_var, wraplength=390).grid(
            row=2, column=0, columnspan=2, sticky="w", pady=(12, 0)
        )

        diagnostics = ttk.LabelFrame(self, text="实时诊断", padding=10)
        diagnostics.grid(row=3, column=0, sticky="ew", padx=14, pady=(0, 10))
        diagnostics.columnconfigure((0, 1), weight=1)
        ttk.Label(diagnostics, textvariable=self.network_status_var).grid(row=0, column=0, sticky="w")
        ttk.Label(diagnostics, textvariable=self.fault_status_var).grid(row=0, column=1, sticky="w")
        ttk.Label(diagnostics, textvariable=self.counter_status_var, wraplength=860).grid(
            row=1, column=0, columnspan=2, sticky="w", pady=(5, 0)
        )

        log_frame = ttk.LabelFrame(self, text="运行日志", padding=10)
        log_frame.grid(row=4, column=0, sticky="nsew", padx=14, pady=(0, 14))
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)
        self.log_text = scrolledtext.ScrolledText(log_frame, wrap=tk.WORD, state="disabled", height=12)
        self.log_text.grid(row=0, column=0, sticky="nsew")
        self._log("请让电脑与 ESP32 位于同一局域网；运动前必须先确认硬件并 ARM。")

    def _bind_hold_button(self, button: ttk.Button, direction: int) -> None:
        button.bind("<ButtonPress-1>", lambda _event: self.start_drive(direction))
        button.bind("<ButtonRelease-1>", lambda _event: self.stop_motor())
        button.bind("<Leave>", lambda _event: self.stop_motor() if self.drive_direction == direction else None)

    def _on_speed_change(self, value: str) -> None:
        speed = round(float(value))
        self.speed_var.set(speed)
        self.speed_text_var.set(f"{speed}%")

    def _on_steering_change(self, value: str) -> None:
        steering = round(float(value))
        self.steering_var.set(steering)
        self.steering_text_var.set(f"{steering}%")

    def _on_direct_change(self, axis: str, value: str) -> None:
        output = round(float(value))
        if axis == "a":
            self.motor_a_var.set(output)
            self.motor_a_text_var.set(f"{output}%")
        else:
            self.motor_b_var.set(output)
            self.motor_b_text_var.set(f"{output}%")

    def _on_angle_change(self, value: str) -> None:
        angle = round(float(value))
        self.angle_var.set(angle)
        self.angle_text_var.set(f"{angle}°")
        if self.servo_after_id is not None:
            self.after_cancel(self.servo_after_id)
        self.servo_after_id = self.after(180, lambda: self._send_servo(angle))

    def apply_url_and_refresh(self) -> None:
        try:
            self.api.set_base_url(self.url_var.get())
        except ValueError as exc:
            self.connection_var.set(str(exc))
            return
        self.url_var.set(self.api.base_url)
        self._log(f"设备地址已设置为 {self.api.base_url}")
        self.poll_status()

    def start_drive(self, direction: int) -> None:
        self._cancel_direct_output()
        self.drive_direction = direction
        self._send_drive_heartbeat()

    def _send_drive_heartbeat(self) -> None:
        if self.closing or self.drive_direction == 0:
            return
        speed = max(0, min(100, self.speed_var.get())) * self.drive_direction
        steering = max(-100, min(100, self.steering_var.get()))
        self._request_async(
            "drive", "/api/v1/drive", method="POST",
            payload={"throttle": speed, "steering": steering},
        )
        self.drive_after_id = self.after(MOTOR_HEARTBEAT_MS, self._send_drive_heartbeat)

    def stop_motor(self) -> None:
        self.drive_direction = 0
        if self.drive_after_id is not None:
            self.after_cancel(self.drive_after_id)
            self.drive_after_id = None
        self._cancel_direct_output()
        self.motor_a_var.set(0)
        self.motor_b_var.set(0)
        self.motor_a_text_var.set("0%")
        self.motor_b_text_var.set("0%")
        self._request_async(
            "stop", "/api/v1/drive", method="POST", payload={"throttle": 0, "steering": 0}
        )

    def _cancel_direct_output(self) -> None:
        self.direct_active = False
        if self.direct_after_id is not None:
            self.after_cancel(self.direct_after_id)
            self.direct_after_id = None

    def start_direct_motors(self) -> None:
        self.drive_direction = 0
        if self.drive_after_id is not None:
            self.after_cancel(self.drive_after_id)
            self.drive_after_id = None
        self.direct_active = True
        self._send_direct_heartbeat()

    def _send_direct_heartbeat(self) -> None:
        if self.closing or not self.direct_active:
            return
        motor_a = max(-100, min(100, self.motor_a_var.get()))
        motor_b = max(-100, min(100, self.motor_b_var.get()))
        self._request_async(
            "motors", "/api/v1/motors", method="POST",
            payload={"motor_a": motor_a, "motor_b": motor_b},
        )
        self.direct_after_id = self.after(MOTOR_HEARTBEAT_MS, self._send_direct_heartbeat)

    def arm(self) -> None:
        self._request_async("arm", "/api/v1/arm", method="POST")

    def disarm(self) -> None:
        self.drive_direction = 0
        self._cancel_direct_output()
        self._request_async("disarm", "/api/v1/disarm", method="POST")

    def estop(self) -> None:
        self.drive_direction = 0
        self._cancel_direct_output()
        self._request_async("estop", "/api/v1/estop", method="POST")

    def clear_fault(self) -> None:
        self._request_async("fault_clear", "/api/v1/fault/clear", method="POST")

    def set_servo(self, angle: int) -> None:
        self.angle_var.set(angle)
        self.angle_text_var.set(f"{angle}°")
        self._send_servo(angle)

    def _send_servo(self, angle: int) -> None:
        self.servo_after_id = None
        self._request_async(
            "servo", "/api/v1/servo", method="POST", payload={"angle": max(0, min(180, angle))}
        )

    def set_power(self, asserted: bool) -> None:
        self._request_async(
            "power", "/api/v1/power", method="POST", payload={"asserted": asserted}
        )

    def poll_status(self) -> None:
        if self.closing:
            return
        self._request_async("status", "/api/v1/status")
        self.after(STATUS_POLL_MS, self.poll_status)

    def _request_async(
        self,
        operation: str,
        path: str,
        *,
        method: str = "GET",
        payload: dict[str, Any] | None = None,
    ) -> None:
        if self.closing or not self.request_slots.acquire(blocking=False):
            return

        def worker() -> None:
            try:
                response = self.api.request(method, path, payload)
                result = ApiResult(operation=operation, ok=True, payload=response)
            except RuntimeError as exc:
                result = ApiResult(operation=operation, ok=False, error=str(exc))
            finally:
                self.request_slots.release()
            self.results.put(result)

        threading.Thread(target=worker, daemon=True).start()

    def _drain_results(self) -> None:
        while True:
            try:
                result = self.results.get_nowait()
            except queue.Empty:
                break
            self._handle_result(result)
        if not self.closing:
            self.after(100, self._drain_results)

    def _handle_result(self, result: ApiResult) -> None:
        if not result.ok:
            self.connection_var.set("未连接")
            if result.operation != "status" or not getattr(self, "_last_status_error", False):
                self._log(f"{result.operation} 失败：{result.error}")
            self._last_status_error = True
            return

        self._last_status_error = False
        payload = result.payload or {}
        self.connection_var.set("已连接")
        if "state" in payload and "armed" in payload:
            armed = "已解锁" if payload.get("armed") else "未解锁"
            self.safety_status_var.set(f"状态：{payload['state']}（{armed}）")
        if "motor_a" in payload and "motor_b" in payload:
            self.motor_status_var.set(
                f"电机：{payload['motor_a'].get('output', '--')}/{payload['motor_b'].get('output', '--')}%"
            )
        if "servo" in payload and isinstance(payload["servo"], dict):
            servo = payload["servo"]
            suffix = "" if servo.get("enabled") else "（关闭）"
            self.servo_status_var.set(f"舵机：{servo.get('output', '--')}°{suffix}")
        if "power" in payload:
            self.power_status_var.set(f"GPIO35：{'置位' if payload.get('power') else '释放'}")
        imu = payload.get("imu")
        if isinstance(imu, dict):
            if imu.get("valid"):
                accel = imu.get("accel", ["--", "--", "--"])
                gyro = imu.get("gyro", ["--", "--", "--"])
                self.imu_status_var.set(
                    f"IMU 原始值：A={accel} G={gyro} 样本={imu.get('samples', '--')} "
                    f"错误={imu.get('errors', '--')}"
                )
            else:
                self.imu_status_var.set(
                    f"IMU：不可用　样本={imu.get('samples', 0)} 错误={imu.get('errors', 0)}"
                )
        network = payload.get("network")
        if isinstance(network, dict):
            retry = network.get("reconnect_delay_ms", 0)
            reason = network.get("last_disconnect_reason", 0)
            self.network_status_var.set(
                f"网络：{network.get('mode', '--')} IP={network.get('sta_ip', '--')} "
                f"重连={network.get('reconnects', 0)} 原因={reason} 下次={retry}ms"
            )
        if "faults" in payload:
            self.fault_status_var.set(
                f"故障：mask=0x{int(payload.get('faults', 0)):08x} "
                f"nFAULT={'有效' if payload.get('drv8833_fault') else '正常'}"
            )
        counters = payload.get("counters")
        system = payload.get("system")
        if isinstance(counters, dict) and isinstance(system, dict):
            self.counter_status_var.set(
                f"运行 {system.get('uptime_ms', 0)} ms｜堆 {system.get('free_heap', 0)} B｜"
                f"命令 {counters.get('received', 0)}｜拒绝 {counters.get('rejected', 0)}｜"
                f"过期 {counters.get('expired', 0)}｜看门狗 {counters.get('watchdog_stops', 0)}｜"
                f"队列溢出 {counters.get('queue_overflow', 0)}"
            )
        if result.operation != "status":
            self._log(f"{result.operation} 成功：{json.dumps(payload, ensure_ascii=False)}")

    def _log(self, message: str) -> None:
        timestamp = time.strftime("%H:%M:%S")
        self.log_text.configure(state="normal")
        self.log_text.insert(tk.END, f"[{timestamp}] {message}\n")
        line_count = int(self.log_text.index("end-1c").split(".")[0])
        if line_count > MAX_LOG_LINES:
            self.log_text.delete("1.0", f"{line_count - MAX_LOG_LINES}.0")
        self.log_text.configure(state="disabled")
        self.log_text.see(tk.END)

    def on_close(self) -> None:
        if self.closing:
            return
        self.drive_direction = 0
        self._cancel_direct_output()
        self._request_async("disarm", "/api/v1/disarm", method="POST")
        self.closing = True
        self.after(80, self.destroy)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="ESP32-S3 机器人图形控制器")
    parser.add_argument("--url", default=DEFAULT_DEVICE_URL, help="设备地址，默认 http://192.168.4.1")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    app = RobotControllerApp(args.url)
    app.mainloop()


if __name__ == "__main__":
    main()
