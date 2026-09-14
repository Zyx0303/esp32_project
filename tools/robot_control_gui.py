"""ESP32-S3 robot desktop controller.

Connect the computer to the ``ESP32-Robot`` Wi-Fi network, then run:

    python tools/robot_control_gui.py

Only Python's standard library is required.  The default device address is
http://192.168.4.1 and can also be supplied with ``--url``.
"""

from __future__ import annotations

import argparse
from http.client import HTTPException
import json
import queue
import os
import stat
import uuid
from pathlib import Path
import threading
import time
import tkinter as tk
from dataclasses import dataclass
from tkinter import font as tkfont
from tkinter import messagebox, scrolledtext, ttk
from typing import Any, Callable
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


DEFAULT_DEVICE_URL = "http://192.168.4.1"
REQUEST_TIMEOUT_SECONDS = 0.8
MOTOR_HEARTBEAT_MS = 300
STATUS_POLL_MS = 1500
MAX_PARALLEL_REQUESTS = 2
MAX_LOG_LINES = 300

COLORS = {
    "canvas": "#F4F7FB",
    "card": "#FFFFFF",
    "ink": "#172033",
    "muted": "#64748B",
    "line": "#DDE5F0",
    "navy": "#0F172A",
    "navy_soft": "#1E293B",
    "blue": "#2563EB",
    "blue_hover": "#1D4ED8",
    "green": "#159A68",
    "green_hover": "#0F7F56",
    "red": "#DC3545",
    "red_hover": "#B42332",
    "amber": "#D97706",
    "amber_hover": "#B45309",
    "soft_blue": "#E8F0FE",
    "soft_green": "#E6F6EF",
    "soft_red": "#FDECEE",
}


HELP_TEXT = """建议操作顺序

1. 确认设备地址并点击“连接 / 刷新”。
2. 先观察设备状态、故障、IMU 和网络信息。
3. 光板测试不需要 ARM；接入执行器后，应先架空轮子并把速度调低。
4. 需要运动时才点击“ARM 解锁”，测试完成后点击“DISARM 锁定”。

连接与安全按钮

连接 / 刷新
    使用地址框中的 URL 读取设备状态。更换设备地址、设备重启或网络恢复后点击它，
    用来确认当前连接的是预期机器人；它本身不会解锁或驱动执行器。

ARM 解锁
    允许固件接受运动类命令。之所以单独设置解锁步骤，是为了防止刚连接、误触按钮
    或旧命令导致电机突然动作。只有确认硬件安全、轮子架空或场地清空后才能按。

DISARM 锁定
    取消解锁并停止持续运动输出。它是正常结束测试时使用的安全锁定方式；调整接线、
    搬动机器人或离开设备前都应点击。光板和 IMU 测试应保持未解锁状态。

急停
    立即请求紧急停止，用于动作失控、方向错误、机械卡住或人员靠近等情况。
    急停比普通“停止”更强，会进入安全状态；排除原因后再按“清除故障”，不要把急停
    当作日常启停按钮。

清除故障
    在故障原因已经消失后，请求清除可恢复的故障或急停状态。它不会修复断线、短路、
    IMU 掉线等真实问题，也不会自动 ARM；清除后仍要检查状态再决定是否解锁。

行驶控制

速度滑块
    设置前进/后退的油门幅度，0% 最慢，100% 最大。首次板测建议从 10% 开始，
    因为低速能降低接线方向错误或机械干涉造成的风险。

转向滑块
    设置左右轮差速：负值和正值代表相反转向方向，0% 为直行。它不是独立电机控制，
    固件会根据速度和转向混合出左右电机目标。

◀ 后退 / 前进 ▶
    必须按住才持续发送运动心跳，松开或鼠标移出按钮就发送停止。采用“按住运行”是为了
    让操作员失去控制、窗口卡顿或网络中断时，固件能依靠心跳超时自动停机。

■ 停止
    立即把油门、转向以及双电机独立输出归零。它用于一次动作结束，但不会取消 ARM；
    完成整轮测试后仍应点击“DISARM 锁定”。

舵机控制

舵机角度滑块
    设置 0° 到 180° 的目标角度，停止拖动片刻后自动发送。首次接机械结构时应从 90°
    附近小范围测试，避免舵机撞限位。

0° / 90° / 180°
    快速发送三个基准角度，便于检查方向、中位和行程端点。没有确认机械行程之前，
    不要直接点击 0° 或 180°；光板未接舵机时无需使用。

双电机独立板测

Motor A / Motor B 滑块
    分别设置两路电机的独立输出，正负号代表相反方向。它绕过“速度 + 转向”的混合，
    只用于核对通道、极性和驱动板，不适合正常驾驶。

开始独立输出
    按当前 A/B 数值持续发送双路电机心跳。必须先把轮子架空、从小幅值开始并完成 ARM；
    点击后即持续输出，所以应随时准备按“双电机归零”或“急停”。

双电机归零
    停止独立输出并把 A/B 滑块和电机命令都设为 0。它不会取消 ARM，结束板测后仍需
    点击“DISARM 锁定”。

外部接口与传感器

GPIO35 置位
    请求外部低有效接口进入 asserted 状态。这里控制的是板上的外部接口电路，不是把
    ESP32 GPIO35 当作普通高电平输出；只有确认原理图和外部负载含义后才能使用。

GPIO35 释放
    解除外部接口的 asserted 状态，通常用于结束该接口测试或恢复安全状态。

IMU 状态
    只读显示 MPU6050 的有效标志、加速度、角速度、样本数和错误数。查看 IMU 不需要
    ARM。静止时陀螺仪应接近零，加速度合量应接近 1g，错误数不应持续增加。

IMU 记录
    点击“开始记录 IMU”在机器人上写入 100 Hz 原始数据；再次点击结束并通过 HTTP 下载。
    文件保存在项目 imu_logs 文件夹，设备和电脑端均设置只读。断网后点击“结束 / 重试下载”。
    分区有限，满时会自动结束；日志末尾记录丢帧计数和结束原因。

状态与日志

设备状态显示当前安全状态、ARM、电机和舵机输出；实时诊断显示网络、硬件故障和计数；
运行日志记录每次命令是否成功。按钮没有反应时，先看这里，不要连续重复点击运动命令。
"""


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


def configure_chinese_fonts(root: tk.Misc) -> str:
    """Select an installed CJK font instead of relying on missing Windows fonts."""
    available = {name.casefold(): name for name in tkfont.families(root)}
    candidates = (
        "Noto Sans CJK SC", "Noto Sans SC", "Source Han Sans SC",
        "Microsoft YaHei UI", "Microsoft YaHei", "PingFang SC",
        "WenQuanYi Micro Hei", "WenQuanYi Zen Hei", "SimHei",
        "Noto Sans CJK TC", "Noto Sans CJK JP",
    )
    family = next((available[name.casefold()] for name in candidates
                   if name.casefold() in available),
                  tkfont.nametofont("TkDefaultFont", root=root).actual("family"))
    for name in ("TkDefaultFont", "TkTextFont", "TkFixedFont", "TkMenuFont",
                 "TkHeadingFont", "TkCaptionFont", "TkSmallCaptionFont",
                 "TkIconFont", "TkTooltipFont"):
        try:
            tkfont.nametofont(name, root=root).configure(family=family)
        except tk.TclError:
            pass
    root.option_add("*Font", "TkDefaultFont")
    return family


class ToolTip:
    """Small delayed hover hint with no third-party dependency."""

    def __init__(self, widget: tk.Widget, message: str) -> None:
        self.widget = widget
        self.ui_font = tkfont.nametofont("TkDefaultFont", root=widget).actual("family")
        self.message = message
        self.window: tk.Toplevel | None = None
        self.after_id: str | None = None
        widget.bind("<Enter>", self._schedule, add="+")
        widget.bind("<Leave>", self._hide, add="+")
        widget.bind("<ButtonPress>", self._hide, add="+")

    def _schedule(self, _event: tk.Event[Any]) -> None:
        self._cancel()
        self.after_id = self.widget.after(500, self._show)

    def _cancel(self) -> None:
        if self.after_id is not None:
            self.widget.after_cancel(self.after_id)
            self.after_id = None

    def _show(self) -> None:
        if self.window is not None:
            return
        x = self.widget.winfo_rootx() + 12
        y = self.widget.winfo_rooty() + self.widget.winfo_height() + 8
        self.window = tk.Toplevel(self.widget)
        self.window.wm_overrideredirect(True)
        self.window.wm_geometry(f"+{x}+{y}")
        tk.Label(
            self.window,
            text=self.message,
            justify="left",
            wraplength=340,
            background="#111827",
            foreground="#F8FAFC",
            padx=10,
            pady=7,
            font=(self.ui_font, 9),
        ).pack()

    def _hide(self, _event: tk.Event[Any] | None = None) -> None:
        self._cancel()
        if self.window is not None:
            self.window.destroy()
            self.window = None


class RobotApi:
    def __init__(self, base_url: str) -> None:
        self.base_url = normalize_base_url(base_url)

    def set_base_url(self, base_url: str) -> None:
        self.base_url = normalize_base_url(base_url)

    def request(self, method: str, path: str, payload: dict[str, Any] | None = None, *,
                timeout: float = REQUEST_TIMEOUT_SECONDS) -> dict[str, Any]:
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
            with urlopen(request, timeout=timeout) as response:
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

    def download_imu_log(self, filename: str) -> Path:
        directory = Path(__file__).resolve().parent.parent / "imu_logs"
        directory.mkdir(parents=True, exist_ok=True)
        destination = directory / (time.strftime("imu_%Y%m%d_%H%M%S_") + uuid.uuid4().hex[:8] + ".log")
        partial = destination.with_suffix(".part")
        request = Request(self.base_url + "/api/v1/imu/log/download?file=" + filename, headers={"Connection": "close"})
        try:
            with urlopen(request, timeout=15) as response, partial.open("xb") as output:
                expected = response.headers.get("Content-Length")
                received = 0
                while chunk := response.read(65536):
                    output.write(chunk)
                    received += len(chunk)
                if expected is not None and received != int(expected):
                    raise RuntimeError("日志下载不完整，请重试")
                output.flush()
                os.fsync(output.fileno())
            partial.replace(destination)
            destination.chmod(stat.S_IREAD | stat.S_IRGRP | stat.S_IROTH)
        except Exception:
            partial.unlink(missing_ok=True)
            raise
        return destination


class RobotControllerApp(tk.Tk):
    def __init__(self, base_url: str) -> None:
        super().__init__()
        self.ui_font = configure_chinese_fonts(self)
        self.title("ESP32-S3 机器人控制器")
        self.geometry("1180x820")
        self.minsize(1040, 700)
        self.configure(background=COLORS["canvas"])

        self.api = RobotApi(base_url)
        self.results: queue.Queue[ApiResult] = queue.Queue()
        self.request_slots = threading.BoundedSemaphore(MAX_PARALLEL_REQUESTS)
        self.closing = False
        self.drive_direction = 0
        self.drive_after_id: str | None = None
        self.direct_after_id: str | None = None
        self.direct_active = False
        self.servo_after_id: str | None = None
        self.recording = False
        self.record_busy = False
        self.record_url: str | None = None
        self.record_text = tk.StringVar(value="开始记录 IMU")
        self.record_status = tk.StringVar(value="100 Hz 原始数据 · 保存在电脑 imu_logs 文件夹")

        self.url_var = tk.StringVar(value=self.api.base_url)
        self.connection_var = tk.StringVar(value="● 等待连接")
        self.safety_status_var = tk.StringVar(value="状态：--（未解锁）")
        self.motor_status_var = tk.StringVar(value="电机：--")
        self.servo_status_var = tk.StringVar(value="舵机：--")
        self.network_status_var = tk.StringVar(value="网络：--")
        self.fault_status_var = tk.StringVar(value="诊断：--")
        self.imu_status_var = tk.StringVar(value="IMU：--")
        self.counter_status_var = tk.StringVar(value="计数：--")
        self.speed_var = tk.IntVar(value=10)
        self.speed_text_var = tk.StringVar(value="10%")
        self.steering_var = tk.IntVar(value=0)
        self.steering_text_var = tk.StringVar(value="0%")
        self.motor_a_var = tk.IntVar(value=0)
        self.motor_b_var = tk.IntVar(value=0)
        self.motor_a_text_var = tk.StringVar(value="0%")
        self.motor_b_text_var = tk.StringVar(value="0%")
        self.power_status_var = tk.StringVar(value="GPIO35：--")
        self.angle_var = tk.IntVar(value=90)
        self.angle_text_var = tk.StringVar(value="90°")
        self.tooltips: list[ToolTip] = []

        self._configure_style()
        self._build_ui()
        self.bind("<F1>", lambda _event: self.show_help())
        self.protocol("WM_DELETE_WINDOW", self.on_close)
        self.after(100, self._drain_results)
        self.after(250, self.poll_status)

    def _configure_style(self) -> None:
        style = ttk.Style(self)
        style.theme_use("clam")
        style.configure(".", font=(self.ui_font, 10), background=COLORS["canvas"])
        style.configure("App.TFrame", background=COLORS["canvas"])
        style.configure("Card.TFrame", background=COLORS["card"])
        style.configure(
            "Card.TLabelframe",
            background=COLORS["card"],
            bordercolor=COLORS["line"],
            lightcolor=COLORS["line"],
            darkcolor=COLORS["line"],
            relief="solid",
        )
        style.configure(
            "Card.TLabelframe.Label",
            background=COLORS["card"], foreground=COLORS["ink"],
            font=(self.ui_font, 11, "bold"),
        )
        style.configure("Card.TLabel", background=COLORS["card"], foreground=COLORS["ink"])
        style.configure(
            "Muted.TLabel", background=COLORS["card"], foreground=COLORS["muted"],
            font=(self.ui_font, 9),
        )
        style.configure(
            "Value.TLabel", background=COLORS["card"], foreground=COLORS["ink"],
            font=(self.ui_font, 11, "bold"),
        )
        style.configure(
            "Online.TLabel", background=COLORS["soft_green"], foreground=COLORS["green"],
            padding=(10, 5), font=(self.ui_font, 10, "bold"),
        )
        style.configure(
            "Offline.TLabel", background=COLORS["soft_red"], foreground=COLORS["red"],
            padding=(10, 5), font=(self.ui_font, 10, "bold"),
        )
        style.configure(
            "Safe.TLabel", background=COLORS["soft_blue"], foreground=COLORS["blue"],
            padding=(10, 5), font=(self.ui_font, 10, "bold"),
        )
        style.configure(
            "Armed.TLabel", background="#FFF4DB", foreground=COLORS["amber"],
            padding=(10, 5), font=(self.ui_font, 10, "bold"),
        )
        for name, color, hover in (
            ("Primary", COLORS["blue"], COLORS["blue_hover"]),
            ("Success", COLORS["green"], COLORS["green_hover"]),
            ("Danger", COLORS["red"], COLORS["red_hover"]),
            ("Warning", COLORS["amber"], COLORS["amber_hover"]),
            ("Dark", COLORS["navy_soft"], COLORS["navy"]),
        ):
            style.configure(
                f"{name}.TButton", background=color, foreground="white",
                bordercolor=color, padding=(14, 8), font=(self.ui_font, 10, "bold"),
            )
            style.map(
                f"{name}.TButton",
                background=[("active", hover), ("pressed", hover)],
                bordercolor=[("active", hover)],
            )
        style.configure(
            "Secondary.TButton", background="#EDF2F7", foreground=COLORS["ink"],
            bordercolor="#D7E0EB", padding=(14, 8), font=(self.ui_font, 10, "bold"),
        )
        style.map("Secondary.TButton", background=[("active", "#E2E8F0")])
        style.configure(
            "Header.TButton", background=COLORS["navy_soft"], foreground="#E2E8F0",
            bordercolor="#334155", padding=(13, 7), font=(self.ui_font, 10, "bold"),
        )
        style.map("Header.TButton", background=[("active", "#334155")])
        style.configure("TEntry", padding=8, fieldbackground="white", bordercolor=COLORS["line"])
        style.configure("TScale", background=COLORS["card"], troughcolor="#DFE7F2")
        style.configure("TNotebook", background=COLORS["canvas"], borderwidth=0)
        style.configure(
            "TNotebook.Tab", background="#E8EDF5", foreground=COLORS["muted"],
            padding=(22, 10), font=(self.ui_font, 10, "bold"),
        )
        style.map(
            "TNotebook.Tab",
            background=[("selected", COLORS["card"])],
            foreground=[("selected", COLORS["blue"])],
        )

    def _tip(self, widget: tk.Widget, message: str) -> None:
        self.tooltips.append(ToolTip(widget, message))

    def _build_ui(self) -> None:
        self.columnconfigure(0, weight=1)
        self.rowconfigure(2, weight=1)

        header = tk.Frame(self, background=COLORS["navy"], padx=22, pady=16)
        header.grid(row=0, column=0, sticky="ew")
        header.columnconfigure(0, weight=1)
        tk.Label(
            header, text="ROBOT CONTROL", background=COLORS["navy"], foreground="#60A5FA",
            font=(self.ui_font, 9, "bold"),
        ).grid(row=0, column=0, sticky="w")
        tk.Label(
            header, text="ESP32-S3 机器人控制台", background=COLORS["navy"], foreground="white",
            font=(self.ui_font, 21, "bold"),
        ).grid(row=1, column=0, sticky="w")
        tk.Label(
            header, text="实时状态 · 安全控制 · 硬件板测", background=COLORS["navy"],
            foreground="#94A3B8", font=(self.ui_font, 10),
        ).grid(row=2, column=0, sticky="w", pady=(3, 0))
        tk.Label(
            header, text="局域网 HTTP", background=COLORS["navy_soft"], foreground="#BFDBFE",
            padx=12, pady=6, font=(self.ui_font, 9, "bold"),
        ).grid(row=1, column=1, padx=(12, 14))
        help_button = ttk.Button(header, text="帮助  F1", style="Header.TButton", command=self.show_help)
        help_button.grid(row=1, column=2, sticky="e")
        self._tip(help_button, "查看所有按钮的用途、操作理由和安全注意事项。")

        top = ttk.Frame(self, style="App.TFrame", padding=(18, 14, 18, 8))
        top.grid(row=1, column=0, sticky="ew")
        top.columnconfigure((0, 1), weight=1, uniform="top")

        connection = ttk.Frame(top, style="Card.TFrame", padding=14)
        connection.grid(row=0, column=0, sticky="nsew", padx=(0, 6))
        connection.columnconfigure(1, weight=1)
        ttk.Label(connection, text="设备连接", style="Value.TLabel").grid(row=0, column=0, sticky="w")
        self.connection_label = ttk.Label(
            connection, textvariable=self.connection_var, style="Offline.TLabel"
        )
        self.connection_label.grid(row=0, column=2, sticky="e", padx=(12, 0))
        ttk.Label(connection, text="设备地址", style="Muted.TLabel").grid(
            row=1, column=0, sticky="w", pady=(10, 0)
        )
        address_entry = ttk.Entry(connection, textvariable=self.url_var)
        address_entry.grid(row=1, column=1, sticky="ew", padx=10, pady=(10, 0))
        connect_button = ttk.Button(
            connection, text="连接 / 刷新", style="Primary.TButton", command=self.apply_url_and_refresh
        )
        connect_button.grid(row=1, column=2, pady=(10, 0))
        self.scan_button = ttk.Button(connection, text="扫描设备（可选）",
                                      command=self.scan_devices, style="Secondary.TButton")
        self.scan_button.grid(row=2, column=1, sticky="w", padx=10, pady=(8, 0))
        self._tip(connect_button, "验证地址并立即刷新状态；不会解锁或驱动任何硬件。")

        status = ttk.Frame(top, style="Card.TFrame", padding=14)
        status.grid(row=0, column=1, sticky="nsew", padx=(6, 0))
        status.columnconfigure((0, 1, 2, 3), weight=1)
        self.safety_label = ttk.Label(status, textvariable=self.safety_status_var, style="Safe.TLabel")
        self.safety_label.grid(row=0, column=0, sticky="w")
        ttk.Label(status, textvariable=self.motor_status_var, style="Card.TLabel").grid(row=0, column=1)
        ttk.Label(status, textvariable=self.servo_status_var, style="Card.TLabel").grid(row=0, column=2)
        ttk.Label(
            status, text="先确认状态，再执行控制", style="Muted.TLabel"
        ).grid(row=0, column=3, sticky="e")

        safety_buttons = ttk.Frame(status, style="Card.TFrame")
        safety_buttons.grid(row=1, column=0, columnspan=4, sticky="ew", pady=(12, 0))
        safety_buttons.columnconfigure((0, 1, 2, 3), weight=1)
        arm_button = ttk.Button(safety_buttons, text="ARM 解锁", style="Success.TButton", command=self.arm)
        disarm_button = ttk.Button(
            safety_buttons, text="DISARM 锁定", style="Secondary.TButton", command=self.disarm
        )
        estop_button = ttk.Button(safety_buttons, text="急停", style="Danger.TButton", command=self.estop)
        clear_button = ttk.Button(
            safety_buttons, text="清除故障", style="Warning.TButton", command=self.clear_fault
        )
        for column, button in enumerate((arm_button, disarm_button, estop_button, clear_button)):
            button.grid(row=0, column=column, sticky="ew", padx=4)
        self._tip(arm_button, "允许运动命令。仅在硬件、场地和人员都安全后使用。")
        self._tip(disarm_button, "正常结束测试：停止持续输出并取消运动权限。")
        self._tip(estop_button, "异常时立即进入紧急停止；排除原因后才能清除。")
        self._tip(clear_button, "仅清除已排除原因的可恢复故障，不会自动解锁。")

        notebook = ttk.Notebook(self)
        notebook.grid(row=2, column=0, sticky="nsew", padx=18, pady=(4, 18))

        controls = ttk.Frame(notebook, style="App.TFrame", padding=(0, 12, 0, 0))
        diagnostics_page = ttk.Frame(notebook, style="App.TFrame", padding=(0, 12, 0, 0))
        notebook.add(controls, text="设备控制")
        notebook.add(diagnostics_page, text="诊断与日志")
        controls.columnconfigure((0, 1), weight=1, uniform="control")
        controls.rowconfigure((0, 1), weight=1)

        motor = ttk.LabelFrame(controls, text="行驶控制 · 按住运行，松开停止", style="Card.TLabelframe", padding=16)
        motor.grid(row=0, column=0, sticky="nsew", padx=(0, 6), pady=(0, 6))
        motor.columnconfigure(1, weight=1)
        ttk.Label(motor, text="速度", style="Card.TLabel").grid(row=0, column=0, sticky="w")
        speed_scale = ttk.Scale(
            motor, from_=0, to=100, variable=self.speed_var, command=self._on_speed_change
        )
        speed_scale.grid(row=0, column=1, sticky="ew", padx=10)
        ttk.Label(motor, textvariable=self.speed_text_var, style="Value.TLabel", width=5).grid(row=0, column=2)
        ttk.Label(motor, text="转向", style="Card.TLabel").grid(row=1, column=0, sticky="w", pady=(10, 0))
        steering_scale = ttk.Scale(
            motor, from_=-100, to=100, variable=self.steering_var, command=self._on_steering_change
        )
        steering_scale.grid(row=1, column=1, sticky="ew", padx=10, pady=(10, 0))
        ttk.Label(motor, textvariable=self.steering_text_var, style="Value.TLabel", width=5).grid(
            row=1, column=2, pady=(10, 0)
        )
        ttk.Label(
            motor, text="首次板测建议 10%，轮子架空", style="Muted.TLabel"
        ).grid(row=2, column=0, columnspan=3, sticky="w", pady=(12, 0))
        drive_buttons = ttk.Frame(motor, style="Card.TFrame")
        drive_buttons.grid(row=3, column=0, columnspan=3, sticky="ew", pady=(12, 0))
        drive_buttons.columnconfigure((0, 1, 2), weight=1)
        reverse = ttk.Button(drive_buttons, text="◀  后退", style="Dark.TButton")
        stop = ttk.Button(drive_buttons, text="■  停止", style="Danger.TButton", command=self.stop_motor)
        forward = ttk.Button(drive_buttons, text="前进  ▶", style="Primary.TButton")
        reverse.grid(row=0, column=0, sticky="ew", padx=(0, 4))
        stop.grid(row=0, column=1, sticky="ew", padx=4)
        forward.grid(row=0, column=2, sticky="ew", padx=(4, 0))
        self._bind_hold_button(reverse, -1)
        self._bind_hold_button(forward, 1)
        self._tip(reverse, "按住后退，松开或移出按钮立即停止。")
        self._tip(stop, "油门、转向和独立双电机输出全部归零，但保留 ARM。")
        self._tip(forward, "按住前进，松开或移出按钮立即停止。")

        servo = ttk.LabelFrame(controls, text="舵机定位", style="Card.TLabelframe", padding=16)
        servo.grid(row=0, column=1, sticky="nsew", padx=(6, 0), pady=(0, 6))
        servo.columnconfigure(0, weight=1)
        ttk.Label(servo, text="目标角度", style="Muted.TLabel").grid(row=0, column=0, sticky="w")
        ttk.Label(servo, textvariable=self.angle_text_var, style="Value.TLabel", font=(self.ui_font, 22, "bold")).grid(
            row=1, column=0, pady=(2, 8)
        )
        angle_scale = ttk.Scale(
            servo, from_=0, to=180, variable=self.angle_var, command=self._on_angle_change
        )
        angle_scale.grid(row=2, column=0, sticky="ew")
        ttk.Label(
            servo, text="拖动后自动发送；接机械结构时先在 90° 附近小范围测试", style="Muted.TLabel",
            wraplength=430,
        ).grid(row=3, column=0, sticky="w", pady=(10, 0))
        angle_buttons = ttk.Frame(servo, style="Card.TFrame")
        angle_buttons.grid(row=4, column=0, sticky="ew", pady=(12, 0))
        angle_buttons.columnconfigure((0, 1, 2), weight=1)
        for column, angle in enumerate((0, 90, 180)):
            angle_button = ttk.Button(
                angle_buttons, text=f"{angle}°", style="Secondary.TButton",
                command=lambda value=angle: self.set_servo(value),
            )
            angle_button.grid(row=0, column=column, sticky="ew", padx=3)
            self._tip(angle_button, f"立即发送舵机 {angle}° 目标；确认机械行程后再使用端点。")

        direct = ttk.LabelFrame(controls, text="双电机独立板测", style="Card.TLabelframe", padding=16)
        direct.grid(row=1, column=0, sticky="nsew", padx=(0, 6), pady=(6, 0))
        direct.columnconfigure(1, weight=1)
        ttk.Label(direct, text="Motor A", style="Card.TLabel").grid(row=0, column=0, sticky="w")
        tk.Scale(
            direct, from_=-100, to=100, resolution=10, orient="horizontal",
            showvalue=False, background=COLORS["card"], highlightthickness=0, variable=self.motor_a_var,
            command=lambda value: self._on_direct_change("a", value),
        ).grid(row=0, column=1, sticky="ew", padx=10)
        ttk.Label(direct, textvariable=self.motor_a_text_var, style="Value.TLabel", width=5).grid(row=0, column=2)
        ttk.Label(direct, text="Motor B", style="Card.TLabel").grid(row=1, column=0, sticky="w", pady=(10, 0))
        tk.Scale(
            direct, from_=-100, to=100, resolution=10, orient="horizontal",
            showvalue=False, background=COLORS["card"], highlightthickness=0, variable=self.motor_b_var,
            command=lambda value: self._on_direct_change("b", value),
        ).grid(row=1, column=1, sticky="ew", padx=10, pady=(10, 0))
        ttk.Label(direct, textvariable=self.motor_b_text_var, style="Value.TLabel", width=5).grid(
            row=1, column=2, pady=(10, 0)
        )
        ttk.Label(
            direct, text="每格 10% 占空比；绕过差速混合，确认通道与方向", style="Muted.TLabel"
        ).grid(row=2, column=0, columnspan=3, sticky="w", pady=(12, 0))
        direct_buttons = ttk.Frame(direct, style="Card.TFrame")
        direct_buttons.grid(row=3, column=0, columnspan=3, sticky="ew", pady=(12, 0))
        direct_buttons.columnconfigure((0, 1), weight=1)
        direct_start = ttk.Button(
            direct_buttons, text="开始独立输出", style="Warning.TButton", command=self.start_direct_motors
        )
        direct_zero = ttk.Button(
            direct_buttons, text="双电机归零", style="Secondary.TButton", command=self.stop_motor
        )
        direct_start.grid(row=0, column=0, sticky="ew", padx=(0, 4))
        direct_zero.grid(row=0, column=1, sticky="ew", padx=(4, 0))
        self._tip(direct_start, "按滑块值持续输出；先架空轮子、低幅值并完成 ARM。")
        self._tip(direct_zero, "停止独立输出并把 A/B 数值归零。")

        auxiliary = ttk.LabelFrame(controls, text="外部接口与 IMU", style="Card.TLabelframe", padding=16)
        auxiliary.grid(row=1, column=1, sticky="nsew", padx=(6, 0), pady=(6, 0))
        auxiliary.columnconfigure((0, 1), weight=1)
        ttk.Label(auxiliary, textvariable=self.power_status_var, style="Value.TLabel").grid(
            row=0, column=0, columnspan=2, sticky="w"
        )
        gpio_assert = ttk.Button(
            auxiliary, text="GPIO35 置位", style="Warning.TButton", command=lambda: self.set_power(True)
        )
        gpio_release = ttk.Button(
            auxiliary, text="GPIO35 释放", style="Secondary.TButton", command=lambda: self.set_power(False)
        )
        gpio_assert.grid(row=1, column=0, sticky="ew", padx=(0, 4), pady=(12, 0))
        gpio_release.grid(row=1, column=1, sticky="ew", padx=(4, 0), pady=(12, 0))
        self._tip(gpio_assert, "激活板上的外部低有效接口；确认原理图和负载后使用。")
        self._tip(gpio_release, "释放外部低有效接口，恢复非激活状态。")
        ttk.Separator(auxiliary).grid(row=2, column=0, columnspan=2, sticky="ew", pady=14)
        ttk.Label(auxiliary, text="IMU 实时摘要", style="Muted.TLabel").grid(row=3, column=0, columnspan=2, sticky="w")
        ttk.Label(
            auxiliary, textvariable=self.imu_status_var, style="Card.TLabel", wraplength=450,
            justify="left",
        ).grid(row=4, column=0, columnspan=2, sticky="w", pady=(6, 0))

        self.record_button = ttk.Button(
            auxiliary, textvariable=self.record_text, command=self.toggle_imu_recording,
            style="Primary.TButton",
        )
        self.record_button.grid(row=5, column=0, sticky="ew", pady=(10, 0))
        self.download_button = ttk.Button(
            auxiliary, text="结束 / 重试下载", command=lambda: self.toggle_imu_recording(True),
            style="Secondary.TButton",
        )
        self.download_button.grid(row=5, column=1, sticky="ew", pady=(10, 0))
        ttk.Label(auxiliary, textvariable=self.record_status, style="Muted.TLabel",
                  wraplength=450).grid(row=6, column=0, columnspan=2, sticky="w")

        diagnostics_page.columnconfigure(0, weight=1)
        diagnostics_page.rowconfigure(1, weight=1)
        diagnostics = ttk.LabelFrame(
            diagnostics_page, text="实时诊断", style="Card.TLabelframe", padding=16
        )
        diagnostics.grid(row=0, column=0, sticky="ew", pady=(0, 10))
        diagnostics.columnconfigure((0, 1), weight=1)
        ttk.Label(diagnostics, textvariable=self.network_status_var, style="Card.TLabel", wraplength=480).grid(
            row=0, column=0, sticky="w"
        )
        self.fault_label = ttk.Label(
            diagnostics, textvariable=self.fault_status_var, style="Card.TLabel", wraplength=480
        )
        self.fault_label.grid(row=0, column=1, sticky="w", padx=(16, 0))
        ttk.Separator(diagnostics).grid(row=1, column=0, columnspan=2, sticky="ew", pady=12)
        ttk.Label(
            diagnostics, textvariable=self.counter_status_var, style="Muted.TLabel", wraplength=980
        ).grid(row=2, column=0, columnspan=2, sticky="w")

        log_frame = ttk.LabelFrame(
            diagnostics_page, text="运行日志", style="Card.TLabelframe", padding=12
        )
        log_frame.grid(row=1, column=0, sticky="nsew")
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)
        self.log_text = scrolledtext.ScrolledText(
            log_frame, wrap=tk.WORD, state="disabled", height=16,
            background="#0B1220", foreground="#CBD5E1", insertbackground="white",
            selectbackground="#1D4ED8", relief="flat", padx=12, pady=10,
            font=(self.ui_font, 9),
        )
        self.log_text.grid(row=0, column=0, sticky="nsew")
        self._log("控制台已就绪。运动前请确认硬件安全并 ARM；光板与 IMU 测试保持 DISARM。")

    def show_help(self) -> None:
        window = tk.Toplevel(self)
        window.title("控制器帮助：按钮与安全说明")
        window.geometry("780x700")
        window.minsize(620, 480)
        window.configure(background=COLORS["canvas"])
        window.transient(self)

        frame = ttk.Frame(window, style="App.TFrame", padding=18)
        frame.pack(fill="both", expand=True)
        frame.columnconfigure(0, weight=1)
        frame.rowconfigure(1, weight=1)

        ttk.Label(
            frame,
            text="按钮功能与使用理由",
            background=COLORS["canvas"], foreground=COLORS["ink"],
            font=(self.ui_font, 18, "bold"),
        ).grid(row=0, column=0, sticky="w", pady=(0, 10))

        text = scrolledtext.ScrolledText(
            frame, wrap=tk.WORD, padx=16, pady=14, relief="flat",
            background=COLORS["card"], foreground=COLORS["ink"],
            selectbackground=COLORS["blue"], font=(self.ui_font, 10),
            spacing1=2, spacing3=4,
        )
        text.grid(row=1, column=0, sticky="nsew")
        text.insert("1.0", HELP_TEXT)
        text.configure(state="disabled")

        ttk.Button(frame, text="关闭帮助", style="Primary.TButton", command=window.destroy).grid(
            row=2, column=0, sticky="e", pady=(10, 0)
        )

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
        output = max(-100, min(100, round(float(value) / 10) * 10))
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

    def scan_devices(self) -> None:
        if self.record_url is not None or self.drive_direction or self.direct_active:
            self._log("请先停止电机输出并结束记录，再扫描切换设备。")
            return
        self.scan_button.configure(state="disabled")
        self._log("正在后台扫描设备；未找到时仍可使用控制台和手动地址。")
        def worker() -> None:
            from start_robot_control import discover_device
            try:
                found = discover_device(None)
                result = ApiResult("scan", True, {"url": found[0] if found else ""})
            except Exception as exc:
                result = ApiResult("scan", False, error=str(exc))
            self.results.put(result)
        threading.Thread(target=worker, daemon=True).start()

    def apply_url_and_refresh(self) -> None:
        if self.record_busy or self.record_url is not None:
            self._log("请先结束记录并完成下载，再切换设备地址。")
            return
        try:
            self.api.set_base_url(self.url_var.get())
        except ValueError as exc:
            self.connection_var.set(f"● {exc}")
            self.connection_label.configure(style="Offline.TLabel")
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

    def toggle_imu_recording(self, finish: bool = False) -> None:
        if self.record_busy:
            return
        self.record_busy = True
        self.record_button.configure(state="disabled")
        self.download_button.configure(state="disabled")
        base_url = self.record_url or self.api.base_url
        self.record_url = base_url
        should_stop = finish or self.recording
        self.record_status.set("正在结束并下载……" if should_stop else "正在启动记录……")

        def worker() -> None:
            try:
                api = RobotApi(base_url)
                if should_stop:
                    status = api.request("POST", "/api/v1/imu/log/stop", timeout=15)
                    if not status.get("sealed"):
                        raise RuntimeError("日志尚未封存：" + str(status.get("error", "无可下载日志")))
                    saved = api.download_imu_log(str(status["file"]))
                    result = ApiResult("imu_record", True, {
                        "recording": False, "saved": str(saved), "error": status.get("error", ""),
                        "dropped": status.get("dropped", 0),
                    })
                else:
                    status = api.request("POST", "/api/v1/imu/log/start", timeout=15)
                    if not status.get("recording"):
                        raise RuntimeError("记录启动失败：" + str(status.get("error", "")))
                    result = ApiResult("imu_record", True, status)
            except (RuntimeError, OSError, ValueError, HTTPException) as exc:
                result = ApiResult("imu_record", False, error=str(exc))
            self.results.put(result)

        threading.Thread(target=worker, daemon=True).start()

    def poll_status(self) -> None:
        if self.closing:
            return
        self._request_async("status", "/api/v1/status")
        if self.record_url is not None and not self.record_busy:
            self._request_async("imu_log_status", "/api/v1/imu/log/status")
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
        if result.operation == "imu_log_status":
            if result.ok and not self.record_busy and self.record_url is not None:
                status = result.payload or {}
                if not status.get("recording"):
                    self.record_text.set("结束 / 下载 IMU 日志")
                    self.record_status.set("设备已结束记录：" + str(status.get("error") or "已封存"))
            return
        if result.operation == "scan":
            self.scan_button.configure(state="normal")
            url = (result.payload or {}).get("url")
            if url and self.record_url is None and not self.drive_direction and not self.direct_active:
                self.url_var.set(url)
                self.apply_url_and_refresh()
            else:
                self._log("扫描完成，未切换设备：" + (result.error or "未找到设备或当前正在操作"))
            return
        if result.operation == "imu_record":
            self.record_busy = False
            self.record_button.configure(state="normal")
            self.download_button.configure(state="normal")
            if not result.ok:
                # A timed-out POST may have succeeded. Keep the device bound and offer an idempotent stop.
                self.recording = True
                self.record_text.set("结束记录 / 重试下载")
                self.record_status.set("操作未完成，可点击结束 / 重试下载")
                self._log("IMU 记录：" + result.error)
                return
            payload = result.payload or {}
            self.recording = bool(payload.get("recording"))
            self.record_text.set("结束记录 IMU" if self.recording else "开始记录 IMU")
            if self.recording:
                self.record_status.set("正在记录：" + str(payload.get("file", "")))
            else:
                self.record_url = None
                self.record_status.set("已下载并设置只读：" + str(payload.get("saved", "")))
            self._log("IMU 记录：" + json.dumps(payload, ensure_ascii=False))
            return
        if not result.ok:
            self.connection_var.set("● 未连接")
            self.connection_label.configure(style="Offline.TLabel")
            if result.operation != "status" or not getattr(self, "_last_status_error", False):
                self._log(f"{result.operation} 失败：{result.error}")
            self._last_status_error = True
            return

        self._last_status_error = False
        payload = result.payload or {}
        self.connection_var.set("● 已连接")
        self.connection_label.configure(style="Online.TLabel")
        if "state" in payload and "armed" in payload:
            armed = "已解锁" if payload.get("armed") else "未解锁"
            self.safety_status_var.set(f"状态：{payload['state']}（{armed}）")
            if payload.get("estop") or int(payload.get("faults", 0)) != 0:
                self.safety_label.configure(style="Offline.TLabel")
            elif payload.get("armed"):
                self.safety_label.configure(style="Armed.TLabel")
            else:
                self.safety_label.configure(style="Safe.TLabel")
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
            fault_mask = int(payload.get("faults", 0))
            self.fault_status_var.set(
                f"故障：mask=0x{fault_mask:08x} "
                f"nFAULT={'有效' if payload.get('drv8833_fault') else '正常'}"
            )
            self.fault_label.configure(style="Offline.TLabel" if fault_mask else "Card.TLabel")
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
        if self.record_busy:
            self._log("日志正在传输，请稍候再关闭窗口。")
            return
        if self.record_url is not None:
            if not messagebox.askyesno("记录尚未完成", "日志尚未确认下载。关闭窗口后，设备可能继续记录至容量上限。仍然关闭？", parent=self):
                return
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
