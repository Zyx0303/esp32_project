#!/usr/bin/env python3
"""自动发现局域网中的机器人并启动桌面控制界面。

常用方式：
    python tools/start_robot_control.py
    python tools/start_robot_control.py --url http://172.26.96.61
    python tools/start_robot_control.py --find-only

脚本只探测只读的 ``GET /api/v1/device``，不会 ARM 或发送运动命令。
找到设备后才启动 ``robot_control_gui.py``。
"""

from __future__ import annotations

import argparse
import ipaddress
import json
import os
import socket
import subprocess
import sys
import threading
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path
from typing import Callable, Iterable
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


TOOLS_DIR = Path(__file__).resolve().parent
GUI_PATH = TOOLS_DIR / "robot_control_gui.py"
DEVICE_PATH = "/api/v1/device"
RESCUE_URL = "http://192.168.4.1"
DEFAULT_TIMEOUT_SECONDS = 0.80
DEFAULT_WORKERS = 48


def normalize_url(value: str) -> str:
    """把用户输入统一为不带结尾斜杠的 HTTP 基地址。"""
    value = value.strip().rstrip("/")
    if not value:
        raise ValueError("设备地址不能为空")
    if not value.startswith(("http://", "https://")):
        value = "http://" + value
    return value


def is_robot_device(payload: object) -> bool:
    """严格识别本项目设备，避免把同网段的普通网页误认为机器人。"""
    return (
        isinstance(payload, dict)
        and payload.get("ok") is True
        and payload.get("api_version") == 1
        and payload.get("target") == "esp32s3"
    )


def probe_device(base_url: str, timeout: float) -> dict[str, object] | None:
    """探测一个地址；失败只表示该候选不可用，不中断整个局域网扫描。"""
    request = Request(
        normalize_url(base_url) + DEVICE_PATH,
        headers={"Accept": "application/json", "Connection": "close"},
        method="GET",
    )
    try:
        with urlopen(request, timeout=timeout) as response:
            if response.status != 200:
                return None
            payload = json.loads(response.read().decode("utf-8", errors="strict"))
    except (HTTPError, URLError, TimeoutError, OSError, UnicodeError, json.JSONDecodeError):
        return None
    return payload if is_robot_device(payload) else None


def default_route_ipv4() -> ipaddress.IPv4Address | None:
    """查询默认 IPv4 路由所使用的本机地址，不会真正发送 UDP 数据。"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.connect(("8.8.8.8", 80))
        return ipaddress.IPv4Address(sock.getsockname()[0])
    except (OSError, ipaddress.AddressValueError):
        return None
    finally:
        sock.close()


def local_ipv4_addresses() -> list[ipaddress.IPv4Address]:
    """枚举可用 IPv4，兼容电脑同时连接有线网和手机热点的情况。"""
    addresses: set[ipaddress.IPv4Address] = set()
    default = default_route_ipv4()
    if default is not None:
        addresses.add(default)
    try:
        _, _, host_addresses = socket.gethostbyname_ex(socket.gethostname())
    except OSError:
        host_addresses = []
    for value in host_addresses:
        try:
            address = ipaddress.IPv4Address(value)
        except ipaddress.AddressValueError:
            continue
        if not address.is_loopback and not address.is_link_local:
            addresses.add(address)
    return sorted(addresses, key=int)


def subnet_candidates(local_ip: ipaddress.IPv4Address | None) -> Iterable[str]:
    """生成当前 /24 局域网候选；手机热点和本项目现场网络均使用 /24。"""
    if local_ip is None or local_ip.is_loopback or local_ip.is_link_local:
        return ()
    network = ipaddress.IPv4Network(f"{local_ip}/24", strict=False)
    return (f"http://{host}" for host in network.hosts() if host != local_ip)


def discover_device(
    explicit_url: str | None,
    timeout: float = DEFAULT_TIMEOUT_SECONDS,
    workers: int = DEFAULT_WORKERS,
) -> tuple[str, dict[str, object]] | None:
    """按“指定地址、环境变量、救援地址、当前网段”的顺序寻找设备。"""
    preferred: list[str] = []
    for candidate in (explicit_url, os.environ.get("ROBOT_DEVICE_URL"), RESCUE_URL):
        if candidate:
            normalized = normalize_url(candidate)
            if normalized not in preferred:
                preferred.append(normalized)

    for candidate in preferred:
        # 已知地址只探测少数几个，允许更宽裕的超时；局域网批量扫描仍使用短超时。
        payload = probe_device(candidate, max(timeout, 1.0))
        if payload is not None:
            return candidate, payload

    candidates: list[str] = []
    seen: set[str] = set(preferred)
    for local_ip in local_ipv4_addresses():
        for candidate in subnet_candidates(local_ip):
            if candidate not in seen:
                seen.add(candidate)
                candidates.append(candidate)
    if not candidates:
        return None

    # 并发只用于只读设备信息探测；一旦发现目标便取消尚未开始的请求。
    executor = ThreadPoolExecutor(max_workers=max(1, workers))
    futures = {executor.submit(probe_device, url, timeout): url for url in candidates}
    try:
        for future in as_completed(futures):
            payload = future.result()
            if payload is not None:
                found_url = futures[future]
                for pending in futures:
                    pending.cancel()
                executor.shutdown(wait=False, cancel_futures=True)
                return found_url, payload
    finally:
        executor.shutdown(wait=True, cancel_futures=True)
    return None


def discover_devices(
    explicit_url: str | None,
    timeout: float = DEFAULT_TIMEOUT_SECONDS,
    workers: int = DEFAULT_WORKERS,
    on_found: Callable[[str, dict[str, object]], None] | None = None,
) -> list[tuple[str, dict[str, object]]]:
    """扫描所有候选地址，返回已确认身份的机器人列表。"""
    preferred: list[str] = []
    for candidate in (explicit_url, os.environ.get("ROBOT_DEVICE_URL"), RESCUE_URL):
        if candidate:
            normalized = normalize_url(candidate)
            if normalized not in preferred:
                preferred.append(normalized)

    candidates = list(preferred)
    seen = set(preferred)
    for local_ip in local_ipv4_addresses():
        for candidate in subnet_candidates(local_ip):
            if candidate not in seen:
                seen.add(candidate)
                candidates.append(candidate)

    found: list[tuple[str, dict[str, object]]] = []
    with ThreadPoolExecutor(max_workers=max(1, workers)) as executor:
        futures = {
            executor.submit(
                probe_device,
                url,
                max(timeout, 1.0) if url in preferred else timeout,
            ): url
            for url in candidates
        }
        for future in as_completed(futures):
            payload = future.result()
            if payload is None:
                continue
            item = (futures[future], payload)
            found.append(item)
            if on_found is not None:
                on_found(*item)

    return sorted(found, key=lambda item: item[0])


def choose_device(timeout: float) -> str | None:
    """打开设备扫描器，让用户选择已验证的机器人地址。"""
    try:
        import tkinter as tk
        from tkinter import messagebox, ttk
    except ImportError:
        return None

    root = tk.Tk()
    root.title("ESP32-S3 机器人设备扫描器")
    root.geometry("760x520")
    root.minsize(650, 440)
    root.configure(background="#F4F7FB")

    style = ttk.Style(root)
    style.theme_use("clam")
    style.configure("Picker.TFrame", background="#F4F7FB")
    style.configure("PickerCard.TFrame", background="#FFFFFF")
    style.configure(
        "Picker.Treeview", background="#FFFFFF", fieldbackground="#FFFFFF",
        foreground="#172033", rowheight=34, bordercolor="#DDE5F0",
        font=("Microsoft YaHei UI", 10),
    )
    style.configure(
        "Picker.Treeview.Heading", background="#E8EEF7", foreground="#334155",
        relief="flat", padding=(10, 8), font=("Microsoft YaHei UI", 10, "bold"),
    )
    style.map("Picker.Treeview", background=[("selected", "#DBEAFE")], foreground=[("selected", "#1D4ED8")])
    style.configure(
        "PickerPrimary.TButton", background="#2563EB", foreground="white",
        bordercolor="#2563EB", padding=(16, 9), font=("Microsoft YaHei UI", 10, "bold"),
    )
    style.map("PickerPrimary.TButton", background=[("active", "#1D4ED8")])
    style.configure(
        "PickerSecondary.TButton", background="#EDF2F7", foreground="#172033",
        bordercolor="#D7E0EB", padding=(14, 9), font=("Microsoft YaHei UI", 10, "bold"),
    )
    style.map("PickerSecondary.TButton", background=[("active", "#E2E8F0")])
    style.configure("Picker.TEntry", padding=8, fieldbackground="white", bordercolor="#DDE5F0")

    result: list[str | None] = [None]
    rows: dict[str, str] = {}
    scanning = [False]
    closed = [False]

    header = tk.Frame(root, background="#0F172A", padx=20, pady=15)
    header.pack(fill="x")
    tk.Label(
        header, text="DEVICE DISCOVERY", background="#0F172A", foreground="#60A5FA",
        font=("Segoe UI", 9, "bold"),
    ).pack(anchor="w")
    tk.Label(
        header, text="选择一台机器人", background="#0F172A", foreground="white",
        font=("Microsoft YaHei UI", 19, "bold"),
    ).pack(anchor="w")
    tk.Label(
        header, text="自动验证设备身份，再进入安全控制台", background="#0F172A",
        foreground="#94A3B8", font=("Microsoft YaHei UI", 10),
    ).pack(anchor="w", pady=(2, 0))

    outer = ttk.Frame(root, style="Picker.TFrame", padding=18)
    outer.pack(fill="both", expand=True)
    outer.columnconfigure(0, weight=1)
    outer.rowconfigure(1, weight=1)

    local_text = ", ".join(map(str, local_ipv4_addresses())) or "未检测到"
    ttk.Label(
        outer, text=f"电脑局域网地址  {local_text}", background="#F4F7FB",
        foreground="#64748B", font=("Microsoft YaHei UI", 9),
    ).grid(
        row=0, column=0, sticky="w", pady=(0, 10)
    )

    tree = ttk.Treeview(
        outer, columns=("address", "build"), show="headings", height=8,
        style="Picker.Treeview",
    )
    tree.heading("address", text="设备地址")
    tree.heading("build", text="固件版本")
    tree.column("address", width=260, anchor="w")
    tree.column("build", width=280, anchor="w")
    tree.grid(row=1, column=0, sticky="nsew")

    status_var = tk.StringVar(value="准备扫描")
    ttk.Label(
        outer, textvariable=status_var, background="#F4F7FB", foreground="#2563EB",
        font=("Microsoft YaHei UI", 9, "bold"),
    ).grid(row=2, column=0, sticky="w", pady=(10, 8))

    manual = ttk.Frame(outer, style="PickerCard.TFrame", padding=10)
    manual.grid(row=3, column=0, sticky="ew")
    manual.columnconfigure(1, weight=1)
    ttk.Label(
        manual, text="手动地址", background="#FFFFFF", foreground="#334155",
        font=("Microsoft YaHei UI", 10, "bold"),
    ).grid(row=0, column=0, padx=(0, 10))
    manual_var = tk.StringVar()
    manual_entry = ttk.Entry(manual, textvariable=manual_var, style="Picker.TEntry")
    manual_entry.grid(row=0, column=1, sticky="ew")
    ttk.Label(
        manual, text="例：http://192.168.4.1", background="#FFFFFF", foreground="#94A3B8",
        font=("Microsoft YaHei UI", 9),
    ).grid(row=1, column=1, sticky="w", pady=(4, 0))

    buttons = ttk.Frame(outer, style="Picker.TFrame")
    buttons.grid(row=4, column=0, sticky="ew", pady=(12, 0))
    buttons.columnconfigure(0, weight=1)

    ttk.Button(
        buttons,
        text="帮助",
        style="PickerSecondary.TButton",
        command=lambda: messagebox.showinfo(
            "设备扫描帮助",
            "重新扫描：探测当前局域网内运行本项目固件的 ESP32。\n\n"
            "连接所选设备：验证列表中选中的设备，然后打开控制窗口。也可以双击列表项。\n\n"
            "手动地址：已知机器人 IP 时输入，例如 http://192.168.4.1；留空则使用列表选择。\n\n"
            "扫描不到时：确认 ESP32 已启动、电脑与它在同一 2.4 GHz 局域网，"
            "或连接救援热点 ESP32-Robot 后重新扫描。",
            parent=root,
        ),
    ).grid(row=0, column=0, sticky="w")

    scan_button = ttk.Button(buttons, text="重新扫描", style="PickerSecondary.TButton")
    scan_button.grid(row=0, column=1, padx=(8, 0))
    connect_button = ttk.Button(buttons, text="连接所选设备", style="PickerPrimary.TButton")
    connect_button.grid(row=0, column=2, padx=(8, 0))

    def add_result(url: str, device: dict[str, object]) -> None:
        if closed[0] or url in rows:
            return
        build = f"{device.get('build_date', '?')} {device.get('build_time', '?')}"
        item_id = tree.insert("", "end", values=(url, build))
        rows[url] = item_id
        if len(rows) == 1:
            tree.selection_set(item_id)
            tree.focus(item_id)
        status_var.set(f"已找到 {len(rows)} 台机器人，扫描仍在继续……")

    def scan_finished() -> None:
        if closed[0]:
            return
        scanning[0] = False
        scan_button.configure(state="normal")
        status_var.set(
            f"扫描完成，共找到 {len(rows)} 台机器人"
            if rows
            else "扫描完成，未找到机器人；可重新扫描或手动输入地址"
        )

    def scan_worker() -> None:
        discover_devices(
            None,
            timeout=timeout,
            on_found=lambda url, device: root.after(0, add_result, url, device),
        )
        if not closed[0]:
            root.after(0, scan_finished)

    def start_scan() -> None:
        if scanning[0]:
            return
        scanning[0] = True
        rows.clear()
        for item_id in tree.get_children():
            tree.delete(item_id)
        status_var.set("正在扫描当前局域网，请稍候……")
        scan_button.configure(state="disabled")
        threading.Thread(target=scan_worker, daemon=True).start()

    def connect() -> None:
        typed = manual_var.get().strip()
        if typed:
            try:
                url = normalize_url(typed)
            except ValueError as exc:
                messagebox.showerror("地址错误", str(exc), parent=root)
                return
        else:
            selection = tree.selection()
            if not selection:
                messagebox.showinfo("请选择设备", "请先选择扫描到的机器人，或手动输入地址。", parent=root)
                return
            url = str(tree.item(selection[0], "values")[0])

        status_var.set(f"正在验证 {url}……")
        root.update_idletasks()
        if probe_device(url, max(timeout, 1.5)) is None:
            messagebox.showerror("连接失败", f"{url} 不是可访问的机器人设备。", parent=root)
            status_var.set("验证失败，请重新扫描或检查地址")
            return
        result[0] = url
        closed[0] = True
        root.destroy()

    def close() -> None:
        closed[0] = True
        root.destroy()

    scan_button.configure(command=start_scan)
    connect_button.configure(command=connect)
    tree.bind("<Double-1>", lambda _event: connect())
    manual_entry.bind("<Return>", lambda _event: connect())
    root.protocol("WM_DELETE_WINDOW", close)
    root.after(100, start_scan)
    root.mainloop()
    return result[0]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", help="已知设备地址；仍会校验 /api/v1/device")
    parser.add_argument(
        "--find-only",
        action="store_true",
        help="只打印发现结果，不启动图形界面（用于诊断和自动化）",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=DEFAULT_TIMEOUT_SECONDS,
        help=f"单个候选探测超时，默认 {DEFAULT_TIMEOUT_SECONDS:.2f} 秒",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.timeout <= 0:
        print("错误：--timeout 必须大于 0", file=sys.stderr)
        return 2

    if not args.find_only and args.url is None:
        selected_url = choose_device(args.timeout)
        if selected_url is None:
            print("未选择机器人，已取消启动。")
            return 0
        return subprocess.run(
            [sys.executable, str(GUI_PATH), "--url", selected_url], check=False
        ).returncode

    local_addresses = local_ipv4_addresses()
    address_text = ", ".join(map(str, local_addresses)) or "未检测到"
    print(f"电脑当前局域网地址：{address_text}")
    print("正在寻找 ESP32-S3 机器人……")
    found = discover_device(args.url, timeout=args.timeout)
    if found is None:
        print("未找到机器人。请确认：")
        print("  1. ESP32 已上电并完成启动；")
        print("  2. 电脑和 ESP32 连接同一个 2.4 GHz 局域网；")
        print("  3. 或电脑已连接救援热点 ESP32-Robot；")
        print("  4. 也可以用 --url http://设备IP 指定地址。")
        return 1

    base_url, device = found
    print(f"已找到机器人：{base_url}")
    print(
        "设备信息："
        f"target={device.get('target')} api=v{device.get('api_version')} "
        f"build={device.get('build_date', '?')} {device.get('build_time', '?')}"
    )
    if args.find_only:
        return 0

    if not GUI_PATH.is_file():
        print(f"错误：找不到图形界面 {GUI_PATH}", file=sys.stderr)
        return 1

    print("正在启动电脑控制界面……")
    completed = subprocess.run([sys.executable, str(GUI_PATH), "--url", base_url], check=False)
    return completed.returncode


if __name__ == "__main__":
    raise SystemExit(main())
