import contextlib
import queue
import socket
import threading
import time
import tkinter as tk
from dataclasses import dataclass
from tkinter import messagebox
from tkinter import scrolledtext
from tkinter import ttk


FRAME_HEAD1 = 0xFF
FRAME_HEAD2 = 0xFA
FRAME_TAIL1 = 0x88
FRAME_TAIL2 = 0x77

MODE_NONE = 0x00
MODE_WORM = 0x01
MODE_SNAKE_FORWARD = 0x02
MODE_SNAKE_LATERAL = 0x03
MODE_SNAKE_TURN_LEFT = 0x04
MODE_SNAKE_CIRCULAR = MODE_SNAKE_TURN_LEFT
MODE_SNAKE_TURN_RIGHT = 0x05

PORTS = [12340, 12341, 12342, 12343, 12344]
SEGMENT_LABELS = {
    12340: "体节 1",
    12341: "体节 2",
    12342: "体节 3",
    12343: "体节 4",
    12344: "体节 5",
}

UI_REFRESH_MS = 200
SOCKET_TIMEOUT = 0.2
MAX_LOG_LINES = 600


def get_local_ip() -> str:
    """Best-effort local IPv4 detection for display."""
    with contextlib.suppress(OSError):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.connect(("8.8.8.8", 80))
            return sock.getsockname()[0]
        finally:
            sock.close()

    with contextlib.suppress(OSError):
        return socket.gethostbyname(socket.gethostname())

    return "127.0.0.1"


def mode_name(mode: int) -> str:
    return {
        MODE_NONE: "未设置",
        MODE_WORM: "蠕动步态",
        MODE_SNAKE_FORWARD: "蛇形直线",
        MODE_SNAKE_LATERAL: "蛇形侧向",
        MODE_SNAKE_TURN_LEFT: "蛇形左转",
        MODE_SNAKE_TURN_RIGHT: "蛇形右转",
    }.get(mode, f"未知(0x{mode:02X})")


def create_command(command_type: int, parameter: int) -> bytes:
    """Build the 6-byte gait control command used by the ESP32/STM32 stack."""
    if command_type in (MODE_SNAKE_FORWARD, MODE_SNAKE_LATERAL, MODE_SNAKE_TURN_LEFT, MODE_SNAKE_TURN_RIGHT):
        parameter = int(parameter * 255 / 90)
        parameter = max(0, min(255, parameter))
    else:
        parameter = max(0, min(255, parameter))

    return bytes(
        [
            FRAME_HEAD1,
            FRAME_HEAD2,
            command_type,
            parameter,
            FRAME_TAIL1,
            FRAME_TAIL2,
        ]
    )


def decode_line(payload: bytes) -> str:
    payload = payload.strip(b"\r")
    if not payload:
        return ""

    for encoding in ("utf-8", "gbk"):
        with contextlib.suppress(UnicodeDecodeError):
            return payload.decode(encoding)

    ascii_preview = "".join(chr(b) if 32 <= b < 127 else "." for b in payload)
    return f"HEX {payload.hex(' ')} | {ascii_preview}"


@dataclass
class ConnectionState:
    connected: bool = False
    address: str = "-"
    last_seen: float = 0.0
    rx_packets: int = 0
    rx_bytes: int = 0
    ack_count: int = 0
    last_message: str = "-"


class RobotBackend:
    def __init__(self, ports: list[int]):
        self.ports = ports
        self.host_ip = get_local_ip()
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.log_queue: queue.Queue[str] = queue.Queue()
        self.states = {port: ConnectionState() for port in ports}
        self.active_connections: dict[int, socket.socket] = {}
        self.server_sockets: dict[int, socket.socket] = {}
        self.line_buffers = {port: bytearray() for port in ports}
        self.server_threads: list[threading.Thread] = []

    def start(self) -> None:
        for port in self.ports:
            thread = threading.Thread(target=self.server_loop, args=(port,), daemon=True)
            self.server_threads.append(thread)
            thread.start()
        self.log(f"服务器已启动，监听本机 {self.host_ip} 的端口 {', '.join(str(p) for p in self.ports)}")

    def stop(self) -> None:
        self.stop_event.set()

        with self.lock:
            sockets = list(self.active_connections.values())
            servers = list(self.server_sockets.values())
            self.active_connections.clear()
            self.server_sockets.clear()

        for sock in sockets + servers:
            with contextlib.suppress(OSError):
                sock.shutdown(socket.SHUT_RDWR)
            with contextlib.suppress(OSError):
                sock.close()

    def server_loop(self, port: int) -> None:
        try:
            server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server_socket.settimeout(0.5)
            server_socket.bind(("0.0.0.0", port))
            server_socket.listen(1)
        except OSError as exc:
            self.log(f"{SEGMENT_LABELS[port]} 监听失败: {exc}")
            return

        with self.lock:
            self.server_sockets[port] = server_socket

        self.log(f"{SEGMENT_LABELS[port]} 监听启动: 0.0.0.0:{port}")

        try:
            while not self.stop_event.is_set():
                try:
                    client_socket, address = server_socket.accept()
                except socket.timeout:
                    continue
                except OSError:
                    if self.stop_event.is_set():
                        break
                    raise

                client_socket.settimeout(SOCKET_TIMEOUT)
                self.attach_client(port, client_socket, address)
                client_thread = threading.Thread(
                    target=self.client_loop,
                    args=(port, client_socket, address),
                    daemon=True,
                )
                client_thread.start()
        finally:
            with self.lock:
                self.server_sockets.pop(port, None)
            with contextlib.suppress(OSError):
                server_socket.close()

    def attach_client(self, port: int, client_socket: socket.socket, address: tuple[str, int]) -> None:
        old_socket = None
        with self.lock:
            old_socket = self.active_connections.get(port)
            self.active_connections[port] = client_socket
            self.line_buffers[port].clear()

            state = self.states[port]
            state.connected = True
            state.address = f"{address[0]}:{address[1]}"
            state.last_seen = time.time()
            state.last_message = "TCP 已连接"

        if old_socket is not None and old_socket is not client_socket:
            with contextlib.suppress(OSError):
                old_socket.shutdown(socket.SHUT_RDWR)
            with contextlib.suppress(OSError):
                old_socket.close()

        self.log(f"{SEGMENT_LABELS[port]} 已连接: {address[0]}:{address[1]}")

    def client_loop(self, port: int, client_socket: socket.socket, address: tuple[str, int]) -> None:
        try:
            while not self.stop_event.is_set():
                try:
                    data = client_socket.recv(4096)
                except socket.timeout:
                    continue
                except OSError:
                    break

                if not data:
                    break

                self.handle_incoming_data(port, data)
        finally:
            self.detach_client(port, client_socket, address)

    def detach_client(self, port: int, client_socket: socket.socket, address: tuple[str, int]) -> None:
        with self.lock:
            if self.active_connections.get(port) is client_socket:
                self.active_connections.pop(port, None)
                state = self.states[port]
                state.connected = False
                state.address = "-"
                state.last_message = "TCP 已断开"

        with contextlib.suppress(OSError):
            client_socket.close()

        self.log(f"{SEGMENT_LABELS[port]} 已断开: {address[0]}:{address[1]}")

    def handle_incoming_data(self, port: int, data: bytes) -> None:
        now = time.time()
        ack_hits = data.count(0xAA)
        clean_data = bytes(b for b in data if b != 0xAA)

        with self.lock:
            state = self.states[port]
            state.connected = True
            state.last_seen = now
            state.rx_packets += 1
            state.rx_bytes += len(data)
            state.ack_count += ack_hits
            buffer = self.line_buffers[port]

        if ack_hits:
            message = f"{SEGMENT_LABELS[port]} 收到 ACK x{ack_hits}"
            with self.lock:
                self.states[port].last_message = message
            self.log(message)

        if clean_data:
            with self.lock:
                buffer = self.line_buffers[port]
                buffer.extend(clean_data)
                lines: list[bytes] = []

                while b"\n" in buffer:
                    raw_line, remainder = buffer.split(b"\n", 1)
                    lines.append(bytes(raw_line))
                    buffer.clear()
                    buffer.extend(remainder)

                if len(buffer) > 256:
                    lines.append(bytes(buffer))
                    buffer.clear()

            for raw_line in lines:
                decoded = decode_line(raw_line)
                if not decoded:
                    continue
                with self.lock:
                    self.states[port].last_message = decoded
                self.log(f"{SEGMENT_LABELS[port]}: {decoded}")

    def send_command(self, ports: list[int], command_type: int, parameter: int) -> tuple[int, int, bytes]:
        command = create_command(command_type, parameter)
        success_count = 0

        for port in ports:
            with self.lock:
                client_socket = self.active_connections.get(port)

            if client_socket is None:
                self.log(f"{SEGMENT_LABELS[port]} 未连接，跳过发送")
                continue

            try:
                client_socket.sendall(command)
            except OSError as exc:
                self.log(f"{SEGMENT_LABELS[port]} 发送失败: {exc}")
                continue

            success_count += 1
            self.log(
                f"{SEGMENT_LABELS[port]} 已发送 {mode_name(command_type)} 参数={parameter} "
                f"命令={command.hex(' ')}"
            )

        return success_count, len(ports), command

    def snapshot(self) -> dict[int, ConnectionState]:
        with self.lock:
            return {
                port: ConnectionState(
                    connected=state.connected,
                    address=state.address,
                    last_seen=state.last_seen,
                    rx_packets=state.rx_packets,
                    rx_bytes=state.rx_bytes,
                    ack_count=state.ack_count,
                    last_message=state.last_message,
                )
                for port, state in self.states.items()
            }

    def connected_count(self) -> int:
        with self.lock:
            return sum(1 for state in self.states.values() if state.connected)

    def log(self, message: str) -> None:
        timestamp = time.strftime("%H:%M:%S")
        self.log_queue.put(f"[{timestamp}] {message}")


class SegmentCard:
    def __init__(self, parent: ttk.Frame, title: str):
        self.frame = ttk.LabelFrame(parent, text=title, padding=10)
        self.status_var = tk.StringVar(value="未连接")
        self.peer_var = tk.StringVar(value="-")
        self.packets_var = tk.StringVar(value="0")
        self.bytes_var = tk.StringVar(value="0")
        self.acks_var = tk.StringVar(value="0")
        self.last_seen_var = tk.StringVar(value="-")
        self.message_var = tk.StringVar(value="-")

        self.status_label = tk.Label(self.frame, textvariable=self.status_var, fg="#b22222")
        self.status_label.grid(row=0, column=0, columnspan=2, sticky="w")

        self._add_row("对端", self.peer_var, 1)
        self._add_row("收包", self.packets_var, 2)
        self._add_row("字节", self.bytes_var, 3)
        self._add_row("ACK", self.acks_var, 4)
        self._add_row("最近活跃", self.last_seen_var, 5)
        self._add_row("最近消息", self.message_var, 6)

    def _add_row(self, label: str, variable: tk.StringVar, row: int) -> None:
        ttk.Label(self.frame, text=f"{label}:").grid(row=row, column=0, sticky="w", padx=(0, 8), pady=2)
        ttk.Label(self.frame, textvariable=variable).grid(row=row, column=1, sticky="w", pady=2)

    def update(self, state: ConnectionState) -> None:
        if state.connected:
            self.status_var.set("已连接")
            self.status_label.configure(fg="#1b7f3b")
        else:
            self.status_var.set("未连接")
            self.status_label.configure(fg="#b22222")

        self.peer_var.set(state.address)
        self.packets_var.set(str(state.rx_packets))
        self.bytes_var.set(str(state.rx_bytes))
        self.acks_var.set(str(state.ack_count))
        self.message_var.set((state.last_message or "-")[:40])

        if state.last_seen > 0:
            self.last_seen_var.set(time.strftime("%H:%M:%S", time.localtime(state.last_seen)))
        else:
            self.last_seen_var.set("-")


class RobotNoImuApp(tk.Tk):
    def __init__(self, backend: RobotBackend):
        super().__init__()
        self.backend = backend
        self.title("五体节运动控制器（无 IMU）")
        self.geometry("1280x860")
        self.minsize(1100, 760)

        self.selected_vars = {port: tk.BooleanVar(value=True) for port in PORTS}
        self.status_text = tk.StringVar(value="服务器已启动")
        self.connected_text = tk.StringVar(value="0 / 5 已连接")
        self.host_text = tk.StringVar(
            value=f"本机 IP: {self.backend.host_ip}    监听端口: {', '.join(str(p) for p in PORTS)}"
        )
        self.cards: dict[int, SegmentCard] = {}

        self.build_ui()
        self.protocol("WM_DELETE_WINDOW", self.on_close)
        self.after(UI_REFRESH_MS, self.refresh_ui)
        self.after(UI_REFRESH_MS, self.flush_logs)

    def build_ui(self) -> None:
        self.columnconfigure(0, weight=1)
        self.rowconfigure(2, weight=1)

        header = ttk.Frame(self, padding=12)
        header.grid(row=0, column=0, sticky="ew")
        header.columnconfigure(0, weight=1)

        ttk.Label(header, text="五体节运动控制器（无 IMU）", font=("Microsoft YaHei UI", 18, "bold")).grid(
            row=0, column=0, sticky="w"
        )
        ttk.Label(header, textvariable=self.host_text).grid(row=1, column=0, sticky="w", pady=(6, 0))
        ttk.Label(header, textvariable=self.connected_text).grid(row=2, column=0, sticky="w", pady=(6, 0))

        main = ttk.Frame(self, padding=(12, 0, 12, 12))
        main.grid(row=1, column=0, sticky="nsew")
        main.columnconfigure(0, weight=3)
        main.columnconfigure(1, weight=2)
        main.rowconfigure(0, weight=1)

        cards_frame = ttk.LabelFrame(main, text="通信连接状态", padding=10)
        cards_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 8))
        for index, port in enumerate(PORTS):
            cards_frame.columnconfigure(index % 2, weight=1)
            cards_frame.rowconfigure(index // 2, weight=1)
            card = SegmentCard(cards_frame, f"{SEGMENT_LABELS[port]} / 端口 {port}")
            card.frame.grid(row=index // 2, column=index % 2, sticky="nsew", padx=6, pady=6)
            self.cards[port] = card

        control_frame = ttk.Frame(main)
        control_frame.grid(row=0, column=1, sticky="nsew")
        control_frame.rowconfigure(2, weight=1)
        control_frame.columnconfigure(0, weight=1)

        target_group = ttk.LabelFrame(control_frame, text="发送目标", padding=10)
        target_group.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        ttk.Label(target_group, text="勾选要控制的体节：").grid(row=0, column=0, columnspan=3, sticky="w")

        for idx, port in enumerate(PORTS):
            ttk.Checkbutton(
                target_group,
                text=f"{SEGMENT_LABELS[port]} ({port})",
                variable=self.selected_vars[port],
            ).grid(row=1 + idx // 2, column=idx % 2, sticky="w", padx=(0, 12), pady=4)

        ttk.Button(target_group, text="全选", command=self.select_all).grid(row=4, column=0, sticky="ew", pady=(8, 0))
        ttk.Button(target_group, text="全不选", command=self.clear_all).grid(row=4, column=1, sticky="ew", pady=(8, 0))

        command_group = ttk.LabelFrame(control_frame, text="步态命令", padding=10)
        command_group.grid(row=1, column=0, sticky="ew", pady=(0, 8))
        command_group.columnconfigure(1, weight=1)

        self.worm_spin = tk.Spinbox(command_group, from_=1, to=8, width=8)
        self.worm_spin.delete(0, "end")
        self.worm_spin.insert(0, "1")
        self._add_command_row(
            command_group,
            0,
            "蠕动步态",
            self.worm_spin,
            lambda: self.send_selected_command(MODE_WORM, int(self.worm_spin.get())),
            "发送",
        )

        self.snake_forward_spin = tk.Spinbox(command_group, from_=0, to=90, width=8)
        self.snake_forward_spin.delete(0, "end")
        self.snake_forward_spin.insert(0, "30")
        self._add_command_row(
            command_group,
            1,
            "蛇形直线",
            self.snake_forward_spin,
            lambda: self.send_selected_command(MODE_SNAKE_FORWARD, int(self.snake_forward_spin.get())),
            "发送",
        )

        self.snake_lateral_spin = tk.Spinbox(command_group, from_=0, to=90, width=8)
        self.snake_lateral_spin.delete(0, "end")
        self.snake_lateral_spin.insert(0, "30")
        self._add_command_row(
            command_group,
            2,
            "蛇形侧向",
            self.snake_lateral_spin,
            lambda: self.send_selected_command(MODE_SNAKE_LATERAL, int(self.snake_lateral_spin.get())),
            "发送",
        )

        self.snake_turn_left_spin = tk.Spinbox(command_group, from_=0, to=90, width=8)
        self.snake_turn_left_spin.delete(0, "end")
        self.snake_turn_left_spin.insert(0, "30")
        self._add_command_row(
            command_group,
            3,
            "蛇形左转",
            self.snake_turn_left_spin,
            lambda: self.send_selected_command(MODE_SNAKE_TURN_LEFT, int(self.snake_turn_left_spin.get())),
            "发送",
        )

        self.snake_turn_right_spin = tk.Spinbox(command_group, from_=0, to=90, width=8)
        self.snake_turn_right_spin.delete(0, "end")
        self.snake_turn_right_spin.insert(0, "30")
        self._add_command_row(
            command_group,
            4,
            "蛇形右转",
            self.snake_turn_right_spin,
            lambda: self.send_selected_command(MODE_SNAKE_TURN_RIGHT, int(self.snake_turn_right_spin.get())),
            "发送",
        )

        ttk.Button(
            command_group,
            text="发送复位/停止（步态 8）",
            command=lambda: self.send_selected_command(MODE_WORM, 8),
        ).grid(row=5, column=0, columnspan=3, sticky="ew", pady=(10, 0))

        notes_group = ttk.LabelFrame(control_frame, text="说明", padding=10)
        notes_group.grid(row=2, column=0, sticky="nsew")
        notes_group.columnconfigure(0, weight=1)
        ttk.Label(
            notes_group,
            text=(
                "1. 程序启动后自动监听 12340-12344。\n"
                "2. 蠕动步态参数范围 1-8，其中 8 可作为停止/复位。\n"
                "3. 蛇形命令角度范围 0-90，程序会自动换算成协议参数。\n"
                "4. 收到 ESP32 的 ACK 或 STM32 日志后，会显示在下方日志窗口。"
            ),
            justify="left",
        ).grid(row=0, column=0, sticky="nw")

        log_group = ttk.LabelFrame(self, text="运行日志", padding=10)
        log_group.grid(row=2, column=0, sticky="nsew", padx=12, pady=(0, 12))
        log_group.columnconfigure(0, weight=1)
        log_group.rowconfigure(0, weight=1)

        self.log_text = scrolledtext.ScrolledText(log_group, height=14, wrap=tk.WORD)
        self.log_text.grid(row=0, column=0, sticky="nsew")
        self.log_text.configure(state="disabled")

        status_bar = ttk.Frame(self, padding=(12, 0, 12, 12))
        status_bar.grid(row=3, column=0, sticky="ew")
        ttk.Label(status_bar, textvariable=self.status_text).grid(row=0, column=0, sticky="w")

    def _add_command_row(
        self,
        parent: ttk.LabelFrame,
        row: int,
        label: str,
        input_widget: tk.Spinbox,
        command,
        button_text: str,
    ) -> None:
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w", pady=4)
        input_widget.grid(row=row, column=1, sticky="w", padx=6, pady=4)
        ttk.Button(parent, text=button_text, command=command).grid(row=row, column=2, sticky="ew", pady=4)

    def select_all(self) -> None:
        for variable in self.selected_vars.values():
            variable.set(True)

    def clear_all(self) -> None:
        for variable in self.selected_vars.values():
            variable.set(False)

    def selected_ports(self) -> list[int]:
        return [port for port, variable in self.selected_vars.items() if variable.get()]

    def send_selected_command(self, command_type: int, parameter: int) -> None:
        ports = self.selected_ports()
        if not ports:
            self.status_text.set("请先勾选至少一个体节")
            messagebox.showwarning("未选择体节", "请先勾选至少一个体节，再发送命令。")
            return

        success_count, total_count, command = self.backend.send_command(ports, command_type, parameter)
        if success_count == 0:
            self.status_text.set(f"{mode_name(command_type)} 发送失败，没有可用连接")
            return

        self.status_text.set(
            f"{mode_name(command_type)} 参数={parameter} 已发送，成功 {success_count}/{total_count}，命令 {command.hex(' ')}"
        )

    def refresh_ui(self) -> None:
        snapshot = self.backend.snapshot()
        connected_count = self.backend.connected_count()
        self.connected_text.set(f"{connected_count} / {len(PORTS)} 已连接")

        for port, card in self.cards.items():
            card.update(snapshot[port])

        self.after(UI_REFRESH_MS, self.refresh_ui)

    def flush_logs(self) -> None:
        updated = False
        while True:
            try:
                line = self.backend.log_queue.get_nowait()
            except queue.Empty:
                break

            updated = True
            self.log_text.configure(state="normal")
            self.log_text.insert(tk.END, line + "\n")
            self.log_text.configure(state="disabled")

        if updated:
            line_count = int(self.log_text.index("end-1c").split(".")[0])
            if line_count > MAX_LOG_LINES:
                self.log_text.configure(state="normal")
                self.log_text.delete("1.0", f"{line_count - MAX_LOG_LINES}.0")
                self.log_text.configure(state="disabled")
            self.log_text.see(tk.END)

        self.after(UI_REFRESH_MS, self.flush_logs)

    def on_close(self) -> None:
        self.backend.stop()
        self.destroy()


def main() -> None:
    backend = RobotBackend(PORTS)
    backend.start()
    app = RobotNoImuApp(backend)
    app.mainloop()


if __name__ == "__main__":
    main()
