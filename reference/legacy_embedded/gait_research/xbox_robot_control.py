import contextlib
import socket
import threading
import time
from dataclasses import dataclass
import ctypes
import tkinter as tk
from tkinter import ttk


FRAME_HEAD1 = 0xFF
FRAME_HEAD2 = 0xFA
FRAME_TAIL1 = 0x88
FRAME_TAIL2 = 0x77

MODE_WORM = 0x01
MODE_SNAKE_FORWARD = 0x02
MODE_SNAKE_TURN_LEFT = 0x04
MODE_SNAKE_TURN_RIGHT = 0x05

PORTS = [12340, 12341, 12342, 12343, 12344]
SOCKET_TIMEOUT = 0.2
STATUS_INTERVAL_S = 5.0
POLL_INTERVAL_S = 0.02
SNAKE_AMPLITUDE_DEG = 45

XINPUT_GAMEPAD_DPAD_UP = 0x0001
XINPUT_GAMEPAD_DPAD_DOWN = 0x0002
XINPUT_GAMEPAD_DPAD_LEFT = 0x0004
XINPUT_GAMEPAD_DPAD_RIGHT = 0x0008
XINPUT_GAMEPAD_A = 0x1000
XINPUT_GAMEPAD_B = 0x2000
XINPUT_GAMEPAD_X = 0x4000
XINPUT_GAMEPAD_Y = 0x8000


class XINPUT_GAMEPAD(ctypes.Structure):
    _fields_ = [
        ("wButtons", ctypes.c_ushort),
        ("bLeftTrigger", ctypes.c_ubyte),
        ("bRightTrigger", ctypes.c_ubyte),
        ("sThumbLX", ctypes.c_short),
        ("sThumbLY", ctypes.c_short),
        ("sThumbRX", ctypes.c_short),
        ("sThumbRY", ctypes.c_short),
    ]


class XINPUT_STATE(ctypes.Structure):
    _fields_ = [
        ("dwPacketNumber", ctypes.c_ulong),
        ("Gamepad", XINPUT_GAMEPAD),
    ]


@dataclass
class ConnectionState:
    connected: bool = False
    address: str = "-"
    rx_packets: int = 0
    rx_bytes: int = 0
    ack_count: int = 0
    last_message: str = "-"
    last_seen: float = 0.0


def get_local_ip() -> str:
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


def create_command(command_type: int, parameter: int) -> bytes:
    if command_type in (MODE_SNAKE_FORWARD, MODE_SNAKE_TURN_LEFT, MODE_SNAKE_TURN_RIGHT):
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

    return payload.hex(" ")


def load_xinput():
    for dll_name in ("xinput1_4", "xinput9_1_0", "xinput1_3"):
        try:
            dll = ctypes.WinDLL(dll_name)
            get_state = dll.XInputGetState
            get_state.argtypes = [ctypes.c_uint, ctypes.POINTER(XINPUT_STATE)]
            get_state.restype = ctypes.c_uint
            return dll_name, get_state
        except Exception:
            continue
    raise RuntimeError("XInput DLL not available")


class RobotServer:
    def __init__(self, ports: list[int]):
        self.ports = ports
        self.host_ip = get_local_ip()
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.states = {port: ConnectionState() for port in ports}
        self.active_connections: dict[int, socket.socket] = {}
        self.server_sockets: dict[int, socket.socket] = {}
        self.line_buffers = {port: bytearray() for port in ports}
        self.threads: list[threading.Thread] = []

    def start(self) -> None:
        for port in self.ports:
            thread = threading.Thread(target=self.server_loop, args=(port,), daemon=True)
            self.threads.append(thread)
            thread.start()
        self.log(
            "server started on {ip} ports {ports}".format(
                ip=self.host_ip,
                ports=", ".join(str(port) for port in self.ports),
            )
        )

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
            self.log(f"port {port} listen failed: {exc}")
            return

        with self.lock:
            self.server_sockets[port] = server_socket

        self.log(f"listening on 0.0.0.0:{port}")

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
                self.threads.append(client_thread)
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
            state.last_message = "connected"

        if old_socket is not None and old_socket is not client_socket:
            with contextlib.suppress(OSError):
                old_socket.shutdown(socket.SHUT_RDWR)
            with contextlib.suppress(OSError):
                old_socket.close()

        self.log(f"segment {port - 12340} connected from {address[0]}:{address[1]}")

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
                state.last_message = "disconnected"

        with contextlib.suppress(OSError):
            client_socket.close()

        self.log(f"segment {port - 12340} disconnected from {address[0]}:{address[1]}")

    def handle_incoming_data(self, port: int, data: bytes) -> None:
        now = time.time()
        ack_hits = data.count(0xAA)
        clean_data = bytes(byte for byte in data if byte != 0xAA)

        with self.lock:
            state = self.states[port]
            state.connected = True
            state.last_seen = now
            state.rx_packets += 1
            state.rx_bytes += len(data)
            state.ack_count += ack_hits
            buffer = self.line_buffers[port]

        if ack_hits:
            with self.lock:
                self.states[port].last_message = f"ack x{ack_hits}"
            self.log(f"segment {port - 12340} ack x{ack_hits}")

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
                self.log(f"segment {port - 12340}: {decoded}")

    def send_command(self, command_type: int, parameter: int) -> tuple[int, int, bytes]:
        command = create_command(command_type, parameter)
        success_count = 0

        for port in self.ports:
            with self.lock:
                client_socket = self.active_connections.get(port)

            if client_socket is None:
                continue

            try:
                client_socket.sendall(command)
            except OSError as exc:
                self.log(f"segment {port - 12340} send failed: {exc}")
                continue

            success_count += 1
            self.log(
                "segment {seg} sent mode=0x{mode:02X} param={param} cmd={cmd}".format(
                    seg=port - 12340,
                    mode=command_type,
                    param=parameter,
                    cmd=command.hex(" "),
                )
            )

        return success_count, len(self.ports), command

    def connected_ports(self) -> list[int]:
        with self.lock:
            return [port for port, state in self.states.items() if state.connected]

    def snapshot(self) -> dict[int, ConnectionState]:
        with self.lock:
            return {
                port: ConnectionState(
                    connected=state.connected,
                    address=state.address,
                    rx_packets=state.rx_packets,
                    rx_bytes=state.rx_bytes,
                    ack_count=state.ack_count,
                    last_message=state.last_message,
                    last_seen=state.last_seen,
                )
                for port, state in self.states.items()
            }

    def print_status(self) -> None:
        with self.lock:
            for port in self.ports:
                state = self.states[port]
                print(
                    "segment={seg} connected={connected} peer={peer} ack={ack} msg={msg}".format(
                        seg=port - 12340,
                        connected=state.connected,
                        peer=state.address,
                        ack=state.ack_count,
                        msg=state.last_message,
                    )
                )

    @staticmethod
    def log(message: str) -> None:
        timestamp = time.strftime("%H:%M:%S")
        print(f"[{timestamp}] {message}", flush=True)


class XboxRobotController:
    def __init__(self, server: RobotServer):
        self.server = server
        self.dll_name, self.get_state = load_xinput()
        self.last_buttons = 0
        self.last_status_at = 0.0
        self.last_motion: tuple[str, int, int] | None = None
        self.state_lock = threading.Lock()
        self.controller_slot: int | None = None
        self.controller_connected = False
        self.last_action_label = "-"
        self.last_action_command = "-"
        self.last_action_result = "-"
        self.last_action_time = 0.0

    def read_state(self) -> tuple[int | None, XINPUT_STATE | None]:
        for slot in range(4):
            state = XINPUT_STATE()
            ret = self.get_state(slot, ctypes.byref(state))
            if ret == 0:
                return slot, state
        return None, None

    def send_action(self, label: str, command_type: int, parameter: int, remember: bool) -> None:
        success_count, total_count, command = self.server.send_command(command_type, parameter)
        command_hex = command.hex(" ")
        with self.state_lock:
            self.last_action_label = label
            self.last_action_command = command_hex
            self.last_action_result = f"{success_count}/{total_count}"
            self.last_action_time = time.time()
        self.server.log(
            "action={label} success={success}/{total} cmd={cmd}".format(
                label=label,
                success=success_count,
                total=total_count,
                cmd=command_hex,
            )
        )
        if remember:
            self.last_motion = (label, command_type, parameter)
        elif label in ("stop", "home_all"):
            self.last_motion = None

    def snapshot(self) -> dict[str, object]:
        with self.state_lock:
            return {
                "slot": self.controller_slot,
                "connected": self.controller_connected,
                "last_action_label": self.last_action_label,
                "last_action_command": self.last_action_command,
                "last_action_result": self.last_action_result,
                "last_action_time": self.last_action_time,
            }

    def handle_button_edges(self, buttons: int, previous_buttons: int) -> None:
        rising = buttons & ~previous_buttons

        if rising & XINPUT_GAMEPAD_A:
            self.send_action("snake_forward_45", MODE_SNAKE_FORWARD, SNAKE_AMPLITUDE_DEG, remember=True)

        if rising & XINPUT_GAMEPAD_B:
            self.send_action("snake_turn_right_45", MODE_SNAKE_TURN_RIGHT, SNAKE_AMPLITUDE_DEG, remember=True)

        if rising & XINPUT_GAMEPAD_X:
            self.send_action("snake_turn_left_45", MODE_SNAKE_TURN_LEFT, SNAKE_AMPLITUDE_DEG, remember=True)

        if rising & XINPUT_GAMEPAD_Y:
            self.send_action("home_all", MODE_WORM, 8, remember=False)

        if rising & XINPUT_GAMEPAD_DPAD_UP:
            self.send_action("worm_gait_1", MODE_WORM, 1, remember=True)

        if rising & XINPUT_GAMEPAD_DPAD_RIGHT:
            self.send_action("worm_gait_2", MODE_WORM, 2, remember=True)

        if rising & XINPUT_GAMEPAD_DPAD_DOWN:
            self.send_action("worm_gait_3", MODE_WORM, 3, remember=True)

        if rising & XINPUT_GAMEPAD_DPAD_LEFT:
            self.send_action("worm_gait_4", MODE_WORM, 4, remember=True)

    def run(self) -> None:
        self.server.log(f"XInput loaded from {self.dll_name}")
        self.server.log("button mapping: A=snake straight, X=left, B=right, Y=home, D-pad=worm 1/2/3/4")
        self.server.log("waiting for ESP32 segment connections and controller input")

        try:
            while not self.server.stop_event.is_set():
                now = time.time()
                if now - self.last_status_at >= STATUS_INTERVAL_S:
                    connected = [port - 12340 for port in self.server.connected_ports()]
                    self.server.log(f"connected segments: {connected}")
                    self.server.print_status()
                    self.last_status_at = now

                slot, state = self.read_state()
                if state is None:
                    with self.state_lock:
                        self.controller_connected = False
                        self.controller_slot = None
                    if self.last_buttons != 0:
                        self.last_buttons = 0
                    time.sleep(POLL_INTERVAL_S)
                    continue

                with self.state_lock:
                    self.controller_connected = True
                    self.controller_slot = slot
                buttons = state.Gamepad.wButtons
                self.handle_button_edges(buttons, self.last_buttons)
                self.last_buttons = buttons
                time.sleep(POLL_INTERVAL_S)
        except KeyboardInterrupt:
            self.server.log("controller loop stopped by user")


class XboxRobotStatusWindow(tk.Tk):
    def __init__(self, server: RobotServer, controller: XboxRobotController):
        super().__init__()
        self.server = server
        self.controller = controller
        self.title("Xbox Robot Control")
        self.geometry("640x360")
        self.minsize(560, 320)

        self.host_var = tk.StringVar(value=f"host: {self.server.host_ip}")
        self.controller_var = tk.StringVar(value="controller: disconnected")
        self.connected_var = tk.StringVar(value="connected segments: []")
        self.last_action_var = tk.StringVar(value="last action: -")
        self.last_command_var = tk.StringVar(value="last command: -")
        self.last_result_var = tk.StringVar(value="last result: -")
        self.segment_vars = {
            port: {
                "status": tk.StringVar(value="disconnected"),
                "peer": tk.StringVar(value="-"),
            }
            for port in PORTS
        }

        self.build_ui()
        self.protocol("WM_DELETE_WINDOW", self.on_close)
        self.after(200, self.refresh_ui)

    def build_ui(self) -> None:
        self.columnconfigure(0, weight=1)
        self.rowconfigure(1, weight=1)

        header = ttk.Frame(self, padding=12)
        header.grid(row=0, column=0, sticky="ew")
        header.columnconfigure(0, weight=1)

        ttk.Label(header, text="Xbox Robot Control", font=("Microsoft YaHei UI", 16, "bold")).grid(
            row=0, column=0, sticky="w"
        )
        ttk.Label(header, textvariable=self.host_var).grid(row=1, column=0, sticky="w", pady=(6, 0))
        ttk.Label(
            header,
            text="A=snake straight  X=left  B=right  Y=home  D-pad Up/Right/Down/Left=worm 1/2/3/4",
        ).grid(row=2, column=0, sticky="w", pady=(6, 0))

        main = ttk.Frame(self, padding=(12, 0, 12, 12))
        main.grid(row=1, column=0, sticky="nsew")
        main.columnconfigure(0, weight=1)
        main.columnconfigure(1, weight=1)
        main.rowconfigure(0, weight=1)

        status_group = ttk.LabelFrame(main, text="Controller / Command", padding=10)
        status_group.grid(row=0, column=0, sticky="nsew", padx=(0, 8))
        status_group.columnconfigure(0, weight=1)

        ttk.Label(status_group, textvariable=self.controller_var).grid(row=0, column=0, sticky="w")
        ttk.Label(status_group, textvariable=self.connected_var).grid(row=1, column=0, sticky="w", pady=(8, 0))
        ttk.Label(status_group, textvariable=self.last_action_var).grid(row=2, column=0, sticky="w", pady=(8, 0))
        ttk.Label(status_group, textvariable=self.last_result_var).grid(row=3, column=0, sticky="w", pady=(8, 0))
        ttk.Label(status_group, textvariable=self.last_command_var, wraplength=260).grid(
            row=4, column=0, sticky="w", pady=(8, 0)
        )

        segment_group = ttk.LabelFrame(main, text="Segments", padding=10)
        segment_group.grid(row=0, column=1, sticky="nsew")
        segment_group.columnconfigure(1, weight=1)

        for row, port in enumerate(PORTS):
            seg_no = port - 12340
            ttk.Label(segment_group, text=f"segment {seg_no}").grid(row=row, column=0, sticky="w", padx=(0, 12), pady=4)
            ttk.Label(segment_group, textvariable=self.segment_vars[port]["status"]).grid(
                row=row, column=1, sticky="w", pady=4
            )
            ttk.Label(segment_group, textvariable=self.segment_vars[port]["peer"]).grid(
                row=row, column=2, sticky="w", padx=(12, 0), pady=4
            )

    def refresh_ui(self) -> None:
        server_state = self.server.snapshot()
        connected = []
        for port in PORTS:
            state = server_state[port]
            seg_no = port - 12340
            if state.connected:
                connected.append(seg_no)
                self.segment_vars[port]["status"].set("connected")
                self.segment_vars[port]["peer"].set(state.address)
            else:
                self.segment_vars[port]["status"].set("disconnected")
                self.segment_vars[port]["peer"].set("-")

        self.connected_var.set(f"connected segments: {connected}")

        controller_state = self.controller.snapshot()
        if controller_state["connected"]:
            self.controller_var.set(f"controller: connected on slot {controller_state['slot']}")
        else:
            self.controller_var.set("controller: disconnected")

        self.last_action_var.set(f"last action: {controller_state['last_action_label']}")
        self.last_result_var.set(f"last result: {controller_state['last_action_result']}")
        self.last_command_var.set(f"last command: {controller_state['last_action_command']}")

        if not self.server.stop_event.is_set():
            self.after(200, self.refresh_ui)

    def on_close(self) -> None:
        self.server.stop()
        self.destroy()


def main() -> None:
    server = RobotServer(PORTS)
    server.start()
    controller = XboxRobotController(server)
    controller_thread = threading.Thread(target=controller.run, daemon=True)
    controller_thread.start()
    try:
        app = XboxRobotStatusWindow(server, controller)
        app.mainloop()
    finally:
        server.stop()


if __name__ == "__main__":
    main()
