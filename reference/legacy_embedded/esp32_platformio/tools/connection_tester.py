import argparse
import socket
import struct
import threading
import time

FRAME_HEAD1 = 0xFF
FRAME_HEAD2 = 0xFA
FRAME_TAIL1 = 0x88
FRAME_TAIL2 = 0x77
ACK_BYTE = 0xAA


def build_command(mode: int, parameter: int) -> bytes:
    if not (0 <= mode <= 255 and 0 <= parameter <= 255):
        raise ValueError("mode and parameter must be in [0, 255]")
    return struct.pack("6B", FRAME_HEAD1, FRAME_HEAD2, mode, parameter, FRAME_TAIL1, FRAME_TAIL2)


class LinkTester:
    def __init__(self, host: str, port: int, imu_size: int):
        self.host = host
        self.port = port
        self.imu_size = imu_size
        self.server = None
        self.conn = None
        self.addr = None
        self.running = True
        self.total_bytes = 0
        self.last_bytes = 0
        self.last_report_time = time.time()
        self.last_packet_time = 0.0

    def start_server(self):
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind((self.host, self.port))
        self.server.listen(1)
        print(f"[INFO] Listening on {self.host}:{self.port}")
        print("[INFO] Waiting for ESP32 to connect...")

        self.conn, self.addr = self.server.accept()
        self.conn.settimeout(1.0)
        print(f"[OK] ESP32 connected from {self.addr[0]}:{self.addr[1]}")

    def recv_loop(self):
        while self.running and self.conn:
            try:
                data = self.conn.recv(4096)
                if not data:
                    print("[WARN] Connection closed by ESP32")
                    self.running = False
                    break

                self.total_bytes += len(data)
                self.last_packet_time = time.time()
                if len(data) == 1 and data[0] == ACK_BYTE:
                    print("[ACK] Received command ACK (0xAA)")
            except socket.timeout:
                continue
            except OSError as e:
                print(f"[ERROR] recv failed: {e}")
                self.running = False
                break

    def report_loop(self):
        while self.running:
            now = time.time()
            if now - self.last_report_time >= 2.0:
                delta = self.total_bytes - self.last_bytes
                rate = delta / (now - self.last_report_time)
                self.last_bytes = self.total_bytes
                self.last_report_time = now

                packet_hint = ""
                if self.imu_size > 0:
                    packet_hint = f", ~{delta / self.imu_size:.2f} IMU packets/2s"

                print(
                    f"[STAT] total={self.total_bytes} B, rate={rate:.1f} B/s{packet_hint}"
                )

                if self.last_packet_time > 0 and now - self.last_packet_time > 5:
                    print("[WARN] No incoming data for >5s")

            time.sleep(0.1)

    def interactive_loop(self):
        print("\nCommands:")
        print("  send <mode> <param>   Example: send 1 0")
        print("  quit                  Exit tester")
        while self.running:
            try:
                raw = input("tester> ").strip()
            except (EOFError, KeyboardInterrupt):
                self.running = False
                break

            if not raw:
                continue
            if raw.lower() in {"quit", "exit", "q"}:
                self.running = False
                break

            parts = raw.split()
            if len(parts) == 3 and parts[0].lower() == "send":
                try:
                    mode = int(parts[1], 0)
                    param = int(parts[2], 0)
                    frame = build_command(mode, param)
                    self.conn.sendall(frame)
                    print(f"[TX] Sent command: mode={mode}, param={param}, bytes={frame.hex(' ')}")
                except Exception as e:
                    print(f"[ERROR] send failed: {e}")
            else:
                print("[INFO] Unknown command")

    def close(self):
        if self.conn:
            try:
                self.conn.close()
            except OSError:
                pass
        if self.server:
            try:
                self.server.close()
            except OSError:
                pass

    def run(self):
        try:
            self.start_server()
            recv_t = threading.Thread(target=self.recv_loop, daemon=True)
            stat_t = threading.Thread(target=self.report_loop, daemon=True)
            recv_t.start()
            stat_t.start()
            self.interactive_loop()
        finally:
            self.running = False
            self.close()
            print("[INFO] Tester stopped")


def main():
    parser = argparse.ArgumentParser(description="ESP32 TCP link tester")
    parser.add_argument("--host", default="0.0.0.0", help="Listen host (default: 0.0.0.0)")
    parser.add_argument("--port", type=int, default=12342, help="Listen port (default: 12342)")
    parser.add_argument("--imu-size", type=int, default=4600, help="IMU frame bytes (default: 4600)")
    args = parser.parse_args()

    tester = LinkTester(args.host, args.port, args.imu_size)
    tester.run()


if __name__ == "__main__":
    main()
