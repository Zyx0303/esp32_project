import argparse
import socket
import sys
import time


def main():
    parser = argparse.ArgumentParser(
        description="Check whether ESP32 sends HelloWorld over TCP."
    )
    parser.add_argument("--host", default="0.0.0.0", help="Listen host (default: 0.0.0.0)")
    parser.add_argument("--port", type=int, default=12342, help="Listen port (default: 12342)")
    parser.add_argument(
        "--keyword",
        default="HelloWorld",
        help="Keyword to detect in incoming data (default: HelloWorld)",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=20.0,
        help="Timeout in seconds waiting for keyword (default: 20)",
    )
    args = parser.parse_args()

    keyword_bytes = args.keyword.encode("utf-8")
    deadline = time.time() + args.timeout

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((args.host, args.port))
    server.listen(1)
    server.settimeout(1.0)

    print(f"[INFO] Listening on {args.host}:{args.port}")
    print(f"[INFO] Waiting for ESP32... timeout={args.timeout:.1f}s")

    conn = None
    try:
        while time.time() < deadline:
            try:
                conn, addr = server.accept()
                print(f"[OK] Connected from {addr[0]}:{addr[1]}")
                conn.settimeout(1.0)
                break
            except socket.timeout:
                pass

        if conn is None:
            print("[FAIL] No TCP connection from ESP32 within timeout.")
            sys.exit(1)

        buffer = b""
        while time.time() < deadline:
            try:
                data = conn.recv(4096)
                if not data:
                    print("[FAIL] Connection closed before receiving keyword.")
                    sys.exit(1)

                buffer += data
                text_preview = data.decode("utf-8", errors="replace").strip()
                if text_preview:
                    print(f"[RX] {text_preview}")

                if keyword_bytes in buffer:
                    print(f"[PASS] Detected keyword: {args.keyword}")
                    sys.exit(0)
            except socket.timeout:
                pass

        print(f"[FAIL] Connected, but no '{args.keyword}' received within timeout.")
        sys.exit(2)
    finally:
        if conn is not None:
            conn.close()
        server.close()


if __name__ == "__main__":
    main()
