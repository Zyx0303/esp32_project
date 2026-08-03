#!/usr/bin/env python3
"""
蛇形机器人验证测试脚本
用于在没有Xbox手柄的情况下手动测试各功能
"""

import socket
import time
import struct
import argparse
from dataclasses import dataclass
from typing import Optional

# 通信协议常量
FRAME_HEAD1 = 0xFF
FRAME_HEAD2 = 0xFA
FRAME_TAIL1 = 0x88
FRAME_TAIL2 = 0x77

# 命令类型
CMD_WORM_MODE = 0x01
CMD_SNAKE_FORWARD = 0x02
CMD_SNAKE_LATERAL = 0x03
CMD_SNAKE_CIRCULAR = 0x04
CMD_SNAKE_TURN_LEFT = 0x03  # 侧向可复用
CMD_SNAKE_TURN_RIGHT = 0x04

# ESP32默认配置
DEFAULT_ESP_IP = "192.168.110.16"
DEFAULT_ESP_PORT = 12340


@dataclass
class TestResult:
    passed: bool
    message: str
    details: Optional[str] = None


class SnakeRobotTester:
    def __init__(self, esp_ip: str = DEFAULT_ESP_IP, esp_port: int = DEFAULT_ESP_PORT):
        self.esp_ip = esp_ip
        self.esp_port = esp_port
        self.sock: Optional[socket.socket] = None

    def connect(self) -> TestResult:
        """测试与ESP32的TCP连接"""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(5)
            self.sock.connect((self.esp_ip, self.esp_port))
            return TestResult(True, f"成功连接到 ESP32 ({self.esp_ip}:{self.esp_port})")
        except socket.timeout:
            return TestResult(False, "连接超时", "检查ESP32是否上电，IP地址是否正确")
        except socket.error as e:
            return TestResult(False, f"连接失败: {e}", "检查WiFi连接和TCP服务器配置")
        finally:
            if self.sock:
                self.sock.close()
                self.sock = None

    def send_command(self, cmd_type: int, param: int = 0) -> TestResult:
        """发送控制命令"""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(5)
            self.sock.connect((self.esp_ip, self.esp_port))

            # 构造命令帧
            command = bytes([FRAME_HEAD1, FRAME_HEAD2, cmd_type, param, FRAME_TAIL1, FRAME_TAIL2])
            self.sock.sendall(command)

            # 等待确认
            ack = self.sock.recv(1)
            if ack == b'\xAA':
                cmd_name = self._get_command_name(cmd_type)
                return TestResult(True, f"命令发送成功: {cmd_name} (param={param})", f"命令帧: {command.hex()}")
            else:
                return TestResult(False, f"收到异常响应: {ack.hex() if ack else '空'}")
        except socket.timeout:
            return TestResult(False, "命令发送超时", "ESP32未响应")
        except socket.error as e:
            return TestResult(False, f"发送失败: {e}")
        finally:
            if self.sock:
                self.sock.close()
                self.sock = None

    def _get_command_name(self, cmd_type: int) -> str:
        names = {
            CMD_WORM_MODE: "蠕虫步态",
            CMD_SNAKE_FORWARD: "蛇形前进",
            CMD_SNAKE_LATERAL: "蛇形侧向",
            CMD_SNAKE_CIRCULAR: "蛇形圆周/转向",
        }
        return names.get(cmd_type, f"未知命令(0x{cmd_type:02X})")

    def test_wifi_connection(self, ssid: str = "国家管网") -> TestResult:
        """测试WiFi连接状态（通过串口输出观察）"""
        return TestResult(
            True,
            f"请观察ESP32串口输出，确认已连接WiFi: {ssid}",
            "串口应显示: 'WiFi已连接' 和 IP地址"
        )

    def test_tcp_server(self) -> TestResult:
        """测试TCP服务器可达性"""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(3)
            result = sock.connect_ex((self.esp_ip, self.esp_port))
            sock.close()
            if result == 0:
                return TestResult(True, f"TCP端口 {self.esp_port} 可达")
            else:
                return TestResult(False, f"TCP端口 {self.esp_port} 不可达", f"错误码: {result}")
        except Exception as e:
            return TestResult(False, f"网络错误: {e}")


def run_auto_test(esp_ip: str, esp_port: int):
    """运行自动测试序列"""
    tester = SnakeRobotTester(esp_ip, esp_port)

    tests = [
        ("WiFi连接检查", lambda: tester.test_wifi_connection()),
        ("TCP端口可达性", tester.test_tcp_server),
        ("TCP连接测试", tester.connect),
    ]

    print("=" * 60)
    print("蛇形机器人自动测试")
    print("=" * 60)

    passed = 0
    failed = 0

    for name, test_func in tests:
        print(f"\n[测试] {name}...")
        result = test_func()
        status = "✓ 通过" if result.passed else "✗ 失败"
        print(f"  {status}: {result.message}")
        if result.details:
            print(f"  详情: {result.details}")

        if result.passed:
            passed += 1
        else:
            failed += 1

    print("\n" + "=" * 60)
    print(f"测试结果: {passed} 通过, {failed} 失败")
    print("=" * 60)

    return failed == 0


def manual_control(esp_ip: str, esp_port: int):
    """手动控制模式"""
    tester = SnakeRobotTester(esp_ip, esp_port)

    print("=" * 60)
    print("手动控制模式")
    print("=" * 60)
    print("可用命令:")
    print("  1 - 蠕虫步态模式")
    print("  2 - 蛇形前进")
    print("  3 - 蛇形侧向运动")
    print("  4 - 蛇形圆周/转向")
    print("  c - 测试连接")
    print("  q - 退出")
    print("=" * 60)

    while True:
        cmd = input("\n输入命令: ").strip().lower()

        if cmd == 'q':
            break
        elif cmd == 'c':
            result = tester.connect()
            print(f"  {'✓' if result.passed else '✗'} {result.message}")
        elif cmd in ('1', '2', '3', '4'):
            cmd_map = {'1': CMD_WORM_MODE, '2': CMD_SNAKE_FORWARD, '3': CMD_SNAKE_LATERAL, '4': CMD_SNAKE_CIRCULAR}
            param = int(input("  输入参数 (0-255): ") or "128")
            result = tester.send_command(cmd_map[cmd], param)
            print(f"  {'✓' if result.passed else '✗'} {result.message}")
            if result.details:
                print(f"    {result.details}")
        else:
            print("  未知命令")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="蛇形机器人测试工具")
    parser.add_argument("--ip", default=DEFAULT_ESP_IP, help=f"ESP32 IP地址 (默认: {DEFAULT_ESP_IP})")
    parser.add_argument("--port", type=int, default=DEFAULT_ESP_PORT, help=f"TCP端口 (默认: {DEFAULT_ESP_PORT})")
    parser.add_argument("--auto", action="store_true", help="运行自动测试")
    parser.add_argument("--manual", action="store_true", help="手动控制模式")

    args = parser.parse_args()

    if args.auto:
        run_auto_test(args.ip, args.port)
    elif args.manual:
        manual_control(args.ip, args.port)
    else:
        # 默认运行自动测试
        run_auto_test(args.ip, args.port)
