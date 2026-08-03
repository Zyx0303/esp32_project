#!/usr/bin/env python3
"""ESP32-S3 机器人 HTTP 冒烟与分阶段硬件测试。

默认模式不发送电机、舵机或 GPIO35 输出，只验证设备识别、状态查询和安全状态机。
运动测试必须同时使用 ``--allow-motion`` 和 ``--test``，并在终端输入 ``MOVE`` 确认。
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import dataclass
from typing import Any
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

from start_robot_control import discover_device, normalize_url


REQUEST_TIMEOUT_SECONDS = 1.5
MOTION_PERCENT = 10
MOTION_DURATION_SECONDS = 0.6
HEARTBEAT_SECONDS = 0.2


class SmokeFailure(RuntimeError):
    """Expected test failure with a concise operator-facing message."""


@dataclass
class RobotApi:
    base_url: str

    def request(self, method: str, path: str, payload: dict[str, Any] | None = None) -> dict[str, Any]:
        body = None if payload is None else json.dumps(payload).encode("utf-8")
        headers = {"Accept": "application/json", "Connection": "close"}
        if body is not None:
            headers["Content-Type"] = "application/json"
        request = Request(self.base_url + path, data=body, headers=headers, method=method)
        try:
            with urlopen(request, timeout=REQUEST_TIMEOUT_SECONDS) as response:
                raw = response.read().decode("utf-8", errors="strict")
        except HTTPError as exc:
            detail = exc.read().decode("utf-8", errors="replace")
            raise SmokeFailure(f"{method} {path}: HTTP {exc.code} {detail}") from exc
        except (URLError, TimeoutError, OSError) as exc:
            raise SmokeFailure(f"{method} {path}: 无法连接设备：{exc}") from exc
        try:
            result = json.loads(raw)
        except json.JSONDecodeError as exc:
            raise SmokeFailure(f"{method} {path}: 返回内容不是 JSON") from exc
        if not isinstance(result, dict):
            raise SmokeFailure(f"{method} {path}: JSON 顶层不是对象")
        return result

    def get(self, path: str) -> dict[str, Any]:
        return self.request("GET", path)

    def post(self, path: str, payload: dict[str, Any] | None = None) -> dict[str, Any]:
        return self.request("POST", path, payload)


def require(condition: bool, message: str) -> None:
    if not condition:
        raise SmokeFailure(message)


def require_accepted(response: dict[str, Any], operation: str) -> None:
    require(response.get("ok") is True, f"{operation}: ok 不为 true：{response}")
    require(response.get("accepted") is True, f"{operation}: 命令未被接受：{response}")


def wait_for_state(api: RobotApi, expected: str, timeout: float = 2.0) -> dict[str, Any]:
    deadline = time.monotonic() + timeout
    last: dict[str, Any] = {}
    while time.monotonic() < deadline:
        last = api.get("/api/v1/status")
        if last.get("state") == expected:
            return last
        time.sleep(0.1)
    raise SmokeFailure(f"等待状态 {expected} 超时，最后状态：{last.get('state')}")


def safe_smoke(api: RobotApi) -> dict[str, Any]:
    """无运动验证；结束时设备一定回到 SAFE。"""
    print("[1/6] 读取设备信息")
    device = api.get("/api/v1/device")
    require(device.get("ok") is True, f"设备信息异常：{device}")
    require(device.get("api_version") == 1, f"API 版本不兼容：{device}")
    require(device.get("target") == "esp32s3", f"目标芯片不匹配：{device}")

    print("[2/6] 读取初始状态")
    initial = api.get("/api/v1/status")
    require(initial.get("state") in {"SAFE", "READY"}, f"初始状态不安全：{initial.get('state')}")

    try:
        if initial.get("state") == "READY":
            require_accepted(api.post("/api/v1/disarm"), "预清理 DISARM")
            wait_for_state(api, "SAFE")

        print("[3/6] ARM：只验证 SAFE -> READY，不发送运动输出")
        require_accepted(api.post("/api/v1/arm"), "ARM")
        ready = wait_for_state(api, "READY")
        require(ready.get("armed") is True, "READY 状态下 armed 不为 true")
        for motor in ("motor_a", "motor_b"):
            require(ready.get(motor, {}).get("output") == 0, f"{motor} 在 ARM 后出现非零输出")
        require(ready.get("servo", {}).get("enabled") is False, "ARM 后舵机不应自动启用")

        print("[4/6] DISARM：验证返回 SAFE")
        require_accepted(api.post("/api/v1/disarm"), "DISARM")
        wait_for_state(api, "SAFE")

        print("[5/6] ESTOP：验证锁存急停")
        require_accepted(api.post("/api/v1/estop"), "ESTOP")
        stopped = wait_for_state(api, "ESTOP")
        require(stopped.get("estop") is True, "ESTOP 状态未锁存")

        print("[6/6] CLEAR：验证只回 SAFE，不自动 ARM")
        require_accepted(api.post("/api/v1/fault/clear"), "CLEAR")
        final = wait_for_state(api, "SAFE")
        require(final.get("armed") is False, "CLEAR 后不应自动 ARM")
        return final
    finally:
        # 任何断言或网络异常之后都尽力发送安全命令；失败由固件看门狗兜底。
        try:
            api.post("/api/v1/disarm")
        except SmokeFailure:
            pass


def send_motor_heartbeat(api: RobotApi, motor_a: int, motor_b: int) -> None:
    deadline = time.monotonic() + MOTION_DURATION_SECONDS
    while time.monotonic() < deadline:
        require_accepted(
            api.post("/api/v1/motors", {"motor_a": motor_a, "motor_b": motor_b}),
            "MOTOR_DIRECT",
        )
        time.sleep(HEARTBEAT_SECONDS)


def run_motion_stage(api: RobotApi, stage: str) -> None:
    """执行短时、低输出的单一阶段；finally 始终清零并 DISARM。"""
    require_accepted(api.post("/api/v1/arm"), "ARM before motion")
    wait_for_state(api, "READY")
    try:
        if stage == "motor-a":
            print(f"Motor A：{MOTION_PERCENT}% 持续 {MOTION_DURATION_SECONDS:.1f}s")
            send_motor_heartbeat(api, MOTION_PERCENT, 0)
        elif stage == "motor-b":
            print(f"Motor B：{MOTION_PERCENT}% 持续 {MOTION_DURATION_SECONDS:.1f}s")
            send_motor_heartbeat(api, 0, MOTION_PERCENT)
        elif stage == "servo":
            print("舵机：发送 90°；请确保机械结构允许该角度")
            require_accepted(api.post("/api/v1/servo", {"angle": 90}), "SERVO")
            time.sleep(MOTION_DURATION_SECONDS)
        else:
            raise SmokeFailure(f"未知运动测试：{stage}")
    finally:
        try:
            api.post("/api/v1/motors", {"motor_a": 0, "motor_b": 0})
        except SmokeFailure:
            pass
        try:
            api.post("/api/v1/disarm")
        except SmokeFailure:
            pass


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", help="设备地址；省略时自动发现")
    parser.add_argument(
        "--test",
        choices=("motor-a", "motor-b", "servo"),
        help="在无运动冒烟通过后执行一个指定硬件阶段",
    )
    parser.add_argument(
        "--allow-motion",
        action="store_true",
        help="允许执行 --test 指定的低输出运动阶段",
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="跳过 MOVE 人工确认；只用于受控自动化台架",
    )
    return parser.parse_args()


def validate_motion_options(test: str | None, allow_motion: bool) -> None:
    if test is not None and not allow_motion:
        raise SmokeFailure("指定 --test 时必须同时显式传入 --allow-motion")
    if allow_motion and test is None:
        raise SmokeFailure("--allow-motion 不会单独生效；必须用 --test 指定唯一阶段")


def main() -> int:
    args = parse_args()
    try:
        validate_motion_options(args.test, args.allow_motion)
        found = discover_device(args.url)
        if found is None:
            raise SmokeFailure("未发现 ESP32-S3；请检查网络或用 --url 指定地址")
        base_url, _ = found
        api = RobotApi(normalize_url(base_url))
        print(f"设备：{api.base_url}")
        final = safe_smoke(api)
        print(
            "[PASS] 无运动冒烟："
            f"state={final.get('state')} armed={final.get('armed')} "
            f"ip={final.get('network', {}).get('sta_ip')}"
        )

        if args.test is not None:
            if not args.yes:
                print("\n警告：下一步会产生真实机械输出。")
                print("确认限流电源、轮子悬空、舵机无干涉，并准备急停。")
                if input("输入 MOVE 继续：").strip() != "MOVE":
                    raise SmokeFailure("操作员取消运动测试")
            run_motion_stage(api, args.test)
            print(f"[PASS] 运动阶段完成并已 DISARM：{args.test}")
        return 0
    except (SmokeFailure, ValueError) as exc:
        print(f"[FAIL] {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
