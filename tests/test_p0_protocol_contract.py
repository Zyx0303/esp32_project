"""Source-level contract checks for the versioned P0 HTTP interface."""

from __future__ import annotations

import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
COMPONENT = ROOT / "components" / "robot_controller"


class P0ProtocolContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.wifi = (COMPONENT / "wifi_control.c").read_text(encoding="utf-8")
        cls.types = (COMPONENT / "robot_types.h").read_text(encoding="utf-8")
        cls.gui = (ROOT / "tools/robot_control_gui.py").read_text(encoding="utf-8")

    def registered_routes(self) -> dict[str, str]:
        return {
            path: method
            for path, method in re.findall(
                r"\.uri\s*=\s*\"([^\"]+)\"\s*,\s*\.method\s*=\s*(HTTP_[A-Z]+)",
                self.wifi,
            )
        }

    def test_v1_route_methods_are_frozen(self) -> None:
        expected = {
            "/api/v1/status": "HTTP_GET",
            "/api/v1/device": "HTTP_GET",
            "/api/v1/arm": "HTTP_POST",
            "/api/v1/disarm": "HTTP_POST",
            "/api/v1/estop": "HTTP_POST",
            "/api/v1/fault/clear": "HTTP_POST",
            "/api/v1/drive": "HTTP_POST",
            "/api/v1/motors": "HTTP_POST",
            "/api/v1/servo": "HTTP_POST",
            "/api/v1/power": "HTTP_POST",
        }
        routes = self.registered_routes()
        for path, method in expected.items():
            with self.subTest(path=path):
                self.assertEqual(routes.get(path), method)

    def test_mutating_v1_routes_never_use_get(self) -> None:
        for path, method in self.registered_routes().items():
            if path.startswith("/api/v1/") and path not in {
                "/api/v1/status",
                "/api/v1/device",
            }:
                with self.subTest(path=path):
                    self.assertEqual(method, "HTTP_POST")

    def test_unified_command_carries_sequence_time_and_source(self) -> None:
        command = re.search(
            r"typedef\s+struct\s*\{(?P<body>.*?)\}\s*robot_command_t\s*;",
            self.types,
            re.S,
        )
        self.assertIsNotNone(command)
        body = command.group("body")
        for field in ("sequence", "received_at_us", "source"):
            with self.subTest(field=field):
                self.assertRegex(body, rf"\b{field}\s*;")

    def test_protocol_exposes_required_command_types(self) -> None:
        for command in (
            "ARM",
            "DISARM",
            "ESTOP",
            "CLEAR_FAULT",
            "DRIVE",
            "MOTOR_DIRECT",
            "SERVO",
            "POWER_CONTROL",
        ):
            with self.subTest(command=command):
                self.assertIn(f"ROBOT_CMD_{command}", self.types)

    def test_http_uses_unified_command_submission(self) -> None:
        self.assertRegex(
            self.wifi,
            r"(?:control_manager_submit|command_router_submit|s_submit_cb)\s*\(",
            "HTTP handlers must submit the shared robot_command_t model",
        )

    def test_desktop_gui_uses_v1_safety_and_control_routes(self) -> None:
        for path in (
            "/api/v1/status",
            "/api/v1/arm",
            "/api/v1/disarm",
            "/api/v1/estop",
            "/api/v1/fault/clear",
            "/api/v1/drive",
            "/api/v1/servo",
        ):
            with self.subTest(path=path):
                self.assertIn(path, self.gui)
        for legacy_path in ('"/api/status"', '"/api/motor"', '"/api/servo"', '"/api/stop"'):
            with self.subTest(legacy_path=legacy_path):
                self.assertNotIn(legacy_path, self.gui)


if __name__ == "__main__":
    unittest.main()
