"""Source contracts for the desktop controller's safety and diagnostics."""

from __future__ import annotations

import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


class GuiContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.gui = (ROOT / "tools" / "robot_control_gui.py").read_text(encoding="utf-8")

    def test_gui_uses_versioned_endpoints_for_all_controls(self) -> None:
        for endpoint in (
            "/api/v1/drive",
            "/api/v1/motors",
            "/api/v1/servo",
            "/api/v1/power",
            "/api/v1/status",
            "/api/v1/arm",
            "/api/v1/disarm",
            "/api/v1/estop",
        ):
            with self.subTest(endpoint=endpoint):
                self.assertIn(endpoint, self.gui)

    def test_continuous_motor_outputs_have_heartbeat_and_stop_paths(self) -> None:
        self.assertIn("MOTOR_HEARTBEAT_MS", self.gui)
        self.assertIn("_send_drive_heartbeat", self.gui)
        self.assertIn("_send_direct_heartbeat", self.gui)
        self.assertIn('payload={"throttle": 0, "steering": 0}', self.gui)
        self.assertIn('self._request_async("disarm"', self.gui)

    def test_status_panel_exposes_required_diagnostics(self) -> None:
        for token in (
            "drv8833_fault",
            "last_disconnect_reason",
            "reconnect_delay_ms",
            "watchdog_stops",
            "queue_overflow",
            "free_heap",
            "imu_status_var",
        ):
            with self.subTest(token=token):
                self.assertIn(token, self.gui)


if __name__ == "__main__":
    unittest.main()
