"""Static contract checks for the optional embedded control-strategy seam."""

from __future__ import annotations

import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


class ControlStrategyContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.header = (ROOT / "components/robot_controller/control_strategy.h").read_text(
            encoding="utf-8"
        )
        cls.control = (ROOT / "main/app_control.c").read_text(encoding="utf-8")
        cls.app = (ROOT / "main/app_main.c").read_text(encoding="utf-8")

    def test_interface_reserves_feedback_motor_and_servo_fields(self) -> None:
        for token in (
            "robot_strategy_input_t",
            "robot_status_t status",
            "delta_us",
            "ROBOT_STRATEGY_MOTOR_DRIVE",
            "ROBOT_STRATEGY_MOTOR_DIRECT",
            "servo_valid",
            "servo_angle_deg",
        ):
            self.assertIn(token, self.header)

    def test_strategy_is_optional_by_default(self) -> None:
        self.assertNotIn("app_control_register_strategy", self.app)
        self.assertRegex(self.control, r"if\s*\(s_strategy_ops\s*==\s*NULL\)")

    def test_strategy_uses_unified_safety_manager(self) -> None:
        self.assertIn("run_registered_strategy(now_us)", self.control)
        self.assertIn("control_manager_submit", self.control)
        self.assertIn("ROBOT_COMMAND_SOURCE_INTERNAL", self.control)
        self.assertNotRegex(
            self.control,
            r"run_registered_strategy[\s\S]*?(drv8833_set_motor|servo_pwm_set_angle)",
        )

    def test_strategy_failure_latches_internal_fault(self) -> None:
        self.assertIn("ROBOT_FAULT_INTERNAL", self.control)
        self.assertRegex(
            self.control,
            r"s_strategy_ops->step[\s\S]*?control_manager_report_hardware_fault",
        )


if __name__ == "__main__":
    unittest.main()
