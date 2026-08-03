"""Host-side checks for P0 safety invariants.

These tests intentionally use only the Python standard library.  They do not
claim electrical behaviour; they guard architectural properties that can be
proved from source before a board is connected.
"""

from __future__ import annotations

import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def source(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def function_body(text: str, function_name: str) -> str:
    """Return a C function body using brace matching, not a fragile line regex."""
    match = re.search(rf"\b{re.escape(function_name)}\s*\([^;]*?\)\s*\{{", text, re.S)
    if match is None:
        raise AssertionError(f"function not found: {function_name}")
    start = text.find("{", match.start())
    depth = 0
    for index in range(start, len(text)):
        if text[index] == "{":
            depth += 1
        elif text[index] == "}":
            depth -= 1
            if depth == 0:
                return text[start + 1 : index]
    raise AssertionError(f"unterminated function: {function_name}")


class P0StaticSafetyTests(unittest.TestCase):
    def test_required_p0_modules_are_registered(self) -> None:
        cmake = source("components/robot_controller/CMakeLists.txt")
        for module in (
            "control_manager.c",
            "robot_state_machine.c",
            "robot_config.c",
        ):
            with self.subTest(module=module):
                self.assertIn(f'"{module}"', cmake)

    def test_state_machine_has_all_safe_states(self) -> None:
        types = source("components/robot_controller/robot_types.h")
        for state in ("BOOT", "SAFE", "READY", "RUNNING", "FAULT", "ESTOP"):
            with self.subTest(state=state):
                self.assertIn(f"ROBOT_STATE_{state}", types)

    def test_safe_stop_neutralizes_all_motion_outputs(self) -> None:
        manager = source("components/robot_controller/control_manager.c")
        body = function_body(manager, "safe_stop")
        self.assertRegex(body, r"set_motors\s*\([^;]*\b0\s*,\s*0\s*\)")
        self.assertRegex(body, r"set_motor_sleep\s*\([^;]*\btrue\s*\)")
        self.assertRegex(body, r"disable_servo\s*\(")

    def test_estop_disarm_timeout_and_hardware_fault_share_safe_stop(self) -> None:
        manager = source("components/robot_controller/control_manager.c")
        submit = function_body(manager, "control_manager_submit")
        tick = function_body(manager, "control_manager_tick")
        fault = function_body(manager, "control_manager_report_hardware_fault")
        for command in ("ROBOT_CMD_ESTOP", "ROBOT_CMD_DISARM"):
            branch = re.search(
                rf"{command}.*?safe_stop\s*\(manager\)", submit, re.S
            )
            self.assertIsNotNone(branch, f"{command} must call safe_stop")
        self.assertRegex(
            tick,
            r"ROBOT_FAULT_COMMAND_TIMEOUT[\s\S]*?safe_stop\s*\(manager\)",
        )
        self.assertIn("safe_stop(manager)", fault)

    def test_motion_commands_are_range_and_state_gated(self) -> None:
        manager = source("components/robot_controller/control_manager.c")
        validator = function_body(manager, "command_has_valid_range")
        submit = function_body(manager, "control_manager_submit")
        for token in ("ROBOT_CMD_DRIVE", "ROBOT_CMD_MOTOR_DIRECT", "ROBOT_CMD_SERVO"):
            self.assertIn(token, validator)
        self.assertGreaterEqual(submit.count("robot_state_machine_allows_motion"), 3)

    def test_safety_commands_precede_stale_and_sequence_rejection(self) -> None:
        manager = function_body(
            source("components/robot_controller/control_manager.c"),
            "control_manager_submit",
        )
        estop = manager.index("ROBOT_CMD_ESTOP")
        disarm = manager.index("ROBOT_CMD_DISARM")
        expired = manager.index("ROBOT_COMMAND_EXPIRED")
        sequence = manager.index("ROBOT_COMMAND_OUT_OF_ORDER")
        self.assertLess(estop, expired)
        self.assertLess(disarm, expired)
        self.assertLess(estop, sequence)

    def test_boot_does_not_command_servo_center(self) -> None:
        app = source("main/app_main.c")
        control = source("main/app_control.c")
        actuators = source("main/app_actuators.c")
        combined = app + control + actuators
        self.assertNotRegex(combined, r"set_servo_angle\s*\(\s*90\s*\)")
        self.assertIn("control_manager_init", control)
        self.assertIn("control_manager_boot_complete", control)

    def test_power_defaults_to_released(self) -> None:
        actuators = source("main/app_actuators.c")
        power = source("components/robot_controller/power_control.c")
        init = function_body(actuators, "app_actuators_init_safe_power")
        self.assertRegex(init, r"power_control_(?:set_asserted|init)[\s\S]*?false")
        self.assertRegex(power, r"gpio_set_level\s*\([^;]*\b0\s*\)")

    def test_config_has_version_validation_and_safe_fallback(self) -> None:
        header = source("components/robot_controller/robot_config.h")
        config = source("components/robot_controller/robot_config.c")
        self.assertRegex(header, r"#define\s+ROBOT_CONFIG_VERSION\s+\d+U?")
        self.assertIn("robot_config_validate", config)
        load = function_body(config, "robot_config_load")
        self.assertGreaterEqual(load.count("robot_config_defaults"), 2)

    def test_status_has_minimum_safety_observability(self) -> None:
        types = source("components/robot_controller/robot_types.h")
        status = re.search(
            r"typedef\s+struct\s*\{(?P<body>.*?)\}\s*robot_status_t\s*;",
            types,
            re.S,
        )
        self.assertIsNotNone(status)
        body = status.group("body")
        for field in (
            "state",
            "armed",
            "estop_latched",
            "faults",
            "motor_a_target",
            "motor_b_target",
            "motor_a_output",
            "motor_b_output",
            "servo_target_deg",
            "servo_enabled",
            "power_asserted",
            "commands_received",
            "commands_rejected",
            "commands_expired",
            "watchdog_stops",
        ):
            with self.subTest(field=field):
                self.assertRegex(body, rf"\b{field}\s*;")

    def test_single_control_task_owns_command_execution(self) -> None:
        app = source("main/app_main.c")
        control = source("main/app_control.c")
        console = source("main/app_console.c")
        control_task = function_body(control, "control_task")
        self.assertIn("xQueueReceive", control_task)
        self.assertIn("control_manager_submit", control_task)
        self.assertIn("control_manager_tick", control_task)
        self.assertRegex(app, r"wifi_control_init\s*\(\s*app_control_enqueue\s*,")
        serial = function_body(console, "parse_serial_command")
        self.assertIn("app_control_enqueue", serial)
        self.assertNotIn("drv8833_set_motor", serial)
        self.assertNotIn("servo_pwm_set_angle", serial)

    def test_full_queue_cannot_discard_estop_or_disarm(self) -> None:
        control = source("main/app_control.c")
        priority = function_body(control, "is_priority_command")
        enqueue = function_body(control, "app_control_enqueue")
        self.assertIn("ROBOT_CMD_ESTOP", priority)
        self.assertIn("ROBOT_CMD_DISARM", priority)
        self.assertIn("xQueueSendToFront", enqueue)
        self.assertIn("xQueueReset", enqueue)
        self.assertRegex(enqueue, r"s_queue_overflow\s*=\s*true")

    def test_app_main_is_only_the_composition_root(self) -> None:
        app = source("main/app_main.c")
        cmake = source("main/CMakeLists.txt")
        for module in (
            "app_status.c",
            "app_actuators.c",
            "app_control.c",
            "app_imu.c",
            "app_console.c",
        ):
            with self.subTest(module=module):
                self.assertIn(f'"{module}"', cmake)
        for implementation_detail in (
            "static void control_task",
            "static void sensor_task",
            "static void parse_serial_command",
            "drv8833_set_motor",
            "servo_pwm_set_angle",
        ):
            with self.subTest(implementation_detail=implementation_detail):
                self.assertNotIn(implementation_detail, app)


if __name__ == "__main__":
    unittest.main()
