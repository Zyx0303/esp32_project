"""Safety contract tests for the staged hardware smoke tool."""

from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / "tools"
sys.path.insert(0, str(TOOLS))
SPEC = importlib.util.spec_from_file_location("robot_smoke_test", TOOLS / "robot_smoke_test.py")
assert SPEC is not None and SPEC.loader is not None
SMOKE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = SMOKE
SPEC.loader.exec_module(SMOKE)


class RobotSmokeToolTests(unittest.TestCase):
    def test_default_has_no_motion_stage(self) -> None:
        SMOKE.validate_motion_options(None, False)

    def test_test_requires_explicit_allow_motion(self) -> None:
        with self.assertRaises(SMOKE.SmokeFailure):
            SMOKE.validate_motion_options("motor-a", False)

    def test_allow_motion_requires_one_named_stage(self) -> None:
        with self.assertRaises(SMOKE.SmokeFailure):
            SMOKE.validate_motion_options(None, True)

    def test_motion_levels_remain_low_and_short(self) -> None:
        self.assertLessEqual(SMOKE.MOTION_PERCENT, 10)
        self.assertLessEqual(SMOKE.MOTION_DURATION_SECONDS, 1.0)


if __name__ == "__main__":
    unittest.main()
