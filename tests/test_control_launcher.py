"""Unit tests for the board discovery launcher's pure helpers."""

from __future__ import annotations

import importlib.util
import ipaddress
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "start_robot_control", ROOT / "tools" / "start_robot_control.py"
)
assert SPEC is not None and SPEC.loader is not None
LAUNCHER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(LAUNCHER)


class ControlLauncherTests(unittest.TestCase):
    def test_normalize_url(self) -> None:
        self.assertEqual(LAUNCHER.normalize_url("172.26.96.61/"), "http://172.26.96.61")

    def test_device_identity_is_strict(self) -> None:
        valid = {"ok": True, "api_version": 1, "target": "esp32s3"}
        self.assertTrue(LAUNCHER.is_robot_device(valid))
        self.assertFalse(LAUNCHER.is_robot_device({"ok": True, "api_version": 1}))
        self.assertFalse(LAUNCHER.is_robot_device({"ok": True, "api_version": 2, "target": "esp32s3"}))

    def test_subnet_candidates_exclude_local_address(self) -> None:
        candidates = list(LAUNCHER.subnet_candidates(ipaddress.IPv4Address("172.26.96.110")))
        self.assertEqual(len(candidates), 253)
        self.assertIn("http://172.26.96.61", candidates)
        self.assertNotIn("http://172.26.96.110", candidates)

    def test_local_addresses_are_unique_and_non_loopback(self) -> None:
        addresses = LAUNCHER.local_ipv4_addresses()
        self.assertEqual(len(addresses), len(set(addresses)))
        self.assertTrue(all(not address.is_loopback for address in addresses))


if __name__ == "__main__":
    unittest.main()
