"""Unit tests for the board discovery launcher's pure helpers."""

from __future__ import annotations

import importlib.util
import ipaddress
import threading
import unittest
from unittest.mock import patch
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

    def test_discover_devices_returns_every_verified_robot(self) -> None:
        devices = {
            "http://192.168.4.1": {
                "ok": True, "api_version": 1, "target": "esp32s3"
            },
            "http://172.26.96.169": {
                "ok": True, "api_version": 1, "target": "esp32s3"
            },
        }
        callbacks: list[str] = []
        callback_lock = threading.Lock()

        def fake_probe(url: str, _timeout: float):
            return devices.get(url)

        def on_found(url: str, _device: dict[str, object]) -> None:
            with callback_lock:
                callbacks.append(url)

        with (
            patch.object(LAUNCHER, "local_ipv4_addresses", return_value=[]),
            patch.object(LAUNCHER, "probe_device", side_effect=fake_probe),
            patch.dict(LAUNCHER.os.environ, {"ROBOT_DEVICE_URL": "http://172.26.96.169"}),
        ):
            found = LAUNCHER.discover_devices(None, on_found=on_found)

        self.assertEqual([url for url, _device in found], sorted(devices))
        self.assertCountEqual(callbacks, devices)

    def test_device_picker_has_help_for_scan_and_manual_connection(self) -> None:
        source = (ROOT / "tools" / "start_robot_control.py").read_text(encoding="utf-8")
        self.assertIn("设备扫描帮助", source)
        self.assertIn("重新扫描：", source)
        self.assertIn("连接所选设备：", source)
        self.assertIn("手动地址：", source)
        self.assertIn("Picker.Treeview", source)
        self.assertIn("PickerPrimary.TButton", source)


if __name__ == "__main__":
    unittest.main()
