"""Offline behavioral checks: no firmware compilation or hardware access."""
import argparse
import importlib.util
import io
import stat
import sys
import tempfile
import types
import unittest
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]

def load(name, file):
    spec = importlib.util.spec_from_file_location(name, ROOT / 'tools' / file)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module

GUI = load('imu_test_gui', 'robot_control_gui.py')
LAUNCH = load('imu_test_launch', 'start_robot_control.py')

class Value:
    def set(self, value): self.value = value

class Response(io.BytesIO):
    def __init__(self, body, length=None):
        super().__init__(body)
        self.headers = {} if length is None else {'Content-Length': str(length)}

class ImuDesktopTests(unittest.TestCase):
    def test_direct_steps_are_ten_percent_and_clamped(self):
        app = types.SimpleNamespace(motor_a_var=Value(), motor_b_var=Value(),
                                    motor_a_text_var=Value(), motor_b_text_var=Value())
        for value, expected in [('14', 10), ('26', 30), ('-26', -30), ('107', 100)]:
            GUI.RobotControllerApp._on_direct_change(app, 'a', value)
            self.assertEqual(app.motor_a_var.value, expected)
        GUI.RobotControllerApp._on_direct_change(app, 'b', '-64')
        self.assertEqual(app.motor_b_var.value, -60)

    def test_launcher_opens_without_discovery(self):
        args = argparse.Namespace(url=None, find_only=False, timeout=0.8)
        with patch.object(LAUNCH, 'parse_args', return_value=args), \
             patch.object(LAUNCH, 'discover_device') as discover, \
             patch.object(LAUNCH.subprocess, 'run', return_value=types.SimpleNamespace(returncode=0)) as run:
            self.assertEqual(LAUNCH.main(), 0)
            discover.assert_not_called()
            self.assertIn(str(LAUNCH.GUI_PATH), run.call_args.args[0])

    def test_download_is_complete_unique_and_readonly(self):
        body = b'time_us,ax\n10000,42\n'
        with tempfile.TemporaryDirectory() as tmp, \
             patch.object(GUI, '__file__', str(Path(tmp)/'tools'/'gui.py')), \
             patch.object(GUI, 'urlopen', side_effect=lambda *a, **k: Response(body, len(body))):
            api = GUI.RobotApi('http://robot')
            first = api.download_imu_log('I000001.log')
            second = api.download_imu_log('I000001.log')
            self.assertNotEqual(first, second)
            self.assertEqual(first.read_bytes(), body)
            self.assertFalse(first.stat().st_mode & (stat.S_IWUSR | stat.S_IWGRP | stat.S_IWOTH))
            first.chmod(0o600)
            second.chmod(0o600)

    def test_incomplete_download_leaves_no_final_file(self):
        with tempfile.TemporaryDirectory() as tmp, \
             patch.object(GUI, '__file__', str(Path(tmp)/'tools'/'gui.py')), \
             patch.object(GUI, 'urlopen', return_value=Response(b'short', 100)):
            with self.assertRaises(RuntimeError):
                GUI.RobotApi('http://robot').download_imu_log('I000001.log')
            self.assertEqual(list((Path(tmp)/'imu_logs').iterdir()), [])

if __name__ == '__main__': unittest.main()
