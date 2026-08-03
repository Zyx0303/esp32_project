import unittest

from board_test import (
    BoardReport,
    build_extended_frame,
    build_legacy_command,
    find_stlink_tool,
    parse_ack,
)


class BoardTestProtocolTests(unittest.TestCase):
    def test_builds_legacy_gait_frame(self):
        self.assertEqual(
            build_legacy_command(0x02, 128),
            bytes.fromhex("ff fa 02 80 88 77"),
        )

    def test_rejects_motion_when_not_allowed(self):
        with self.assertRaises(ValueError):
            build_legacy_command(0x02, 128, allow_motion=False)

    def test_allows_non_motion_extended_ping_without_motion_flag(self):
        self.assertEqual(
            build_extended_frame(0x01, b""),
            bytes.fromhex("fe ef 01 00 01 ef fe"),
        )

    def test_parse_ack(self):
        self.assertTrue(parse_ack(b"\xaa"))
        self.assertFalse(parse_ack(b""))


class BoardTestDiscoveryTests(unittest.TestCase):
    def test_report_records_pass_fail_skip(self):
        report = BoardReport()
        report.pass_("usb", "serial found")
        report.fail("tcp", "timeout")
        report.skip("stlink", "tool missing")
        self.assertEqual(report.counts(), {"PASS": 1, "FAIL": 1, "SKIP": 1})

    def test_find_stlink_tool_prefers_cube_programmer(self):
        def fake_which(name):
            return "/bin/" + name if name == "STM32_Programmer_CLI" else None

        self.assertEqual(find_stlink_tool(fake_which), "STM32_Programmer_CLI")

    def test_find_stlink_tool_falls_back_to_st_info(self):
        def fake_which(name):
            return "/bin/" + name if name == "st-info" else None

        self.assertEqual(find_stlink_tool(fake_which), "st-info")


if __name__ == "__main__":
    unittest.main()
