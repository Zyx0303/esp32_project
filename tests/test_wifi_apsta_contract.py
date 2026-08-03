"""Source-level contract checks for LAN plus rescue-AP networking."""

from __future__ import annotations

import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
COMPONENT = ROOT / "components" / "robot_controller"


class WifiApStaContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.wifi = (COMPONENT / "wifi_control.c").read_text(encoding="utf-8")
        cls.header = (COMPONENT / "wifi_control.h").read_text(encoding="utf-8")
        cls.example = (COMPONENT / "wifi_credentials.example.h").read_text(encoding="utf-8")
        cls.gitignore = (ROOT / ".gitignore").read_text(encoding="utf-8")

    def test_apsta_keeps_rescue_ap_and_connects_station(self) -> None:
        for token in (
            "WIFI_MODE_APSTA",
            "esp_netif_create_default_wifi_ap",
            "esp_netif_create_default_wifi_sta",
            "esp_wifi_connect",
            "IP_EVENT_STA_GOT_IP",
        ):
            with self.subTest(token=token):
                self.assertIn(token, self.wifi)

    def test_network_status_exposes_lan_ip_and_reconnects(self) -> None:
        for field in (
            "sta_configured",
            "sta_connected",
            "sta_ip",
            "reconnect_count",
            "last_disconnect_reason",
            "reconnect_delay_ms",
        ):
            with self.subTest(field=field):
                self.assertIn(field, self.header)
        self.assertIn('"network"', self.wifi.replace('\\"', '"'))

    def test_reconnect_uses_bounded_timer_backoff(self) -> None:
        for token in (
            "STA_RECONNECT_INITIAL_DELAY_MS",
            "STA_RECONNECT_MAX_DELAY_MS",
            "esp_timer_start_once",
            "wifi_event_sta_disconnected_t",
            "disconnect_reason_name",
        ):
            with self.subTest(token=token):
                self.assertIn(token, self.wifi)
        disconnected = self.wifi.split("WIFI_EVENT_STA_DISCONNECTED", 1)[1]
        branch = disconnected.split("IP_EVENT_STA_GOT_IP", 1)[0]
        self.assertNotIn("esp_wifi_connect();", branch)

    def test_credentials_are_local_and_example_is_empty(self) -> None:
        self.assertIn("/components/robot_controller/wifi_credentials.h", self.gitignore)
        self.assertIn('#define ROBOT_WIFI_STA_SSID     ""', self.example)
        self.assertIn('#define ROBOT_WIFI_STA_PASSWORD ""', self.example)


if __name__ == "__main__":
    unittest.main()
