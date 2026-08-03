#include "WiFiHandler.h"

WiFiHandler::WiFiHandler(const char* ssid, const char* password)
    : ssid(ssid), password(password) {}

void WiFiHandler::connect() {
    WiFi.begin(ssid, password);  // 添加密码参数
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.print(".");
    }
    Serial.println("Connected to WiFi");
}
