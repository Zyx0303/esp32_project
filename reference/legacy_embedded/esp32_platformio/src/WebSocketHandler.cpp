#include "WebSocketHandler.h"
#include <Arduino.h>
#include <WiFi.h>

WebSocketHandler::WebSocketHandler(const char* serverIP, uint16_t port)
    : serverIP(serverIP), port(port) {}

void WebSocketHandler::begin() {
    webSocket.begin(serverIP, port, "/");
    webSocket.onEvent([this](WStype_t type, uint8_t* payload, size_t length) {
        this->handleWebSocketEvent(type, payload, length);
    });
    webSocket.setReconnectInterval(5000);  // 5秒自动重连
}

void WebSocketHandler::loop() {
    webSocket.loop();
}

void WebSocketHandler::sendTextData(const String& message) {
    if (webSocket.isConnected()) {
        webSocket.sendTXT(message.c_str());  // 使用 c_str() 将 String 转换为 const char*
    } else {
        Serial.println("[WARNING] WebSocket not connected. Message not sent.");
    }
}

void WebSocketHandler::sendBinaryData(const uint8_t* data, size_t length) {
    if (webSocket.isConnected()) {
        webSocket.sendBIN(data, length);
    } else {
        Serial.println("[WARNING] WebSocket not connected. Binary data not sent.");
    }
}

bool WebSocketHandler::isConnected() {
    return webSocket.isConnected();  // 去掉 const 限定符
}

void WebSocketHandler::handleWebSocketEvent(WStype_t type, uint8_t* payload, size_t length) {
    switch (type) {
        case WStype_DISCONNECTED:
            Serial.println("[INFO] WebSocket disconnected.");
            break;
        case WStype_CONNECTED:
            Serial.println("[INFO] WebSocket connected.");
            break;
        case WStype_TEXT:
            Serial.printf("[INFO] Received text: %s\n", payload);
            break;
        case WStype_BIN:
            Serial.println("[INFO] Received binary data.");
            break;
        default:
            break;
    }
}
