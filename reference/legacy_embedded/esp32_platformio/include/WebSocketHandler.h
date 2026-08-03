#ifndef WEBSOCKETHANDLER_H
#define WEBSOCKETHANDLER_H

#include <WebSocketsClient.h>

class WebSocketHandler {
public:
    WebSocketHandler(const char* serverIP, uint16_t port);
    void begin();
    void loop();
    void sendTextData(const String& message);
    void sendBinaryData(const uint8_t* data, size_t length);
    bool isConnected();

private:
    const char* serverIP;
    uint16_t port;
    WebSocketsClient webSocket;
    void handleWebSocketEvent(WStype_t type, uint8_t* payload, size_t length);
};

#endif
