#ifndef WIFIHANDLER_H
#define WIFIHANDLER_H

#include <WiFi.h>

class WiFiHandler {
public:
    WiFiHandler(const char* ssid, const char* password);
    void connect();

private:
    const char* ssid;
    const char* password;
};

#endif // WIFIHANDLER_H
