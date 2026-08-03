#include <WiFi.h>
#include "FS.h"
#include "SD_MMC.h"

#define BAUD_RATE 921600
#define BUFFER_SIZE 4600

// Legacy reference only: fill local credentials before running this archived sketch.
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";
const char* serverIP = "192.168.3.245";
const uint16_t port = 12343;
WiFiClient client;

char buffer[BUFFER_SIZE];
int bufferIndex = 0;

void setup() {
  Serial.begin(BAUD_RATE);
  Serial2.begin(BAUD_RATE, SERIAL_8N1, 16, 17);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("Connected to WiFi");

  // Try to connect to the server
  if (client.connect(serverIP, port)) {
    Serial.println("Connected to server");
  } else {
    Serial.println("Failed to connect to server");
  }

  // Initialize SD card
  if (!SD_MMC.begin()) {
    Serial.println("Card Mount Failed");
    return;
  }
  uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("No SD_MMC card attached");
    return;
  }
  Serial.println("SD_MMC Card initialized");
}

void loop() {

  if (client.connected()) {
    while (Serial2.available()) {
      uint8_t c = Serial2.read();
//       // Serial.print("get from 32: ");
//       // Serial.println((unsigned long)c, 16);  // 打印接收到的字符
//       Serial.println(c);
//       // Serial.println((unsigned long)c, 10);
//       // Serial.println(char(c));
//       // Serial.write(c);
//       // client.write(c);

      buffer[bufferIndex++] = c;
      if (bufferIndex == BUFFER_SIZE) {
        client.write(buffer, bufferIndex);

        bufferIndex = 0;  // reset buffer index
      }
//     }
    // while (Serial.available()) {
    //   char c = Serial.read();
    //   Serial2.write(c);
    //   client.write(c);
    //  }
    while (client.available()) {
      char c = client.read();
      uint8_t byteToSend = static_cast<uint8_t>(c);
      Serial.print("Received from server: ");
      Serial.println((unsigned long)byteToSend, 16);
      Serial2.write(byteToSend);
    }
  }
  }
  else {
    Serial.println("Lost connection to server. Reconnecting...");
    client.connect(serverIP, port);  // Try to reconnect
    delay(5000);                     // Wait for 5 seconds before trying again
  }

//   // delay(1);
}

void writeToSD(char* data, int length) {
  File file = SD_MMC.open("/data.txt", FILE_APPEND);
  if (!file) {
    Serial.println("文件打开失败！");
  } else {
    file.write((const uint8_t*)data, length);
    file.close();
  }
}
