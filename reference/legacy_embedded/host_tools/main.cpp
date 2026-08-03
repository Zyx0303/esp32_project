#include <Arduino.h>
#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define BAUD_RATE 921600
#define IMU_DATA_SIZE 4600

#define FRAME_HEAD1 0xFF
#define FRAME_HEAD2 0xFA
#define FRAME_TAIL1 0x88
#define FRAME_TAIL2 0x77

#define CMD_WORM_MODE      0x01    // 蠕虫步态模式
#define CMD_SNAKE_FORWARD  0x02    // 蛇形直线前进
#define CMD_SNAKE_LATERAL  0x03    // 蛇形侧向运动
#define CMD_SNAKE_CIRCULAR 0x04    // 蛇形圆周运动

// const char* ssid = "YOUR_WIFI_SSID";
// const char* password = "YOUR_WIFI_PASSWORD";
// const char* serverIP = "192.168.3.246";
// const char* ssid = "YOUR_WIFI_SSID";
// const char* password = "YOUR_WIFI_PASSWORD";
// const char* serverIP = "192.168.66.171";
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";
const char* serverIP = "192.168.110.16";
const uint16_t port = 12340;

TaskHandle_t tcpTaskHandle = NULL;
TaskHandle_t serial2TaskHandle = NULL;
TaskHandle_t cmdTaskHandle = NULL;

WiFiClient client;

unsigned long lastStatTime = 0;
const unsigned long STAT_INTERVAL = 1000;
unsigned long serialRxBytes = 0;
unsigned long tcpTxBytes = 0;
unsigned long droppedBytes = 0;
unsigned long lastSerialDataMs = 0;
bool warnedNoSerialData = false;

struct ControlCommand {
    uint8_t head1;
    uint8_t head2;
    uint8_t mode;
    uint8_t parameter;
    uint8_t tail1;
    uint8_t tail2;
} __attribute__((packed));

// 添加全局状态标志
volatile bool commandInProgress = false;
volatile bool needResetSerial = false;

void handleCmdTask(void* parameter) {
    ControlCommand cmd;
    uint8_t cmdBuffer[sizeof(ControlCommand)] = {0};
    uint8_t cmdIndex = 0;
    const uint32_t CMD_TIMEOUT = 1000;
    uint32_t lastCmdTime = 0;

    while (true) {
        if (client.connected()) {
            int available = client.available();
            if (available > 0) {
                uint8_t c = client.read();
                cmdBuffer[cmdIndex++] = c;

                if (cmdIndex == sizeof(ControlCommand)) {
                    memcpy(&cmd, cmdBuffer, sizeof(ControlCommand));

                    if (cmd.head1 == FRAME_HEAD1 &&
                        cmd.head2 == FRAME_HEAD2 &&
                        cmd.tail1 == FRAME_TAIL1 &&
                        cmd.tail2 == FRAME_TAIL2) {

                        // 设置命令处理标志
                        commandInProgress = true;
                        needResetSerial = true;

                        // 等待Serial2Task完成当前数据包的处理
                        vTaskDelay(20 / portTICK_PERIOD_MS);

                        // 发送命令到STM32
                        Serial2.write(cmdBuffer, sizeof(ControlCommand));
                        Serial2.flush();

                        // 等待STM32响应
                        vTaskDelay(30 / portTICK_PERIOD_MS);

                        // 发送确认
                        if (client.connected()) {
                            uint8_t ack = 0xAA;
                            client.write(&ack, 1);
                        }

                        // 重置标志
                        commandInProgress = false;
                    }
                    cmdIndex = 0;
                }

                if (cmdIndex >= sizeof(ControlCommand)) {
                    cmdIndex = 0;
                }
            }
        }
        vTaskDelay(5 / portTICK_PERIOD_MS);
    }
}

void handleSerial2Task(void* parameter) {
    static uint8_t tcpBuffer[1024];

    while (true) {
        if (needResetSerial) {
            while(Serial2.available()) {
                Serial2.read();
            }
            needResetSerial = false;
            warnedNoSerialData = false;
            Serial.println("Serial2 buffer cleared for command handling");
            vTaskDelay(10 / portTICK_PERIOD_MS);
            continue;
        }

        if (commandInProgress) {
            vTaskDelay(5 / portTICK_PERIOD_MS);
            continue;
        }

        uint32_t currentTime = millis();
        if (Serial2.available()) {
            size_t bytesToRead = min(Serial2.available(), static_cast<int>(sizeof(tcpBuffer)));
            if (bytesToRead > 0) {
                size_t bytesRead = Serial2.readBytes(reinterpret_cast<char*>(tcpBuffer), bytesToRead);
                if (bytesRead > 0) {
                    serialRxBytes += bytesRead;
                    lastSerialDataMs = currentTime;
                    warnedNoSerialData = false;

                    if (client.connected()) {
                        size_t written = client.write(tcpBuffer, bytesRead);
                        tcpTxBytes += written;
                        if (written != bytesRead) {
                            droppedBytes += (bytesRead - written);
                            Serial.printf("Warning: TCP wrote %u/%u bytes\n",
                                          static_cast<unsigned int>(written),
                                          static_cast<unsigned int>(bytesRead));
                        }
                    } else {
                        droppedBytes += bytesRead;
                    }
                }
            }
        }

        if (!warnedNoSerialData && currentTime > 5000 && (currentTime - lastSerialDataMs > 3000)) {
            warnedNoSerialData = true;
            Serial.println("Warning: no Serial2 IMU data received in the last 3 seconds");
        }

        if (currentTime - lastStatTime >= STAT_INTERVAL) {
            lastStatTime = currentTime;
            Serial.printf("[STAT] wifi=%s tcp=%s serial_rx=%lu tcp_tx=%lu dropped=%lu Serial2Avail=%d\n",
                          WiFi.status() == WL_CONNECTED ? "up" : "down",
                          client.connected() ? "up" : "down",
                          serialRxBytes,
                          tcpTxBytes,
                          droppedBytes,
                          Serial2.available());
        }
        vTaskDelay(1);
    }
}

void handleTCPTask(void* parameter) {
    const int MAX_RECONNECT_ATTEMPTS = 5;
    int reconnectAttempts = 0;

    while (true) {
        if (!client.connected()) {
            Serial.println("\n===== TCP连接状态 =====");
            Serial.printf("当前WiFi状态: %s\n", WiFi.status() == WL_CONNECTED ? "已连接" : "未连接");

            if (WiFi.status() != WL_CONNECTED) {
                Serial.println("重新连接WiFi...");
                WiFi.reconnect();
                vTaskDelay(5000 / portTICK_PERIOD_MS);
                continue;
            }

            Serial.printf("尝试连接服务器 (第 %d 次)...\n", reconnectAttempts + 1);

            if (client.connect(serverIP, port)) {
                client.setNoDelay(true);
                client.setTimeout(1000);
                Serial.println("服务器连接成功！");
                reconnectAttempts = 0;
            } else {
                reconnectAttempts++;
                if (reconnectAttempts >= MAX_RECONNECT_ATTEMPTS) {
                    Serial.println("达到最大重连次数，重启ESP32...");
                    ESP.restart();
                }
                Serial.println("连接失败，5秒后重试");
                vTaskDelay(5000 / portTICK_PERIOD_MS);
            }
        } else {
            if (!client.connected()) {
                Serial.println("检测到连接断开");
                client.stop();
                continue;
            }
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void setup() {
    Serial.begin(BAUD_RATE);
    Serial2.setRxBufferSize(32768);
    Serial2.setTxBufferSize(2048);
    Serial2.begin(BAUD_RATE, SERIAL_8N1, 16, 17);

    WiFi.setSleep(false);
    WiFi.setAutoReconnect(true);  // 启用自动重连
    WiFi.persistent(true);        // 保存WiFi配置

    WiFi.begin(ssid, password);
    Serial.print("连接WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi已连接");
    Serial.printf("ESP32 IP地址: %s\n", WiFi.localIP().toString().c_str());

    lastStatTime = millis();

    // 命令处理任务 - 核心0，高优先级
    xTaskCreatePinnedToCore(
        handleCmdTask,
        "CmdTask",
        8192,
        nullptr,
        3,
        &cmdTaskHandle,
        0
    );

    // IMU数据处理任务 - 核心1，中等优先级
    xTaskCreatePinnedToCore(
        handleSerial2Task,
        "Serial2Task",
        8192,
        nullptr,
        2,
        &serial2TaskHandle,
        1
    );

    // TCP连接管理任务 - 核心0，低优先级
    xTaskCreatePinnedToCore(
        handleTCPTask,
        "TCPTask",
        8192,
        nullptr,
        1,
        &tcpTaskHandle,
        0
    );
}

void loop() {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}
