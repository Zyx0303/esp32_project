#include "SDCardHandler.h"
#include <Arduino.h>

SDCardHandler::SDCardHandler() : initialized(false) {}

bool SDCardHandler::initialize() {
    initialized = SD_MMC.begin();
    if (!initialized) {
        Serial.println("Card Mount Failed");
    }
    return initialized;
}

void SDCardHandler::writeData(const char* data, int length) {
    if (!initialized) {
        Serial.println("SD card not initialized!");
        return;
    }

    File file = SD_MMC.open("/data.txt", FILE_APPEND);
    if (!file) {
        Serial.println("File open failed!");
    } else {
        file.write((const uint8_t*)data, length);
        file.close();
    }
}
