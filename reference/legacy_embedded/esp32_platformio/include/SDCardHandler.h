#ifndef SDCARDHANDLER_H
#define SDCARDHANDLER_H

#include "FS.h"
#include "SD_MMC.h"

class SDCardHandler {
public:
    SDCardHandler();
    bool initialize();
    void writeData(const char* data, int length);

private:
    bool initialized;
};

#endif // SDCARDHANDLER_H
