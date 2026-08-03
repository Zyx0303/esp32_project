#pragma once

#include "esp_err.h"

/** 初始化连接 CH340/USB 串口桥的 UART0。 */
esp_err_t app_console_init(void);

/** 进入串口命令读取循环；此函数正常情况下不会返回。 */
void app_console_run(void);
