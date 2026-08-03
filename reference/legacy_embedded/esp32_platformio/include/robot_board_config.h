#pragma once

#ifndef ROBOT_BOARD_INDEX
#error "ROBOT_BOARD_INDEX must be 1, 2, or 3"
#endif

#define ROBOT_SERVER_IP "172.20.10.3"

#if ROBOT_BOARD_INDEX == 1
#define ROBOT_WIFI_AP_MODE 0
#define ROBOT_TCP_PORT 12340
#elif ROBOT_BOARD_INDEX == 2
#define ROBOT_WIFI_AP_MODE 0
#define ROBOT_TCP_PORT 12341
#elif ROBOT_BOARD_INDEX == 3
#define ROBOT_WIFI_AP_MODE 0
#define ROBOT_TCP_PORT 12342
#else
#error "Unsupported ROBOT_BOARD_INDEX"
#endif
