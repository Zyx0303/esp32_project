# Flash Guide For 5 Segments

Each body segment needs **2 firmwares**:

1. `STM32` firmware
2. `ESP32` firmware

So for 5 segments, you will flash:

- `STM32` x 5
- `ESP32` x 5

The Python upper-computer program is **not** flashed to the boards.

## 1. STM32 firmware

Use this project for all 5 segments:

- `E:\Fudan\worms\codes\USER\worm.uvprojx`

### Where to change the segment id in STM32

Edit this file:

- `E:\Fudan\worms\codes\HARDWARE\usartx.h`

Change both macros to the same segment id:

```c
#define Segment_No x
#define SNAKE_NO x
```

Valid values are `0` to `4`.

## 2. ESP32 firmware

Use this project for all 5 segments:

- `E:\Fudan\worms\codesnew\robot_v0`

Main source file:

- `E:\Fudan\worms\codesnew\robot_v0\src\main.cpp`

### Where to change the segment mapping in ESP32

In `ESP32`, you do not change a direct "segment id" variable.
You change the TCP port:

```cpp
const uint16_t port = 12342;
```

Edit this file:

- `E:\Fudan\worms\codesnew\robot_v0\src\main.cpp`

Port mapping for 5 segments:

- segment `0` -> `12340`
- segment `1` -> `12341`
- segment `2` -> `12342`
- segment `3` -> `12343`
- segment `4` -> `12344`

This matches the upper-computer receiver setting in:

- `E:\Fudan\worms\codesnew\robot_v0\tcp_receiver.cpp`

## 3. Mapping table

| Segment | `Segment_No` | `SNAKE_NO` | `port` |
| --- | --- | --- | --- |
| 0 | 0 | 0 | 12340 |
| 1 | 1 | 1 | 12341 |
| 2 | 2 | 2 | 12342 |
| 3 | 3 | 3 | 12343 |
| 4 | 4 | 4 | 12344 |

## 4. Recommended flashing order

Flash one segment at a time.

### Segment 0

1. Edit `E:\Fudan\worms\codes\HARDWARE\usartx.h`
2. Set `Segment_No = 0`
3. Set `SNAKE_NO = 0`
4. Build and flash `E:\Fudan\worms\codes\USER\worm.uvprojx`
5. Edit `E:\Fudan\worms\codesnew\robot_v0\src\main.cpp`
6. Set `port = 12340`
7. Build and flash the `ESP32` firmware

### Segment 1

1. Set `Segment_No = 1`
2. Set `SNAKE_NO = 1`
3. Rebuild and flash `STM32`
4. Set `port = 12341`
5. Rebuild and flash `ESP32`

### Segment 2

1. Set `Segment_No = 2`
2. Set `SNAKE_NO = 2`
3. Rebuild and flash `STM32`
4. Set `port = 12342`
5. Rebuild and flash `ESP32`

### Segment 3

1. Set `Segment_No = 3`
2. Set `SNAKE_NO = 3`
3. Rebuild and flash `STM32`
4. Set `port = 12343`
5. Rebuild and flash `ESP32`

### Segment 4

1. Set `Segment_No = 4`
2. Set `SNAKE_NO = 4`
3. Rebuild and flash `STM32`
4. Set `port = 12344`
5. Rebuild and flash `ESP32`

## 5. Notes

### Current default values in the repo

Right now the repo is configured as segment `2`:

- `Segment_No = 2`
- `SNAKE_NO = 2`
- `port = 12342`

If you flash without changing anything, that board will behave as segment `2`.

### `serverIP`

There is also this line in `ESP32`:

```cpp
const char* serverIP = "192.168.66.171";
```

If your upper-computer PC still uses this IP, keep it unchanged for all 5 segments.
If the PC IP changes, update `serverIP` in all 5 `ESP32` builds.

### Do not mix old test files

For this 5-segment deployment, use only:

- `STM32`: `E:\Fudan\worms\codes\USER\worm.uvprojx`
- `ESP32`: `E:\Fudan\worms\codesnew\robot_v0`

Do not mix in old test files from other folders, or you may get inconsistent serial or port behavior.
