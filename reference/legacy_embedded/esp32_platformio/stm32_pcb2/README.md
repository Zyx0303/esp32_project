# PCB1 / servo ID 3 firmware

This is the guarded STM32F407ZG firmware for the second worm-robot PCB.

## Fixed hardware configuration

- ESP32 link: USART1, 921600 baud, existing PA9/PA10 and PB6/PB7 routing
- Servo bus: USART3, PB10/PB11, 115200 baud
- Servo ID: 3
- Mechanical center: 0 degrees after electronic-zero calibration
- Logical positive/left direction: decreasing servo angle
- Mechanical software limit: center +/- 45 degrees

The firmware sends no servo packet at startup.

## Six-byte control frames

All commands use `FF FA mode parameter 88 77`.

| Mode | Behavior |
| --- | --- |
| `00` | Stop and hold the current position |
| `01`, parameter `08` | Stop and hold, compatible with the existing PC stop button |
| `02` | Start 0.2 Hz snake motion; parameter 1..255 maps to 0..90 degrees and clamps at 45 degrees |
| `10` | Direct logical offset; parameter 0..255 maps to -45..+45 degrees |
| `11` | Move slowly to the calibrated center |

Snake mode continues until an explicit stop command is received.
