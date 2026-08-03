# Robot Control HTTP API v1

## Transport

- Preferred transport: ESP32 STA and the client join the same LAN; read the DHCP IP from UART `status` or the startup log.
- Rescue SoftAP: `ESP32-Robot`; fallback address `http://192.168.4.1`.
- All state-changing v1 operations use `POST` with `Content-Type: application/json`.
- All commands are validated, timestamped, assigned a sequence number and placed in the control queue. HTTP handlers never drive hardware directly.
- Non-zero motor commands must be refreshed before the configured command timeout (default 1000 ms).

## State and safety

The state sequence is `BOOT -> SAFE -> READY -> RUNNING`. `FAULT` and `ESTOP` are latched safety states.

- The device starts in `SAFE`; actuators cannot move before `ARM`.
- `ESTOP` and `DISARM` have queue priority.
- `ESTOP` can be submitted regardless of command age or normal sequence ordering.
- `fault/clear` returns the state to `SAFE` only after active hardware faults are gone.
- Clearing a fault does not ARM the robot.

## Common response

Accepted command:

```json
{"ok":true,"accepted":true,"sequence":42,"state":"READY","error":null}
```

Rejected request:

```json
{"ok":false,"sequence":null,"state":"SAFE","error":"drive_range"}
```

An accepted response means the command entered the control queue. The client reads `/api/v1/status` to observe the resulting state. Queue failure returns HTTP 503.

## Endpoints

### Status and device

- `GET /api/v1/status`
- `GET /api/v1/device`

Status includes state, ARM/ESTOP/fault flags, target and ramped motor outputs, target and ramped servo angle, GPIO35 state, DRV8833 fault state, IMU health/raw values, uptime, heap, STA connection/IP/reconnects, SoftAP clients and diagnostic counters.

The `network` object also reports `last_disconnect_reason` and `reconnect_delay_ms`.
STA retries use a bounded 2/4/8/16/30 second backoff so an unavailable router does not continuously
consume radio scan time needed by the rescue SoftAP.

### Safety commands

- `POST /api/v1/arm`
- `POST /api/v1/disarm`
- `POST /api/v1/estop`
- `POST /api/v1/fault/clear`

No JSON body is required.

### Differential drive

`POST /api/v1/drive`

```json
{"throttle":60,"steering":-20}
```

Both values use `-100..100`. Mixing is:

```text
motor_a = clamp(throttle + steering)
motor_b = clamp(throttle - steering)
```

### Direct motor board test

`POST /api/v1/motors`

```json
{"motor_a":20,"motor_b":-20}
```

Both values use `-100..100`. This endpoint is intended for controlled board testing and is still subject to ARM, ramp, reversal deadtime, watchdog and nFAULT protection.

### Servo

`POST /api/v1/servo`

```json
{"angle":90}
```

The first command explicitly enables servo PWM. The target is limited by validated configuration and approached using the configured slew rate. DISARM/ESTOP/FAULT disables PWM instead of automatically returning to center.

### GPIO35 external active-low interface

`POST /api/v1/power`

```json
{"asserted":true}
```

`true` drives GPIO35 high and turns on the 2N7002, pulling the external interface low. It is explicit and is not coupled to motor ARM.

## Compatibility endpoints

The following unversioned GET endpoints remain temporarily available for the existing GUI:

- `/api/status`
- `/api/motor?speed=<value>`
- `/api/servo?angle=<value>`
- `/api/power?on=<0|1>`
- `/api/stop`

They are adapters into the same command queue. They do not bypass ARM or safety checks. New clients must use `/api/v1`.

The old six-byte `FF FA mode parameter 88 77` protocol is not enabled in v1 because the imported programs assign conflicting meanings to modes `0x03` and `0x04`. It can only be added after a separately versioned compatibility mapping and tests are approved.
