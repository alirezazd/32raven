# Build the prototype

The from-scratch path: a pile of parts to an aircraft that arms. Stages run in order —
each one assumes the previous passed. Every stage before *Bench test* runs with no
propellers fitted and no flight battery connected; the boards are powered from USB.

## Running order

| # | Stage | What it covers | Status |
|---|-------|----------------|--------|
| 1 | **[Bill of materials](bom.md)** | Parts, vendors, cost, tools, acceptable substitutions | *in progress* |
| 2 | **[The two boards](boards.md)** | Specs, pinouts, what is already wired, mounting the stack | *in progress* |
| 3 | Frame and motors | Frame assembly, motor mounting, ESC placement | *not written* |
| 4 | **[Wiring](wiring.md)** | Every signal between the two MCUs and the peripherals | **written** |
| 5 | Power | Battery, BEC rails, ESC flashing and AM32 settings | *not written* |
| 6 | Toolchain | Docker or host build environment | *not written* |
| 7 | Configure and flash | `make 32raven-menuconfig`, wired flash, OTA | *not written* |
| 8 | Smoke test | First power-on, error codes, link handshake | *not written* |
| 9 | Sensors | IMU orientation, GPS lock, battery calibration | *not written* |
| 10 | RC link | CRSF binding, channel map, failsafe | *not written* |
| 11 | Bench test | Motor order and direction, **props still off** | *not written* |
| 12 | First flight | Arming, hover, initial tuning | *not written* |

Stages 6 and 7 currently live in the
[repository README](https://github.com/alirezazd/32raven#build-prerequisites) and will move
here as they are rewritten for someone who has never built the firmware before.

## What you are building

A quadcopter carrying two boards that split the work:

- **STM32F407** — the flight controller. IMU, GPS, RC input, AHRS, attitude and rate control,
  DShot mixer. Nothing on this MCU waits on WiFi.
- **ESP32-C3** — the bridge. MAVLink and CRSF telemetry, WiFi, over-the-air firmware updates
  for *both* MCUs, and the on-device OLED UI.

The two are joined by a single UART running a versioned wire protocol (**FcLink**). That link
is the first thing to get right, and it is the first table in
[the wiring reference](wiring.md).

## Reference build

Where a choice exists, this guide documents one specific build — the pin assignments checked
into `config/32raven.config`. That file is the reference build's source of truth, not a
suggestion: every pin quoted in these pages is the value it holds. Most pins are Kconfig
choices, so a different board layout is a `make 32raven-menuconfig` change rather than a
firmware edit.
