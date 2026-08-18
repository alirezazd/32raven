# The firmware

One aircraft, two firmwares. The flight-critical loop and the connectivity surface live on
different MCUs and share only a versioned wire protocol — a decoupled dual-target
architecture, bare-metal on both sides.

- **`stm32/` — the flight computer.** The real-time core: IMU, GPS and RC drivers, AHRS,
  attitude and rate control, the mixer. Runs on the STM32F407.
- **`esp32/` — the bridge.** The connectivity surface: MAVLink and CRSF telemetry, WiFi,
  over-the-air updates, and the on-device UI. Runs on the ESP32-C3.
- **`libs/` — the contract.** Wire-protocol structs and error codes, compiled into both
  targets, so the two sides cannot quietly drift apart.

## Design principles

- **Deterministic by default.** No exceptions, no RTTI, no dynamic allocation — the firmware
  links with no heap at all, and a lint check rejects raw libc allocators. FreeRTOS tasks
  run on static stacks.
- **No HAL — register-level throughout.** Clock, GPIO and peripheral bring-up all run
  direct-register against CMSIS; the STM32 HAL is not in the source tree or the binary.
- **Custom drivers where it matters.** The ICM42688P IMU and u-blox M10 GPS have
  hand-written drivers tuned for the loop they live in.
- **Checked at build time.** Every pin assignment is validated against ST's silicon data
  during the build — a typo'd pin or impossible alternate function aborts it instead of
  shipping.
- **Wireless flashing.** STM32 updates travel over the air through the bridge — no debug
  probe, and no cable to the flight computer, ever.

## The repository

| Directory | What lives there |
|---|---|
| `stm32/` | STM32 firmware — drivers, services, the core flight state machine |
| `esp32/` | ESP-IDF firmware — drivers, services, the on-device UI |
| `libs/` | Shared headers and portable source compiled into both targets |
| `config/` | `Kconfig`, the checked-in reference config, EEPROM schema input |
| `scripts/` | Code generators (Kconfig → headers), pin-map validator, lint hooks |
| `tools/` | Host-side helpers for flashing, bridging and telemetry |
| `third_party/` | Pinned submodules — ESP-IDF, MAVLink, Eigen, nanoprintf, ST open-pin-data, Adafruit GFX |

## In this section

Three pages take it from a fresh clone to firmware on the aircraft:

1. **[Development setup](setup.md)** — clone, pick a build mode, install the ESP-IDF tools
   once.
2. **[Configure and build](building.md)** — one menuconfig tree for both MCUs, and the
   build targets.
3. **[Flashing the firmware](flashing.md)** — the bridge over USB, everything else over the
   air.
