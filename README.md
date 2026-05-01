# 32Raven
![32Raven project image](./32Raven.png)
**Status:** Active development. Interfaces, configs, and behavior can change quickly.

[![Firmware Version](https://img.shields.io/endpoint?url=https%3A%2F%2Fraw.githubusercontent.com%2Falirezazd%2F32raven%2Fbadge-data%2Ffirmware-version-badge.json)](https://github.com/alirezazd/32raven/tags)

**Firmware Version:** v0.0.x Stable

## Overview

**32Raven** is a high-performance, bare-metal flight control ecosystem designed from the ground up.

Unlike standard hobbyist stacks, 32Raven utilizes a **decoupled dual-target architecture** to ensure maximum reliability:

* **`stm32/` (The Brain):** Real-time, deterministic flight-control logic (sensors, PID loops, low-level drivers) running on STM32F407.
* **`esp32/` (The Bridge):** High-level communication, telemetry (MAVLink/CRSF), and wireless integration (WiFi/OTA).
* **`libs/`:** Shared core logic and math utilities owned by this repo.
* **`third_party/`:** Vendored external dependencies such as ESP-IDF and MAVLink.

## Vision & Design Direction

The project is built on a **"Bare-Metal First"** philosophy. Our goal is to eliminate the unpredictability of heavy abstractions to achieve industrial-grade timing and control.

* **Deterministic Performance:** We prioritize minimal abstraction overhead for predictable runtime behavior.
* **HAL Strategy:** HAL is currently used for initial bring-up (clocks, peripherals). Runtime data paths are being systematically migrated to **direct register-level control**.
* **Hardware Stack:** Optimized custom drivers for the `ICM42688P` IMU and `u-blox M9N` GPS.
* **Wireless First:** Built-in support for flashing the STM32 "over the air" via the ESP32 communication bridge.

## Repository Layout

* **`stm32/`** — STM32 firmware, drivers, core flight state machine.
* **`esp32/`** — ESP-IDF firmware, MAVLink/CRSF bridge, and failsafe services.
* **`libs/`** — Shared headers and portable source code owned by this repo.
* **`third_party/`** — External dependencies pinned as submodules.
* **`tools/`** — Helper scripts for flashing, bridging, and telemetry dashboards.
* **`scripts/`** — Linting and project automation.

## Build Prerequisites

* **CMake** (3.22+)
* **ARM GCC toolchain** (`arm-none-eabi-*`) for STM32
* **ESP-IDF tools** for the ESP32 target
* **Python 3** for helper tools

> **Note:** The repo pins ESP-IDF as a submodule in `third_party/esp-idf` at `v5.5.3`.
> Run `git submodule update --init --recursive` after cloning.
> Run `make idf-install` before the first ESP32 build.
> Optional local overrides (e.g. serial port, baud, alternate `IDF_PATH`) can still be set in `user_config.cmake`.

---

## Quick Start

### First-Time Setup

```bash
git submodule update --init --recursive
make idf-install
```

### Configuration

```bash
make 32raven-menuconfig
```

This opens the 32Raven config tree for both `ESP32` and `STM32` and updates `config/32raven.config`.
Current generated consumers are on the STM32 side; the ESP32 section is part of the same project-wide menu tree.

### Build

```bash
make all
```

### Target-Specific Build

```bash
make stm32
make esp32
```

### Build Targets

| Target             | Command      |
| ------------------ | ------------ |
| **32Raven Config** | `make 32raven-menuconfig` |
| **STM32 Only**     | `make stm32` |
| **ESP32 Only**     | `make esp32` |
| **Complete Stack** | `make all`   |
| **Cleanup**        | `make clean` |

---

## Flashing & Deployment

### ESP32 (Serial)

```bash
make flash-esp32

```

### ESP32 (Wireless/OTA)

```bash
make flash-wifi-esp32 ESP_IP=192.168.4.1

```

### STM32 (Wireless via Bridge)

*This flashes the STM32 via the ESP32's WiFi connection:*

```bash
make flash-wifi-stm32 ESP_IP=192.168.4.1

```

---

## Technical Notes

* **Generator:** The top-level build uses `GEN` (default `Ninja`). To change: `make configure GEN="Unix Makefiles"`.
* **Output:** Build outputs are generator-specific, for example `build/Ninja/stm32/` and `build/Ninja/esp32/`.
* **Toolchains:** If switching generators or toolchains, always run `make clean` first.
---
