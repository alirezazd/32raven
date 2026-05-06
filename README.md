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

The build can run in two modes — pick one. The same `make` targets work in either.

### Mode A — Docker (recommended, zero host pollution)

You only need a working Docker engine. Everything else (ARM GCC, CMake, Ninja, Python deps, ESP-IDF host prerequisites) lives inside the build image.

* **Docker Engine** — install per the [official docs](https://docs.docker.com/engine/install/), then add your user to the `docker` group and re-login.

VSCode users with the **Dev Containers** extension can open the repo directly in a container — `.devcontainer/devcontainer.json` reuses the same `Dockerfile` and pre-installs the C/C++, CMake, clangd, and Cortex-Debug extensions.

> **Optional: VSCode performance with nested submodules.** If the Source Control panel feels slow scanning ESP-IDF's submodules, run `make setup-vscode` to apply recommended settings (`git.detectSubmodules: false`, file watcher exclusions, etc.). This is optional; the settings are **not committed** — you control what applies to your workspace.

### Mode B — Host install

* **CMake** (3.22+)
* **ARM GCC toolchain** (`arm-none-eabi-*`) for STM32
* **Python 3** with `kconfiglib` and `jinja2`
* **ripgrep** + **clang-format** (for `make format-cpp`)
* ESP-IDF host prereqs ([list](https://docs.espressif.com/projects/esp-idf/en/v5.5.3/esp32/get-started/linux-macos-setup.html#install-prerequisites)) for ESP32 builds

> ESP-IDF itself is pinned as a submodule at `third_party/esp-idf` (v5.5.3). The toolchain it downloads goes to `~/.espressif` (host mode) or `.docker/home/.espressif` (Docker mode, gitignored).

---

## Quick Start

### First-time setup (clone + submodules)

```bash
git submodule update --init --recursive
```

### Pick a build mode

```bash
make enable-docker     # builds run in container; persists in .build-mode
# — or —
make disable-docker    # builds run on host (default)
```

The toggle is sticky across shells; flip it any time. One-shot override: `USE_DOCKER=0 make stm32`.

### Install ESP-IDF tools (once, only needed for ESP32 builds)

```bash
make idf-install
```

In Docker mode this populates `.docker/home/.espressif`; in host mode, `~/.espressif`. Persisted across builds.

### Configure

```bash
make 32raven-menuconfig
```

Opens the 32Raven config tree for ESP32 + STM32 and updates `config/32raven.config`.

### Build

```bash
make all       # both targets
make stm32     # STM32 only
make esp32     # ESP32 only
```

### Targets

| Target             | Command                   |
| ------------------ | ------------------------- |
| **32Raven Config** | `make 32raven-menuconfig` |
| **STM32 Only**     | `make stm32`              |
| **ESP32 Only**     | `make esp32`              |
| **Complete Stack** | `make all`                |
| **Enable Docker**  | `make enable-docker`      |
| **Disable Docker** | `make disable-docker`     |
| **Rebuild image**  | `make docker-image`       |
| **Cleanup**        | `make clean`              |

---

## Flashing & Deployment

### ESP32 (Serial)

```bash
make flash-esp32
```

> In Docker mode, set `ESP_PORT="/dev/ttyUSB0"` (or your device path) in `user_config.cmake` so the USB tty is passed into the container via `--device`. Without it, idf.py auto-detect won't see any host devices.

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
