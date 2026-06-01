# 32Raven

![32Raven project image](./32Raven.png)

**Status:** Active development. Interfaces, configs, and behavior can change quickly.

[![Firmware Version](https://img.shields.io/endpoint?url=https%3A%2F%2Fraw.githubusercontent.com%2Falirezazd%2F32raven%2Fbadge-data%2Ffirmware-version-badge.json)](https://github.com/alirezazd/32raven/tags)

## Overview

32Raven is a bare-metal flight-control stack built around a **decoupled dual-target architecture**. The flight-critical loop and the connectivity surface live on different MCUs and share only a versioned wire protocol.

- **`stm32/` — flight controller.** Real-time control loop, IMU/GPS/RC drivers, AHRS, attitude/rate control, mixer. Runs on STM32F407.
- **`esp32/` — connectivity bridge.** MAVLink/CRSF telemetry, WiFi, OTA, on-device UI. Runs on ESP32-C3.
- **`libs/` — shared contract.** Wire-protocol structs, error codes, dual-target utilities.
- **`third_party/` — pinned vendor code.** ESP-IDF, MAVLink, Eigen, nanoprintf, ST open-pin-data, Adafruit GFX.

## Design Principles

- **Deterministic by default.** No exceptions, no RTTI, no dynamic allocation — the firmware links with no heap at all, and raw libc allocators are rejected by a lint check. Static stacks for FreeRTOS tasks. Pin alignment validated against ST silicon data at build time.
- **HAL only at bring-up.** Clock and peripheral init use STM32 HAL; runtime data paths are direct-register where it matters.
- **Custom drivers where it matters.** ICM42688P IMU and u-blox M10 GPS have hand-written drivers tuned for the loop they live in.
- **Wireless flashing.** STM32 firmware updates flow over the air through the ESP32 bridge — no probe required for field updates.

## Repository Layout

- **`stm32/`** — STM32 firmware, drivers, core flight state machine.
- **`esp32/`** — ESP-IDF firmware, MAVLink/CRSF bridge, on-device UI.
- **`libs/`** — shared headers and portable source compiled into both targets.
- **`config/`** — `Kconfig`, `32raven.config`, EEPROM schema input.
- **`scripts/`** — code generators (Kconfig → headers via Jinja2), pin-map validator, lint hooks.
- **`tools/`** — host-side helpers for flashing, bridging, and telemetry.
- **`third_party/`** — pinned submodules: ESP-IDF, MAVLink, Eigen, nanoprintf, ST open-pin-data, Adafruit GFX.

### Agent Guides

Subdirectory-specific conventions live in `AGENTS.md` files alongside the code. Read the local guide before working in any of these areas:

- [`AGENTS.md`](./AGENTS.md) — repo-wide rules, build/verify workflow, error-code scheme, "adding a new tunable" workflow.
- [`stm32/AGENTS.md`](./stm32/AGENTS.md) — STM32 driver layout, system wiring, pin map (`board.hpp`).
- [`esp32/AGENTS.md`](./esp32/AGENTS.md) — ESP32 driver/service layout, ESP-IDF integration, FreeRTOS task patterns.
- [`libs/AGENTS.md`](./libs/AGENTS.md) — dual-target shared code (wire protocol, error codes).
- [`scripts/AGENTS.md`](./scripts/AGENTS.md) — generators, templates, lint hooks.
- [`config/AGENTS.md`](./config/AGENTS.md) — Kconfig + checked-in defaults sync rules.

## Build Prerequisites

The build can run in two modes — pick one. The same `make` targets work in either.

### Mode A — Docker (recommended, zero host pollution)

You only need a working Docker engine. Everything else (ARM GCC, CMake, Ninja, Python deps, ESP-IDF host prerequisites) lives inside the build image.

- **Docker Engine** — install per the [official docs](https://docs.docker.com/engine/install/), then add your user to the `docker` group and re-login.

VSCode users with the **Dev Containers** extension can open the repo directly in a container — `.devcontainer/devcontainer.json` reuses the same `Dockerfile` and pre-installs the CMake, clangd, and Cortex-Debug extensions.

> **Optional: VSCode workspace settings.** Run `make setup-vscode` to apply recommended settings: submodule scan disabled (Source Control perf with ESP-IDF), file watcher / search exclusions for vendored code, and `clangd.arguments` so clangd queries both the `arm-none-eabi` and `riscv32-esp-elf` GCC drivers for system headers (otherwise `<cstdint>` and friends won't resolve cross-target). This works in both Docker and host paths. Settings are **not committed** — you control what applies to your workspace.

### Mode B — Host install

- **CMake** (3.22+)
- **ARM GCC toolchain** (`arm-none-eabi-*`) for STM32
- **Python 3** with `kconfiglib` and `jinja2`
- **ripgrep** + **clang-format** (for `make format-cpp`)
- ESP-IDF host prereqs ([list](https://docs.espressif.com/projects/esp-idf/en/v5.5.3/esp32/get-started/linux-macos-setup.html#install-prerequisites)) for ESP32 builds

> ESP-IDF itself is pinned as a submodule at `third_party/esp-idf` (v5.5.3). The toolchain it downloads goes to `~/.espressif` (host mode) or `.docker/home/.espressif` (Docker mode, gitignored).

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

`make help` lists every target with a one-line description.

## Flashing & Deployment

### ESP32 (Serial)

```bash
make flash-esp32
```

> In Docker mode, set `ESP_PORT="/dev/ttyUSB0"` (or your device path) in `user_config.cmake` so the USB tty is passed into the container via `--device`. Without it, idf.py auto-detect won't see any host devices.

### ESP32 (Wireless / OTA)

```bash
make flash-wifi-esp32 ESP_IP=192.168.4.1
```

### STM32 (Wireless via the ESP32 bridge)

```bash
make flash-wifi-stm32 ESP_IP=192.168.4.1
```

The STM32 binary is uploaded over WiFi to the ESP32, which forwards it to the STM32 bootloader over the inter-MCU UART.

## Technical Notes

- **Generator:** the top-level build uses `GEN` (default `Ninja`). Override per-invocation: `make configure GEN="Unix Makefiles"`.
- **Output:** build outputs are generator-specific — `build/Ninja/stm32/`, `build/Ninja/esp32/`.
- **Toolchains:** when switching generators or toolchains, run `make clean` first to drop stale CMake cache.
- **Verifying a change:** `make stm32` and `make esp32` are independent; touch shared code under `libs/` and run both. The STM32 build also runs `scripts/lint/check_pinmap.py` against ST's silicon data — typo'd pin or AF aborts the build.
