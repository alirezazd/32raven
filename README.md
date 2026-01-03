# 32raven

Flight controller project targeting **STM32** and **ESP32** platforms.

The STM32 firmware implements the main flight controller logic, while the ESP32 firmware provides Wi-Fi based programming and auxiliary services. Both targets are built independently but share common libraries.

---

## Repository Structure

```
.
├── esp32/      # ESP32 firmware (ESP-IDF based)
├── stm32/      # STM32 firmware (main flight controller)
├── libs/       # Shared libraries (platform-agnostic)
├── scripts/    # Helper and maintenance scripts
└── build/      # Out-of-tree build directory
```

---

## Build Instructions

### Root Build (Recommended)

To build both targets from the repository root:

```bash
mkdir -p build
cmake -S . -B build -G Ninja
cmake --build build --target all_firmware
```

You can also build targets individually:

```bash
cmake --build build --target stm32
cmake --build build --target esp32
```

> **Note:** The ESP32 build requires a valid ESP-IDF installation, while the STM32 build does not.

### Platform-Specific Builds

You can also build each project independently from their respective directories.

**STM32:**
```bash
cd stm32
mkdir -p build && cd build
cmake .. && cmake --build .
```

**ESP32:**
```bash
cd esp32
idf.py build
```

---

## Development & Tooling

This project uses `clangd` for code analysis. Each platform has its own configuration (`.clangd`) pointing to the appropriate build artifacts.

* **ESP-IDF Detection**: Auto-detected via `IDF_PATH` or standard locations (`~/esp/*/esp-idf`).
* **Header Rules**: `.hpp` files are C++ only, while `.h` files are C compatible.

---

## Contributing

* Please do not commit generated build artifacts or configuration files (like `compile_commands.json`).
* Ensure shared code in `libs/` remains platform-agnostic.

