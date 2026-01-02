# 32raven

Drone flight controller project for STM32 and ESP32 platforms.

## Project Structure

- `libs`: Code shared between both platforms (drivers, math, protocols).
- `stm32`: STM32 specific code (HAL, startup, main loop).
- `esp32`: ESP32 specific code (tasks, wireless communication).

## Build Instructions

### Method 1: Root Builder (Recommended)
This method orchestrates both builds from the root without merging toolchains.

```bash
mkdir build && cd build
cmake ..
cmake --build . --target build-all
```

### Method 2: Individual Directories

**STM32**
```bash
cd stm32
mkdir build && cd build
cmake ..
make
```

**ESP32**
```bash
cd esp32
idf.py build
```
