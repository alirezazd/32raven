SHELL := /usr/bin/bash

CMAKE := /usr/bin/cmake
RM := /usr/bin/rm

BUILD_DIR ?= build
GEN ?= Ninja

.PHONY: help configure all stm32 esp32 clean distclean flash-esp32 monitor-esp32

help:
	@echo "Targets: configure | all | esp32 | stm32 | clean | flash-esp32 | monitor-esp32"
	@echo "Vars: GEN='Ninja' or 'Unix Makefiles', BUILD_DIR=build, IDF_PATH=..., STM32_TOOLCHAIN_FILE=..."

configure:
	@mkdir -p "$(BUILD_DIR)"
	"$(CMAKE)" -S . -B "$(BUILD_DIR)" -G "$(GEN)"

all: configure
	"$(CMAKE)" --build "$(BUILD_DIR)" --target all_firmware

esp32: configure
	"$(CMAKE)" --build "$(BUILD_DIR)" --target esp32

stm32: configure
	"$(CMAKE)" --build "$(BUILD_DIR)" --target stm32

flash-esp32: esp32
	cd esp32 && idf.py -B ../$(BUILD_DIR)/esp32 flash

monitor-esp32: 
	cd esp32 && idf.py -B ../$(BUILD_DIR)/esp32 monitor

flash-monitor-esp32: esp32
	cd esp32 && idf.py -B ../$(BUILD_DIR)/esp32 flash monitor

clean:
	@echo "Removing $(BUILD_DIR)/"
	"$(RM)" -rf "$(BUILD_DIR)"

distclean: clean
