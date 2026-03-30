SHELL := /usr/bin/bash

CMAKE := /usr/bin/cmake
RM := /usr/bin/rm

GEN ?= Ninja
empty :=
space := $(empty) $(empty)
GEN_DIR := $(subst $(space),_,$(GEN))
BUILD_ROOT ?= build
BUILD_DIR ?= $(BUILD_ROOT)/$(GEN_DIR)
LOCAL_IDF_PATH := $(CURDIR)/third_party/esp-idf

CLEAN_DIRS := \
	$(BUILD_DIR)

CLEAN_FILES := \
	config/32raven.config.old \
	config/defconfig \
	esp32/main/esp32_config.hpp \
	stm32/Drivers/USER/inc/stm32_config.hpp \
	stm32/Drivers/USER/inc/stm32_limits.hpp \
	stm32/Drivers/USER/inc/ee_schema.hpp

.PHONY: help configure all stm32 esp32 clean distclean flash-esp32 monitor-esp32 idf-install 32raven-menuconfig format-cpp

help:
	@echo "Targets:"
	@echo "  configure           - Configure CMake"
	@echo "  all                 - Build all firmware"
	@echo "  32raven-menuconfig  - Run 32Raven menuconfig (ESP32 + STM32)"
	@echo "  esp32               - Build ESP32 firmware"
	@echo "  stm32               - Build STM32 firmware"
	@echo "  format-cpp          - Run clang-format on STM32/ESP32 C++ source headers"
	@echo "  clean               - Clean build directory"
	@echo "  esp32-menuconfig    - Run ESP-IDF menuconfig"
	@echo "  flash-esp32         - Flash ESP32 via serial"
	@echo "  monitor-esp32       - Monitor ESP32 via serial"
	@echo "  flash-monitor-esp32 - Flash and monitor ESP32 via serial"
	@echo "  flash-wifi-esp32    - Flash ESP32 via WiFi (OTA)"
	@echo "  flash-wifi-stm32    - Flash STM32 via WiFi (Bridge)"
	@echo "  idf-install         - Install local ESP-IDF tools for third_party/esp-idf"
	@echo "Vars: GEN='Ninja' or 'Unix Makefiles', BUILD_ROOT=build, BUILD_DIR=build/<generator>, IDF_PATH=..., STM32_TOOLCHAIN_FILE=..."

configure:
	@mkdir -p "$(BUILD_DIR)"
	"$(CMAKE)" -S . -B "$(BUILD_DIR)" -G "$(GEN)"

all: configure
	"$(CMAKE)" --build "$(BUILD_DIR)" --target all_firmware

esp32: configure
	"$(CMAKE)" --build "$(BUILD_DIR)" --target esp32
	@printf "\n"
	python3 tools/esp32_size_metrics.py --build-dir "$(BUILD_DIR)/esp32"

stm32: configure
	"$(CMAKE)" --build "$(BUILD_DIR)" --target stm32

format-cpp:
	@bash -lc 'set -euo pipefail; \
		command -v clang-format >/dev/null; \
		rg --files esp32 stm32 -g "*.cpp" -g "*.hpp" \
		  | grep -vxF "esp32/main/esp32_config.hpp" \
		  | grep -vxF "stm32/Drivers/USER/inc/stm32_config.hpp" \
		  | grep -vxF "stm32/Drivers/USER/inc/stm32_limits.hpp" \
		  | grep -vxF "stm32/Drivers/USER/inc/ee_schema.hpp" \
		  | while IFS= read -r file; do \
		      clang-format -i "$$file"; \
		    done'

32raven-menuconfig:
	python3 scripts/32raven_menuconfig.py --kconfig config/Kconfig --config config/32raven.config

IDF_PATH_VAR := $(shell grep "IDF_PATH" user_config.cmake 2>/dev/null | grep -v "^#" | cut -d'"' -f2)
export IDF_PATH ?= $(if $(IDF_PATH_VAR),$(IDF_PATH_VAR),$(LOCAL_IDF_PATH))

# Helper to extract ESP_PORT and ESP_BAUD
ESP_PORT_VAR := $(shell grep "ESP_PORT" user_config.cmake 2>/dev/null | grep -v "^#" | cut -d'"' -f2)
ESP_BAUD_VAR := $(shell grep "ESP_BAUD" user_config.cmake 2>/dev/null | grep -v "^#" | cut -d'"' -f2)

IDF_ARGS :=
ifneq ($(ESP_PORT_VAR),)
	IDF_ARGS += -p $(ESP_PORT_VAR)
endif
ifneq ($(ESP_BAUD_VAR),)
	IDF_ARGS += -b $(ESP_BAUD_VAR)
endif

idf-install:
	bash -lc "cd \"$(IDF_PATH)\" && ./install.sh"

esp32-menuconfig:
	bash -lc ". \"$(IDF_PATH)/export.sh\" && cd esp32 && idf.py $(IDF_ARGS) -B ../$(BUILD_DIR)/esp32 menuconfig"
flash-esp32: esp32
	bash -lc ". \"$(IDF_PATH)/export.sh\" && cd esp32 && idf.py $(IDF_ARGS) -B ../$(BUILD_DIR)/esp32 flash"

monitor-esp32:
	bash -lc ". \"$(IDF_PATH)/export.sh\" && cd esp32 && idf.py $(IDF_ARGS) -B ../$(BUILD_DIR)/esp32 monitor"

flash-monitor-esp32: esp32
	bash -lc ". \"$(IDF_PATH)/export.sh\" && cd esp32 && idf.py $(IDF_ARGS) -B ../$(BUILD_DIR)/esp32 flash monitor"

clean:
	@echo "Cleaning ESP-IDF..."
	-bash -lc ". \"$(IDF_PATH)/export.sh\" && cd esp32 && idf.py $(IDF_ARGS) -B ../$(BUILD_DIR)/esp32 fullclean"
	@echo "Removing clean directories"
	"$(RM)" -rf $(CLEAN_DIRS)
	@echo "Removing clean files"
	"$(RM)" -f $(CLEAN_FILES)

# Default ESP32 IP
ESP_IP ?= 192.168.4.1

# WiFi Flashing
flash-wifi-stm32: stm32
	python3 tools/esp32_client.py $(ESP_IP) flash $(BUILD_DIR)/stm32/32Raven_stm32.bin

flash-wifi-esp32: esp32
	-pkill -f esp32_client.py || true
	python3 tools/esp32_client.py $(ESP_IP) flash_esp $(BUILD_DIR)/esp32/32Raven_esp32.bin

distclean: clean
	@echo "Removing all generator build directories in $(BUILD_ROOT)/"
	"$(RM)" -rf "$(BUILD_ROOT)"
