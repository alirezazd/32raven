SHELL := /usr/bin/bash

CMAKE := cmake
RM := rm

GEN ?= Ninja
empty :=
space := $(empty) $(empty)
GEN_DIR := $(subst $(space),_,$(GEN))
BUILD_ROOT ?= build
BUILD_DIR ?= $(BUILD_ROOT)/$(GEN_DIR)

# user_config.cmake is gitignored; values surface as overrides for IDF_PATH / ESP_PORT / ESP_BAUD.
IDF_PATH_VAR := $(shell grep "IDF_PATH" user_config.cmake 2>/dev/null | grep -v "^#" | cut -d'"' -f2)
ESP_PORT_VAR := $(shell grep "ESP_PORT" user_config.cmake 2>/dev/null | grep -v "^#" | cut -d'"' -f2)
ESP_BAUD_VAR := $(shell grep "ESP_BAUD" user_config.cmake 2>/dev/null | grep -v "^#" | cut -d'"' -f2)

# Persistent build-mode toggle. `make enable-docker` writes USE_DOCKER=1 here;
# `make disable-docker` removes it. Override per-invocation with `USE_DOCKER=0 make ...`.
-include .build-mode

# If we're already inside a container (e.g. the VSCode devcontainer), force
# host-side execution so we don't try to run docker-in-docker.
ifneq ($(wildcard /.dockerenv),)
  USE_DOCKER := 0
endif

# Container runtime: defaults to docker; auto-falls back to podman if docker
# is missing. Override with `CONTAINER_RUNTIME=podman` etc.
CONTAINER_RUNTIME ?= $(shell command -v docker >/dev/null && echo docker || (command -v podman >/dev/null && echo podman || echo docker))
DOCKER_IMAGE ?= 32raven-build:latest
ifeq ($(USE_DOCKER),1)
  DOCKER_TTY := $(shell test -t 0 && echo -it || echo -i)
  # Rootless podman remaps UIDs via user namespaces; --userns=keep-id makes
  # the in-container user share the host UID so build outputs land with the
  # right owner. Docker takes the explicit -u form.
  ifeq ($(CONTAINER_RUNTIME),podman)
    USER_FLAGS := --userns=keep-id
  else
    USER_FLAGS := -u $(shell id -u):$(shell id -g)
  endif
  # Base flags shared by every container invocation. `--network host` lets
  # OTA flashing reach the ESP32 over WiFi without extra plumbing.
  DOCKER_BASE := $(CONTAINER_RUNTIME) run --rm $(DOCKER_TTY) \
    -v $(CURDIR):/workspace \
    -v $(CURDIR)/.docker/home:/home/builder \
    -e HOME=/home/builder \
    -e IDF_TOOLS_PATH=/home/builder/.espressif \
    $(USER_FLAGS) \
    --network host \
    -w /workspace
  # Extra flags for serial flash/monitor: pass the USB tty into the container,
  # plus the device's group so the in-container user can read/write it.
  # Set ESP_PORT="/dev/ttyUSB0" (or similar) in user_config.cmake; if unset,
  # the targets still run but idf.py auto-detect won't see any host devices.
  DOCKER_USB :=
  ifneq ($(ESP_PORT_VAR),)
    DOCKER_USB := --device=$(ESP_PORT_VAR) --group-add $(shell stat -c '%g' $(ESP_PORT_VAR) 2>/dev/null || echo 0)
  endif
  RUN := $(DOCKER_BASE) $(DOCKER_IMAGE)
  RUN_USB := $(DOCKER_BASE) $(DOCKER_USB) $(DOCKER_IMAGE)
  LOCAL_IDF_PATH := /workspace/third_party/esp-idf
else
  RUN :=
  RUN_USB :=
  LOCAL_IDF_PATH := $(CURDIR)/third_party/esp-idf
endif

CLEAN_DIRS := \
	$(BUILD_DIR)

CLEAN_FILES := \
	config/32raven.config.old \
	config/defconfig \
	esp32/main/esp32_config.hpp \
	stm32/Drivers/Inc/stm32_config.hpp \
	stm32/Drivers/Inc/stm32_limits.hpp \
	stm32/Drivers/Inc/ee_schema.hpp

.PHONY: help configure all stm32 esp32 clean distclean flash-esp32 monitor-esp32 idf-install 32raven-menuconfig format-cpp enable-docker disable-docker docker-image setup-vscode

help:
	@echo "Targets:"
	@echo "  configure           - Configure CMake"
	@echo "  all                 - Build all firmware"
	@echo "  32raven-menuconfig  - Run 32Raven menuconfig (ESP32 + STM32)"
	@echo "  esp32               - Build ESP32 firmware"
	@echo "  stm32               - Build STM32 firmware (optimized / Release)"
	@echo "  stm32-debug         - Build STM32 firmware (debug, -O0 -g3)"
	@echo "  format-cpp          - Run clang-format on STM32/ESP32 C++ source headers"
	@echo "  clean               - Clean build directory"
	@echo "  esp32-menuconfig    - Run ESP-IDF menuconfig"
	@echo "  flash-esp32         - Flash ESP32 via serial"
	@echo "  monitor-esp32       - Monitor ESP32 via serial"
	@echo "  flash-monitor-esp32 - Flash and monitor ESP32 via serial"
	@echo "  flash-wifi-esp32    - Flash ESP32 via WiFi (OTA)"
	@echo "  flash-wifi-stm32    - Flash STM32 via WiFi (Bridge)"
	@echo "  idf-install         - Install local ESP-IDF tools for third_party/esp-idf"
	@echo "  enable-docker       - Persist USE_DOCKER=1 in .build-mode (builds run in container)"
	@echo "  disable-docker      - Remove .build-mode (builds run on host)"
	@echo "  docker-image        - (Re)build the $(DOCKER_IMAGE) container image"
	@echo "  setup-vscode        - Configure VSCode settings for 32Raven (git submodule handling, watchers)"
	@echo "Vars: GEN='Ninja' or 'Unix Makefiles', BUILD_ROOT=build, BUILD_DIR=build/<generator>, IDF_PATH=..., STM32_TOOLCHAIN_FILE=..., USE_DOCKER=0|1, DOCKER_IMAGE=..."

configure:
	@mkdir -p "$(BUILD_DIR)"
	$(RUN) $(CMAKE) -S . -B "$(BUILD_DIR)" -G "$(GEN)"

all: configure
	$(RUN) $(CMAKE) --build "$(BUILD_DIR)" --target all_firmware

esp32: configure
	$(RUN) $(CMAKE) --build "$(BUILD_DIR)" --target esp32
	@printf "\n"
	$(RUN) python3 tools/esp32_size_metrics.py --build-dir "$(BUILD_DIR)/esp32"

stm32: configure
	$(RUN) $(CMAKE) --build "$(BUILD_DIR)" --target stm32

stm32-debug: configure
	$(RUN) $(CMAKE) --build "$(BUILD_DIR)" --target stm32-debug

format-cpp:
	@$(RUN) bash -lc 'set -euo pipefail; \
		command -v clang-format >/dev/null; \
		rg --files esp32 stm32 -g "*.cpp" -g "*.hpp" \
		  | grep -vxF "esp32/main/esp32_config.hpp" \
		  | grep -vxF "stm32/Drivers/Inc/stm32_config.hpp" \
		  | grep -vxF "stm32/Drivers/Inc/stm32_limits.hpp" \
		  | grep -vxF "stm32/Drivers/Inc/ee_schema.hpp" \
		  | while IFS= read -r file; do \
		      clang-format -i "$$file"; \
		    done'

32raven-menuconfig:
	$(RUN) python3 scripts/32raven_menuconfig.py --kconfig config/Kconfig --config config/32raven.config

export IDF_PATH ?= $(if $(IDF_PATH_VAR),$(IDF_PATH_VAR),$(LOCAL_IDF_PATH))

IDF_ARGS :=
ifneq ($(ESP_PORT_VAR),)
	IDF_ARGS += -p $(ESP_PORT_VAR)
endif
ifneq ($(ESP_BAUD_VAR),)
	IDF_ARGS += -b $(ESP_BAUD_VAR)
endif

idf-install:
	$(RUN) bash -lc "cd \"$(IDF_PATH)\" && ./install.sh"

esp32-menuconfig:
	$(RUN) bash -lc ". \"$(IDF_PATH)/export.sh\" && cd esp32 && idf.py $(IDF_ARGS) -B ../$(BUILD_DIR)/esp32 menuconfig"

# Serial flash/monitor: in Docker mode the USB tty is passed through via
# --device (set ESP_PORT in user_config.cmake). On host, runs directly.
flash-esp32: esp32
	$(RUN_USB) bash -lc ". \"$(IDF_PATH)/export.sh\" && cd esp32 && idf.py $(IDF_ARGS) -B ../$(BUILD_DIR)/esp32 flash"

monitor-esp32:
	$(RUN_USB) bash -lc ". \"$(IDF_PATH)/export.sh\" && cd esp32 && idf.py $(IDF_ARGS) -B ../$(BUILD_DIR)/esp32 monitor"

flash-monitor-esp32: esp32
	$(RUN_USB) bash -lc ". \"$(IDF_PATH)/export.sh\" && cd esp32 && idf.py $(IDF_ARGS) -B ../$(BUILD_DIR)/esp32 flash monitor"

clean:
	@echo "Cleaning ESP-IDF..."
	-$(RUN) bash -lc ". \"$(IDF_PATH)/export.sh\" && cd esp32 && idf.py $(IDF_ARGS) -B ../$(BUILD_DIR)/esp32 fullclean"
	@echo "Removing clean directories"
	$(RUN) $(RM) -rf $(CLEAN_DIRS)
	@echo "Removing clean files"
	$(RUN) $(RM) -f $(CLEAN_FILES)

# Default ESP32 IP
ESP_IP ?= 192.168.4.1

# WiFi Flashing
flash-wifi-stm32: stm32
	$(RUN) python3 tools/esp32_client.py $(ESP_IP) flash $(BUILD_DIR)/stm32/32Raven_stm32.bin

flash-wifi-esp32: esp32
	-pkill -f esp32_client.py || true
	$(RUN) python3 tools/esp32_client.py $(ESP_IP) flash_esp $(BUILD_DIR)/esp32/32Raven_esp32.bin

distclean: clean
	@echo "Removing all generator build directories in $(BUILD_ROOT)/"
	$(RUN) $(RM) -rf "$(BUILD_ROOT)"

# ---- Docker mode toggle ---------------------------------------------------
# `make enable-docker` builds the image (if missing) and writes USE_DOCKER=1 to
# .build-mode so subsequent `make stm32`, `make esp32`, etc. run inside the
# container. `make disable-docker` removes the file. Both files are gitignored.

docker-image:
	@mkdir -p .docker/home
	$(CONTAINER_RUNTIME) build -t $(DOCKER_IMAGE) -f Dockerfile .

enable-docker:
	@command -v $(CONTAINER_RUNTIME) >/dev/null || { echo "$(CONTAINER_RUNTIME) not found in PATH" >&2; exit 1; }
	@$(CONTAINER_RUNTIME) image inspect $(DOCKER_IMAGE) >/dev/null 2>&1 || $(MAKE) docker-image
	@mkdir -p .docker/home
	@echo "USE_DOCKER=1" > .build-mode
	@echo "Docker mode enabled (image: $(DOCKER_IMAGE)). Builds will run in container."
	@echo "Run 'make idf-install' once to populate .docker/home/.espressif before the first ESP32 build."

disable-docker:
	@rm -f .build-mode
	@echo "Docker mode disabled. Builds will run on the host."

# ---- VSCode Setup -------------------------------------------------------

setup-vscode:
	@bash scripts/setup-vscode.sh
