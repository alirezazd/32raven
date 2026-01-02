BUILD_DIR := build
CMAKE := /usr/bin/cmake
LOG_FILE := $(BUILD_DIR)/make.log

# Colors
GREEN  := \033[0;32m
YELLOW := \033[0;33m
RED    := \033[0;31m
NC     := \033[0m # No Color

.PHONY: all stm32 esp32 clean configure flash-stm32 flash-esp32

# Default target: Build all
all: configure
	@echo "$(YELLOW)[+] Building All Targets...$(NC)"
	@$(CMAKE) --build $(BUILD_DIR) > $(LOG_FILE) 2>&1 || (echo "$(RED)[-] Build Failed! See $(LOG_FILE) for details.$(NC)"; exit 1)
	@echo "$(GREEN)[✔] Build Complete!$(NC)"

# Build STM32 only
stm32: configure
	@echo "$(YELLOW)[+] Building STM32 Firmware...$(NC)"
	@$(CMAKE) --build $(BUILD_DIR) --target stm32 > $(LOG_FILE) 2>&1 || (echo "$(RED)[-] STM32 Build Failed! See $(LOG_FILE) for details.$(NC)"; exit 1)
	@echo "$(GREEN)[✔] STM32 Build Complete!$(NC)"

# Build ESP32 only
esp32: configure
	@echo "$(YELLOW)[+] Building ESP32 Firmware...$(NC)"
	@$(CMAKE) --build $(BUILD_DIR) --target esp32 > $(LOG_FILE) 2>&1 || (echo "$(RED)[-] ESP32 Build Failed! See $(LOG_FILE) for details.$(NC)"; exit 1)
	@echo "$(GREEN)[✔] ESP32 Build Complete!$(NC)"

# Configure the project if build directory doesn't exist
configure:
	@if [ ! -d "$(BUILD_DIR)" ]; then \
		echo "$(YELLOW)[+] Configuring project...$(NC)"; \
		mkdir -p $(BUILD_DIR); \
		$(CMAKE) -S . -B $(BUILD_DIR) > $(BUILD_DIR)/configure.log 2>&1 || (echo "$(RED)[-] Configuration Failed! See $(BUILD_DIR)/configure.log$(NC)"; exit 1); \
	fi

# Clean build directory
clean:
	@echo "$(YELLOW)[+] Cleaning build directory...$(NC)"
	@rm -rf $(BUILD_DIR)
	@echo "$(GREEN)[✔] Clean Complete!$(NC)"

# Helpers
flash-esp32:
	idf.py -C esp32 -B $(BUILD_DIR)/esp32 flash monitor
