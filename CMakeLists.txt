cmake_minimum_required(VERSION 3.16)
project(32raven)

# Goal: Orchestrate builds for independent sub-projects.
# Output is directed to ${CMAKE_BINARY_DIR}/<subproject>

# -----------------------------------------------------------------------------
# Configuration
# -----------------------------------------------------------------------------

# Try to find IDF_PATH from environment if not explicitly set
if(NOT DEFINED IDF_PATH)
    set(IDF_PATH "$ENV{IDF_PATH}")
endif()

# We only enforce IDF_PATH if we are actually trying to build the ESP32 target
# However, for consistency, we warn if it's missing.
if(NOT EXISTS "${IDF_PATH}/export.sh")
    message(WARNING "IDF_PATH not found or invalid (${IDF_PATH}). ESP32 build target may fail. Set -DIDF_PATH=/path/to/esp-idf")
else()
    message(STATUS "Using ESP-IDF at: ${IDF_PATH}")
endif()


# -----------------------------------------------------------------------------
# STM32 Firmware
# -----------------------------------------------------------------------------
add_custom_target(stm32 ALL
    COMMAND ${CMAKE_COMMAND} -S ${CMAKE_CURRENT_SOURCE_DIR}/stm32 -B ${CMAKE_CURRENT_BINARY_DIR}/stm32 -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake
    COMMAND ${CMAKE_COMMAND} --build ${CMAKE_CURRENT_BINARY_DIR}/stm32
    COMMENT "Building STM32 Firmware"
)

# -----------------------------------------------------------------------------
# ESP32 Firmware
# -----------------------------------------------------------------------------
# We use bash to source the export.sh script before running idf.py
add_custom_target(esp32 ALL
    COMMAND bash -c ". ${IDF_PATH}/export.sh > /dev/null 2>&1 && idf.py -C ${CMAKE_CURRENT_SOURCE_DIR}/esp32 -B ${CMAKE_CURRENT_BINARY_DIR}/esp32 build"
    COMMENT "Building ESP32 Firmware"
    VERBATIM
)
