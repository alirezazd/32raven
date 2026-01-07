cmake_minimum_required(VERSION 3.16)
project(32raven NONE)

# -----------------------------------------------------------------------------
# Optional user config
# -----------------------------------------------------------------------------
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/user_config.cmake")
  include("${CMAKE_CURRENT_SOURCE_DIR}/user_config.cmake")
endif()

# -----------------------------------------------------------------------------
# Inputs
# -----------------------------------------------------------------------------
set(IDF_PATH "" CACHE PATH "Path to esp-idf (or set env IDF_PATH)")
if(NOT IDF_PATH AND DEFINED ENV{IDF_PATH})
  set(IDF_PATH "$ENV{IDF_PATH}")
endif()
if(NOT IDF_PATH)
  message(FATAL_ERROR
    "IDF_PATH is required.\n"
    "Set env: export IDF_PATH=/path/to/esp-idf\n"
    "or pass: -DIDF_PATH=/path/to/esp-idf"
  )
endif()
if(NOT EXISTS "${IDF_PATH}/export.sh")
  message(FATAL_ERROR "IDF_PATH is invalid: '${IDF_PATH}' (missing export.sh)")
endif()

add_compile_options(
    -Wall
    -Wextra
    -Wshadow
    -Wconversion
    -Wsign-conversion
    -Wdouble-promotion
    -Wformat=2
    -Wundef
    -Werror=return-type
)

set(STM32_TOOLCHAIN_FILE "" CACHE FILEPATH "STM32 toolchain file (gcc-arm-none-eabi.cmake)")

if(STM32_TOOLCHAIN_FILE AND EXISTS "${STM32_TOOLCHAIN_FILE}")
  set(_STM32_TC "${STM32_TOOLCHAIN_FILE}")
elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/stm32/cmake/gcc-arm-none-eabi.cmake")
  set(_STM32_TC "${CMAKE_CURRENT_SOURCE_DIR}/stm32/cmake/gcc-arm-none-eabi.cmake")
else()
  set(_STM32_TC "")
endif()

# Multi-config support: only relevant for Visual Studio/Xcode style generators.
set(_CFG_ARG "")
if(CMAKE_CONFIGURATION_TYPES)
  set(_CFG_ARG --config $<CONFIG>)
endif()

# -----------------------------------------------------------------------------
# STM32 target
# -----------------------------------------------------------------------------
add_custom_target(stm32
  COMMAND ${CMAKE_COMMAND} -E echo "Building STM32 firmware"
  COMMAND ${CMAKE_COMMAND}
          -S ${CMAKE_CURRENT_SOURCE_DIR}/stm32
          -B ${CMAKE_CURRENT_BINARY_DIR}/stm32
          -G "${CMAKE_GENERATOR}"
          -DCMAKE_MAKE_PROGRAM=${CMAKE_MAKE_PROGRAM}
          -DCMAKE_TOOLCHAIN_FILE:FILEPATH=${_STM32_TC}
          -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
  COMMAND ${CMAKE_COMMAND} --build ${CMAKE_CURRENT_BINARY_DIR}/stm32 ${_CFG_ARG}
  VERBATIM
)

if(NOT _STM32_TC)
  add_custom_command(TARGET stm32 PRE_BUILD
    COMMAND ${CMAKE_COMMAND} -E echo "ERROR: STM32 toolchain file not found."
    COMMAND ${CMAKE_COMMAND} -E echo "Expected: ${CMAKE_CURRENT_SOURCE_DIR}/stm32/cmake/gcc-arm-none-eabi.cmake"
    COMMAND ${CMAKE_COMMAND} -E echo "Or pass: -DSTM32_TOOLCHAIN_FILE=/path/to/gcc-arm-none-eabi.cmake"
    COMMAND ${CMAKE_COMMAND} -E false
  )
endif()

# -----------------------------------------------------------------------------
# ESP32 target
# -----------------------------------------------------------------------------
add_custom_target(esp32
  COMMAND ${CMAKE_COMMAND} -E echo "Building ESP32 firmware"
  COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/esp32/check_forbidden.sh
  COMMAND bash -lc
    "set -euo pipefail; \
     . '${IDF_PATH}/export.sh'; \
     idf.py -C '${CMAKE_CURRENT_SOURCE_DIR}/esp32' \
            -B '${CMAKE_CURRENT_BINARY_DIR}/esp32' \
            -DCMAKE_EXPORT_COMPILE_COMMANDS=ON build"
  VERBATIM
)

# -----------------------------------------------------------------------------
# Compliance Checks
# -----------------------------------------------------------------------------
# Define exceptions for the forbidden header/keyword compliance check.
# Each item will be treated as a grep pattern that, if matched, allows the line.
set(FORBIDDEN_EXCEPTIONS
    "#include <vector>"  # Example: remove this when you actually need to catch it
    "placement new"
)
string(REPLACE ";" "\n" FORBIDDEN_EXCEPTIONS_CONTENT "${FORBIDDEN_EXCEPTIONS}")
file(WRITE "${CMAKE_CURRENT_SOURCE_DIR}/esp32/forbidden_exceptions.txt" "${FORBIDDEN_EXCEPTIONS_CONTENT}")

# -----------------------------------------------------------------------------
# Build both
# -----------------------------------------------------------------------------
add_custom_target(all_firmware DEPENDS stm32 esp32)
