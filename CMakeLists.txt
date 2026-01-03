cmake_minimum_required(VERSION 3.16)
project(32raven)

# Orchestrate independent sub-project builds.
# Outputs go to: ${CMAKE_BINARY_DIR}/stm32 and ${CMAKE_BINARY_DIR}/esp32

# -----------------------------------------------------------------------------
# Configuration helpers
# -----------------------------------------------------------------------------

# Allow passing -DIDF_PATH=/path/to/esp-idf, otherwise try env var.
if(NOT DEFINED IDF_PATH AND DEFINED ENV{IDF_PATH})
  set(IDF_PATH "$ENV{IDF_PATH}")
endif()

# Optional auto-detect (only used when building esp32 target).
function(detect_idf_path out_var)
  if(DEFINED IDF_PATH AND EXISTS "${IDF_PATH}/export.sh")
    set(${out_var} "${IDF_PATH}" PARENT_SCOPE)
    return()
  endif()

  file(GLOB _idf_candidates "$ENV{HOME}/esp/*/esp-idf")
  if(_idf_candidates)
    list(GET _idf_candidates 0 _found)
    if(EXISTS "${_found}/export.sh")
      set(${out_var} "${_found}" PARENT_SCOPE)
      return()
    endif()
  endif()

  set(${out_var} "" PARENT_SCOPE)
endfunction()

# -----------------------------------------------------------------------------
# STM32 Firmware
# -----------------------------------------------------------------------------
add_custom_target(stm32
  COMMAND ${CMAKE_COMMAND}
          -S ${CMAKE_CURRENT_SOURCE_DIR}/stm32
          -B ${CMAKE_CURRENT_BINARY_DIR}/stm32
          -G Ninja
          -DCMAKE_TOOLCHAIN_FILE=${CMAKE_CURRENT_SOURCE_DIR}/cmake/gcc-arm-none-eabi.cmake
          -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
  COMMAND ${CMAKE_COMMAND} --build ${CMAKE_CURRENT_BINARY_DIR}/stm32
  COMMENT "Building STM32 firmware"
  VERBATIM
)

# -----------------------------------------------------------------------------
# ESP32 Firmware
# -----------------------------------------------------------------------------
add_custom_target(esp32
  COMMAND ${CMAKE_COMMAND} -E echo "Building ESP32 firmware (ESP-IDF)"
  COMMAND bash -c
          "set -e; \
           IDF='${IDF_PATH}'; \
           if [ -z \"$$IDF\" ] || [ ! -f \"$$IDF/export.sh\" ]; then \
             IDF=''; \
           fi; \
           if [ -z \"$$IDF\" ]; then \
             for d in \"$HOME\"/esp/*/esp-idf; do \
               if [ -f \"$$d/export.sh\" ]; then IDF=\"$$d\"; break; fi; \
             done; \
           fi; \
           if [ -z \"$$IDF\" ]; then \
             echo 'ERROR: IDF_PATH not found. Set IDF_PATH env var or pass -DIDF_PATH=/path/to/esp-idf'; \
             exit 1; \
           fi; \
           . \"$$IDF/export.sh\" > /dev/null 2>&1; \
           idf.py -C '${CMAKE_CURRENT_SOURCE_DIR}/esp32' -B '${CMAKE_CURRENT_BINARY_DIR}/esp32' \
             -D CMAKE_EXPORT_COMPILE_COMMANDS=ON build"
  COMMENT "Building ESP32 firmware"
  VERBATIM
)

# -----------------------------------------------------------------------------
# Default target
# -----------------------------------------------------------------------------
add_custom_target(all_firmware DEPENDS stm32 esp32)
