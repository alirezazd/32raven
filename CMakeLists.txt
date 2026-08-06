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
set(IDF_LOCAL_PATH "${CMAKE_CURRENT_SOURCE_DIR}/third_party/esp-idf")
set(IDF_PATH "${IDF_LOCAL_PATH}" CACHE PATH "Path to esp-idf")
if(DEFINED ENV{IDF_PATH} AND NOT "$ENV{IDF_PATH}" STREQUAL "")
  set(IDF_PATH "$ENV{IDF_PATH}" CACHE PATH "Path to esp-idf" FORCE)
endif()

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
# Compliance Checks
# -----------------------------------------------------------------------------
# Write the repo default exceptions plus any user-defined exceptions to a
# temporary file in the build directory. This allows 'user_config.cmake' to
# define 'set(LINT_EXCEPTIONS "pattern1" "pattern2")' without losing the shared
# repo allowlist.
set(LINT_DEFAULT_EXCEPTIONS_FILE
    "${CMAKE_CURRENT_SOURCE_DIR}/scripts/lint/forbidden_exceptions.txt")
set(LINT_EXCEPTIONS_FILE "${CMAKE_CURRENT_BINARY_DIR}/lint_exceptions.txt")
if(EXISTS "${LINT_DEFAULT_EXCEPTIONS_FILE}")
    file(READ "${LINT_DEFAULT_EXCEPTIONS_FILE}" LINT_EXCEPTIONS_CONTENT)
else()
    set(LINT_EXCEPTIONS_CONTENT "")
endif()
if(DEFINED LINT_EXCEPTIONS)
    string(REPLACE ";" "\n" LINT_USER_EXCEPTIONS_CONTENT "${LINT_EXCEPTIONS}")
    if(NOT "${LINT_EXCEPTIONS_CONTENT}" STREQUAL "" AND
       NOT "${LINT_USER_EXCEPTIONS_CONTENT}" STREQUAL "")
        string(APPEND LINT_EXCEPTIONS_CONTENT "\n")
    endif()
    string(APPEND LINT_EXCEPTIONS_CONTENT "${LINT_USER_EXCEPTIONS_CONTENT}")
endif()
file(WRITE "${LINT_EXCEPTIONS_FILE}" "${LINT_EXCEPTIONS_CONTENT}")

# -----------------------------------------------------------------------------
# Shared generated config
# -----------------------------------------------------------------------------
# Owned here rather than by either sub-build: those are disjoint CMake instances
# that 'all_firmware' runs concurrently, so a rule in each would put two writers
# on this one path.
find_program(UV_EXECUTABLE NAMES uv)
if(NOT UV_EXECUTABLE)
  message(FATAL_ERROR
    "uv not found; it runs the config generators.\n"
    "Install it (https://docs.astral.sh/uv/) or build in the container: make enable-docker")
endif()
set(UV_RUN ${UV_EXECUTABLE} run --quiet --script)
find_program(CLANG_FORMAT_EXECUTABLE NAMES clang-format)

set(COMMON_KCONFIG       "${CMAKE_CURRENT_SOURCE_DIR}/config/Kconfig")
set(COMMON_MASTER_CONFIG "${CMAKE_CURRENT_SOURCE_DIR}/config/32raven.config")
set(COMMON_CONFIG_GENERATOR
    "${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_common_config.py")
set(COMMON_CONFIG_TEMPLATE
    "${CMAKE_CURRENT_SOURCE_DIR}/scripts/templates/common_config.hpp.j2")
set(COMMON_CONFIG_HPP "${CMAKE_CURRENT_SOURCE_DIR}/libs/inc/common_config.hpp")

set(_FMT_COMMON_CMD)
if(CLANG_FORMAT_EXECUTABLE)
  set(_FMT_COMMON_CMD COMMAND ${CLANG_FORMAT_EXECUTABLE} -i ${COMMON_CONFIG_HPP})
endif()

# DEPENDS stays limited to what the generator reads; anything wider rebuilds both
# firmwares on changes that cannot affect this header.
add_custom_command(
  OUTPUT ${COMMON_CONFIG_HPP}
  COMMAND ${UV_RUN} ${COMMON_CONFIG_GENERATOR}
          --kconfig ${COMMON_KCONFIG}
          --config ${COMMON_MASTER_CONFIG}
          --out ${COMMON_CONFIG_HPP}
  ${_FMT_COMMON_CMD}
  DEPENDS ${COMMON_KCONFIG}
          ${COMMON_MASTER_CONFIG}
          ${COMMON_CONFIG_GENERATOR}
          ${COMMON_CONFIG_TEMPLATE}
  COMMENT "Generating shared configuration header"
  VERBATIM)

add_custom_target(generate_common_config DEPENDS ${COMMON_CONFIG_HPP})

file(GLOB_RECURSE LINT_LIB_SOURCE_FILES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/libs/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/libs/*.hpp")
# Appears mid-build the first time, which would change the glob result and force
# a CMake re-run. The lint stamps name it directly in DEPENDS instead.
list(REMOVE_ITEM LINT_LIB_SOURCE_FILES "${COMMON_CONFIG_HPP}")

file(GLOB_RECURSE LINT_ESP32_SOURCE_FILES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/esp32/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/esp32/*.hpp")

file(GLOB_RECURSE LINT_STM32_SOURCE_FILES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/stm32/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/stm32/*.hpp")

set(FORBIDDEN_CHECK_ESP32_STAMP
    "${CMAKE_CURRENT_BINARY_DIR}/forbidden_check_esp32.stamp")
add_custom_command(
  OUTPUT "${FORBIDDEN_CHECK_ESP32_STAMP}"
  COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/scripts/lint/check_forbidden.sh
          "${LINT_EXCEPTIONS_FILE}"
          "${CMAKE_CURRENT_SOURCE_DIR}/esp32"
          "${CMAKE_CURRENT_SOURCE_DIR}/libs"
  COMMAND ${CMAKE_COMMAND} -E touch "${FORBIDDEN_CHECK_ESP32_STAMP}"
  DEPENDS
    ${LINT_ESP32_SOURCE_FILES}
    ${LINT_LIB_SOURCE_FILES}
    ${COMMON_CONFIG_HPP}
    ${CMAKE_CURRENT_SOURCE_DIR}/scripts/lint/check_forbidden.sh
    ${CMAKE_CURRENT_SOURCE_DIR}/scripts/lint/forbidden_exceptions.txt
    ${LINT_EXCEPTIONS_FILE}
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "Running ESP32 static analysis (check_forbidden.sh)..."
  VERBATIM
)

set(FORBIDDEN_CHECK_STM32_STAMP
    "${CMAKE_CURRENT_BINARY_DIR}/forbidden_check_stm32.stamp")
add_custom_command(
  OUTPUT "${FORBIDDEN_CHECK_STM32_STAMP}"
  COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/scripts/lint/check_forbidden.sh
          "${LINT_EXCEPTIONS_FILE}"
          "${CMAKE_CURRENT_SOURCE_DIR}/stm32"
          "${CMAKE_CURRENT_SOURCE_DIR}/libs"
  COMMAND ${CMAKE_COMMAND} -E touch "${FORBIDDEN_CHECK_STM32_STAMP}"
  DEPENDS
    ${LINT_STM32_SOURCE_FILES}
    ${LINT_LIB_SOURCE_FILES}
    ${COMMON_CONFIG_HPP}
    ${CMAKE_CURRENT_SOURCE_DIR}/scripts/lint/check_forbidden.sh
    ${CMAKE_CURRENT_SOURCE_DIR}/scripts/lint/forbidden_exceptions.txt
    ${LINT_EXCEPTIONS_FILE}
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "Running STM32 static analysis (check_forbidden.sh)..."
  VERBATIM
)

add_custom_target(forbidden_check_esp32 DEPENDS "${FORBIDDEN_CHECK_ESP32_STAMP}")
add_custom_target(forbidden_check_stm32 DEPENDS "${FORBIDDEN_CHECK_STM32_STAMP}")

# -----------------------------------------------------------------------------
# STM32 target
# -----------------------------------------------------------------------------
# Configure + build the STM32 firmware sub-project with a given build type, each
# in its own binary dir. Both link through the same toolchain, so both print the
# flash/RAM/CCRAM usage (-Wl,--print-memory-usage). 'stm32' is the optimized
# image (Release: -O2 -flto); 'stm32-debug' is the -O0 -g3 build.
function(_add_stm32_target name build_type)
  add_custom_target(${name}
    DEPENDS forbidden_check_stm32 generate_common_config
    COMMAND ${CMAKE_COMMAND} -E echo "Building STM32 firmware"
    COMMAND ${CMAKE_COMMAND}
            -S ${CMAKE_CURRENT_SOURCE_DIR}/stm32
            -B ${CMAKE_CURRENT_BINARY_DIR}/${name}
            -G "${CMAKE_GENERATOR}"
            -DCMAKE_MAKE_PROGRAM=${CMAKE_MAKE_PROGRAM}
            -DCMAKE_TOOLCHAIN_FILE:FILEPATH=${_STM32_TC}
            -DCMAKE_BUILD_TYPE=${build_type}
            -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
    COMMAND ${CMAKE_COMMAND} --build ${CMAKE_CURRENT_BINARY_DIR}/${name} ${_CFG_ARG}
    VERBATIM
  )
  if(NOT _STM32_TC)
    add_custom_command(TARGET ${name} PRE_BUILD
      COMMAND ${CMAKE_COMMAND} -E echo "ERROR: STM32 toolchain file not found."
      COMMAND ${CMAKE_COMMAND} -E echo "Expected: ${CMAKE_CURRENT_SOURCE_DIR}/stm32/cmake/gcc-arm-none-eabi.cmake"
      COMMAND ${CMAKE_COMMAND} -E echo "Or pass: -DSTM32_TOOLCHAIN_FILE=/path/to/gcc-arm-none-eabi.cmake"
      COMMAND ${CMAKE_COMMAND} -E false
    )
  endif()
endfunction()

_add_stm32_target(stm32       Release)
_add_stm32_target(stm32-debug Debug)

add_custom_target(esp32
  DEPENDS forbidden_check_esp32 generate_common_config
  COMMAND ${CMAKE_COMMAND} -E echo "Building ESP32 firmware"
  COMMAND bash -lc
    "set -euo pipefail; \
     if [ -z '${IDF_PATH}' ]; then \
       echo 'IDF_PATH is empty.'; \
       exit 1; \
     fi; \
     if [ ! -f '${IDF_PATH}/export.sh' ]; then \
       echo \"ESP-IDF not found at '${IDF_PATH}'\"; \
       echo 'Run: git submodule update --init --recursive third_party/esp-idf'; \
       echo 'Then: make idf-install'; \
       exit 1; \
     fi; \
     . '${IDF_PATH}/export.sh'; \
     idf.py -C '${CMAKE_CURRENT_SOURCE_DIR}/esp32' \
            -B '${CMAKE_CURRENT_BINARY_DIR}/esp32' \
            -DCMAKE_EXPORT_COMPILE_COMMANDS=ON build"
  VERBATIM
)

# -----------------------------------------------------------------------------
# Build both
# -----------------------------------------------------------------------------
add_custom_target(all_firmware DEPENDS stm32 esp32)
