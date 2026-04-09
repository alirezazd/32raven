cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME 32Raven_stm32)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C CXX ASM)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_program(CLANG_FORMAT_EXECUTABLE NAMES clang-format)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

set(STM32_KCONFIG "${CMAKE_CURRENT_SOURCE_DIR}/../config/Kconfig")
set(STM32_MASTER_CONFIG "${CMAKE_CURRENT_SOURCE_DIR}/../config/32raven.config")
set(STM32_CONFIG_GENERATOR
    "${CMAKE_CURRENT_SOURCE_DIR}/../scripts/generate_stm32_config.py")
set(STM32_CONFIG_TEMPLATE_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/../scripts/templates")
set(STM32_RUNTIME_CONFIG_TEMPLATE
    "${STM32_CONFIG_TEMPLATE_DIR}/stm32_config.hpp.j2")
set(STM32_LIMITS_TEMPLATE
    "${STM32_CONFIG_TEMPLATE_DIR}/stm32_limits.hpp.j2")
set(STM32_EE_CONFIG "${CMAKE_CURRENT_SOURCE_DIR}/../config/ee.toml")
set(STM32_EE_SCHEMA_GENERATOR
    "${CMAKE_CURRENT_SOURCE_DIR}/../scripts/generate_ee_schema.py")
set(STM32_EE_SCHEMA_TEMPLATE
    "${STM32_CONFIG_TEMPLATE_DIR}/ee_schema.hpp.j2")
set(STM32_RUNTIME_CONFIG_HPP
    "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/USER/inc/stm32_config.hpp")
set(STM32_LIMITS_HPP
    "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/USER/inc/stm32_limits.hpp")
set(STM32_EE_SCHEMA_HPP
    "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/USER/inc/ee_schema.hpp")

if(CLANG_FORMAT_EXECUTABLE)
    add_custom_command(
        OUTPUT ${STM32_RUNTIME_CONFIG_HPP} ${STM32_LIMITS_HPP}
        COMMAND ${Python3_EXECUTABLE} ${STM32_CONFIG_GENERATOR}
                --kconfig ${STM32_KCONFIG}
                --config ${STM32_MASTER_CONFIG}
                --runtime-out ${STM32_RUNTIME_CONFIG_HPP}
                --limits-out ${STM32_LIMITS_HPP}
        COMMAND ${CLANG_FORMAT_EXECUTABLE} -i
                ${STM32_RUNTIME_CONFIG_HPP}
                ${STM32_LIMITS_HPP}
        DEPENDS ${STM32_KCONFIG} ${STM32_MASTER_CONFIG}
                ${STM32_CONFIG_GENERATOR}
                ${STM32_RUNTIME_CONFIG_TEMPLATE}
                ${STM32_LIMITS_TEMPLATE}
        COMMENT "Generating STM32 configuration headers"
        VERBATIM)
else()
    add_custom_command(
        OUTPUT ${STM32_RUNTIME_CONFIG_HPP} ${STM32_LIMITS_HPP}
        COMMAND ${Python3_EXECUTABLE} ${STM32_CONFIG_GENERATOR}
                --kconfig ${STM32_KCONFIG}
                --config ${STM32_MASTER_CONFIG}
                --runtime-out ${STM32_RUNTIME_CONFIG_HPP}
                --limits-out ${STM32_LIMITS_HPP}
        DEPENDS ${STM32_KCONFIG} ${STM32_MASTER_CONFIG}
                ${STM32_CONFIG_GENERATOR}
                ${STM32_RUNTIME_CONFIG_TEMPLATE}
                ${STM32_LIMITS_TEMPLATE}
        COMMENT "Generating STM32 configuration headers"
        VERBATIM)
endif()

add_custom_target(generate_stm32_config
                  DEPENDS ${STM32_RUNTIME_CONFIG_HPP}
                          ${STM32_LIMITS_HPP})
add_dependencies(${CMAKE_PROJECT_NAME} generate_stm32_config)

if(CLANG_FORMAT_EXECUTABLE)
    add_custom_command(
        OUTPUT ${STM32_EE_SCHEMA_HPP}
        COMMAND ${Python3_EXECUTABLE} ${STM32_EE_SCHEMA_GENERATOR}
                --config ${STM32_EE_CONFIG}
                --out ${STM32_EE_SCHEMA_HPP}
        COMMAND ${CLANG_FORMAT_EXECUTABLE} -i ${STM32_EE_SCHEMA_HPP}
        DEPENDS ${STM32_EE_CONFIG} ${STM32_EE_SCHEMA_GENERATOR}
                ${STM32_EE_SCHEMA_TEMPLATE}
        COMMENT "Generating EEPROM schema header"
        VERBATIM)
else()
    add_custom_command(
        OUTPUT ${STM32_EE_SCHEMA_HPP}
        COMMAND ${Python3_EXECUTABLE} ${STM32_EE_SCHEMA_GENERATOR}
                --config ${STM32_EE_CONFIG}
                --out ${STM32_EE_SCHEMA_HPP}
        DEPENDS ${STM32_EE_CONFIG} ${STM32_EE_SCHEMA_GENERATOR}
                ${STM32_EE_SCHEMA_TEMPLATE}
        COMMENT "Generating EEPROM schema header"
        VERBATIM)
endif()

add_custom_target(generate_ee_schema DEPENDS ${STM32_EE_SCHEMA_HPP})
add_dependencies(${CMAKE_PROJECT_NAME} generate_ee_schema)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    Core/Src/stm32f4xx_hal_msp.c
    Core/Src/stm32f4xx_it.c
    Core/Src/syscalls.c
    Core/Src/sysmem.c
    Core/Src/ctx.cpp
    Core/Src/main.cpp
    Core/Src/system.cpp
    Drivers/USER/cc/gpio.cpp
    Drivers/USER/cc/time_base.cpp
    Drivers/USER/cc/led.cpp
    Drivers/USER/cc/battery.cpp

    Drivers/USER/cc/dshot_tim1.cpp
    Drivers/USER/cc/spi.cpp
    Drivers/USER/cc/button.cpp
    Drivers/USER/cc/uart.cpp
    Drivers/USER/cc/m10.cpp
    Drivers/USER/cc/icm42688p.cpp
    Drivers/USER/cc/rc_receiver.cpp
    Drivers/USER/cc/ee.cpp
    Core/Src/states.cpp
    Services/Src/m10_service.cpp
    Services/Src/config_storage.cpp
    Services/Src/fc_link.cpp
    Services/Src/command_handler.cpp
    Services/Src/panic.cpp
    ../libs/src/error_code.cpp
)

add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}.bin
    COMMENT "Generating binary file: ${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}.bin"
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    Drivers/USER/inc
    Core/Inc
    Services/Inc
    ../libs/inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
    -fno-rtti
    -fno-exceptions
    -Wdouble-promotion
    -Werror=switch
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
