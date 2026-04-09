find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_program(CLANG_FORMAT_EXECUTABLE NAMES clang-format)
find_program(ESP32_CONFIG_PYTHON3_EXECUTABLE
    NAMES python3
    PATHS /usr/bin /usr/local/bin
    NO_DEFAULT_PATH)

if(NOT ESP32_CONFIG_PYTHON3_EXECUTABLE)
    set(ESP32_CONFIG_PYTHON3_EXECUTABLE ${Python3_EXECUTABLE})
endif()

set(ESP32_KCONFIG "${CMAKE_CURRENT_SOURCE_DIR}/../../config/Kconfig")
set(ESP32_MASTER_CONFIG "${CMAKE_CURRENT_SOURCE_DIR}/../../config/32raven.config")
set(ESP32_CONFIG_GENERATOR
    "${CMAKE_CURRENT_SOURCE_DIR}/../../scripts/generate_esp32_config.py")
set(ESP32_FIRMWARE_VERSION_TOOL
    "${CMAKE_CURRENT_SOURCE_DIR}/../../scripts/generate_firmware_ver.py")
set(ESP32_CONFIG_TEMPLATE
    "${CMAKE_CURRENT_SOURCE_DIR}/../../scripts/templates/esp32_config.hpp.j2")
set(ESP32_LIMITS_TEMPLATE
    "${CMAKE_CURRENT_SOURCE_DIR}/../../scripts/templates/esp32_limits.hpp.j2")
set(ESP32_RUNTIME_CONFIG_HPP "${CMAKE_CURRENT_SOURCE_DIR}/esp32_config.hpp")
set(ESP32_LIMITS_HPP "${CMAKE_CURRENT_SOURCE_DIR}/esp32_limits.hpp")

idf_component_register(
    SRCS
    "main.cpp"
    "states.cpp"
    "../drivers/led.cpp"
    "../drivers/buzzer.cpp"
    "../drivers/button.cpp"
    "../drivers/i2c.cpp"
    "../drivers/ssd1306_panel.cpp"
    "../services/programmer.cpp"
    "../services/ui.cpp"
    "../ui/display_renderer.cpp"
    "../ui/widgets/boot_widget.cpp"
    "../ui/widgets/error_widget.cpp"
    "../ui/widgets/main_ui_widget.cpp"
    "../ui/widgets/matrix_widget.cpp"
    "../ui/widgets/rc_channels_widget.cpp"
    "../ui/widgets/renderer_demo_widget.cpp"
    "../services/wifi.cpp"
    "../services/tone_player.cpp"
    "../services/tcp_server.cpp"
    "../services/system.cpp"
    "../services/timebase.cpp"
    "../drivers/uart.cpp"
    "../services/fc_link.cpp"
    "../main/command_handler.cpp"
    "../services/mavlink.cpp"
    "../services/mavlink_task.cpp"
    "../services/udp_server.cpp"
    "../services/panic.cpp"
    "../../third_party/Adafruit-GFX-Library/Adafruit_GFX.cpp"
    "../../libs/src/error_code.cpp"
    "../../libs/inc/error_code.hpp"

    INCLUDE_DIRS
    "."
    "../drivers"
    "../services"
    "../ui"
    "../ui/adafruit_gfx_compat"
    "../ui/assets"
    "../ui/widgets"
    "../../libs/inc"
    "../../third_party/Adafruit-GFX-Library"
    "../../third_party/mavlink/common"

)

if(CLANG_FORMAT_EXECUTABLE)
    add_custom_command(
        OUTPUT ${ESP32_RUNTIME_CONFIG_HPP} ${ESP32_LIMITS_HPP}
        COMMAND ${ESP32_CONFIG_PYTHON3_EXECUTABLE} ${ESP32_CONFIG_GENERATOR}
        --kconfig ${ESP32_KCONFIG}
        --config ${ESP32_MASTER_CONFIG}
        --runtime-out ${ESP32_RUNTIME_CONFIG_HPP}
        --limits-out ${ESP32_LIMITS_HPP}
        COMMAND ${CLANG_FORMAT_EXECUTABLE} -i
        ${ESP32_RUNTIME_CONFIG_HPP}
        ${ESP32_LIMITS_HPP}
        DEPENDS ${ESP32_KCONFIG} ${ESP32_MASTER_CONFIG}
        ${ESP32_CONFIG_GENERATOR}
        ${ESP32_FIRMWARE_VERSION_TOOL}
        ${ESP32_CONFIG_TEMPLATE}
        ${ESP32_LIMITS_TEMPLATE}
        COMMENT "Generating ESP32 configuration headers"
        VERBATIM)
else()
    add_custom_command(
        OUTPUT ${ESP32_RUNTIME_CONFIG_HPP} ${ESP32_LIMITS_HPP}
        COMMAND ${ESP32_CONFIG_PYTHON3_EXECUTABLE} ${ESP32_CONFIG_GENERATOR}
        --kconfig ${ESP32_KCONFIG}
        --config ${ESP32_MASTER_CONFIG}
        --runtime-out ${ESP32_RUNTIME_CONFIG_HPP}
        --limits-out ${ESP32_LIMITS_HPP}
        DEPENDS ${ESP32_KCONFIG} ${ESP32_MASTER_CONFIG}
        ${ESP32_CONFIG_GENERATOR}
        ${ESP32_FIRMWARE_VERSION_TOOL}
        ${ESP32_CONFIG_TEMPLATE}
        ${ESP32_LIMITS_TEMPLATE}
        COMMENT "Generating ESP32 configuration headers"
        VERBATIM)
endif()

add_custom_target(generate_esp32_config
    DEPENDS ${ESP32_RUNTIME_CONFIG_HPP}
    ${ESP32_LIMITS_HPP})
add_dependencies(${COMPONENT_LIB} generate_esp32_config)

target_compile_definitions(${COMPONENT_LIB} PRIVATE ARDUINO=100)
target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-address-of-packed-member")
