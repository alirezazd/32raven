find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_program(CLANG_FORMAT_EXECUTABLE NAMES clang-format)

set(ESP32_KCONFIG "${CMAKE_CURRENT_SOURCE_DIR}/../../config/Kconfig")
set(ESP32_MASTER_CONFIG "${CMAKE_CURRENT_SOURCE_DIR}/../../config/32raven.config")
set(ESP32_CONFIG_GENERATOR
    "${CMAKE_CURRENT_SOURCE_DIR}/../../scripts/generate_esp32_config.py")
set(ESP32_RUNTIME_CONFIG_HPP "${CMAKE_CURRENT_SOURCE_DIR}/esp32_config.hpp")

idf_component_register(
    SRCS
        "main.cpp"
        "ctx.cpp"
        "states.cpp"
        "../drivers/led.cpp"
        "../drivers/buzzer.cpp"
        "../drivers/button.cpp"
        "../drivers/wifi.cpp"
        "../services/tone_player.cpp"
        "../services/tcp_server.cpp"
        "../services/system.cpp"
        "../services/timebase.cpp"
        "../drivers/uart.cpp"
        "../drivers/programmer.cpp"
        "../services/fc_link.cpp"
        "../main/command_handler.cpp"
        "../services/mavlink.cpp"
        "../services/mavlink_tx_task.cpp"
        "../services/panic.cpp"
        "../../libs/src/error_code.cpp"
        "../../libs/inc/error_code.hpp"
        
    INCLUDE_DIRS
        "."
        "../drivers"
        "../services"
        "../../libs/inc"
        "../../third_party/mavlink/common"

)

if(CLANG_FORMAT_EXECUTABLE)
    add_custom_command(
        OUTPUT ${ESP32_RUNTIME_CONFIG_HPP}
        COMMAND ${Python3_EXECUTABLE} ${ESP32_CONFIG_GENERATOR}
                --kconfig ${ESP32_KCONFIG}
                --config ${ESP32_MASTER_CONFIG}
                --runtime-out ${ESP32_RUNTIME_CONFIG_HPP}
        COMMAND ${CLANG_FORMAT_EXECUTABLE} -i ${ESP32_RUNTIME_CONFIG_HPP}
        DEPENDS ${ESP32_KCONFIG} ${ESP32_MASTER_CONFIG} ${ESP32_CONFIG_GENERATOR}
        COMMENT "Generating ESP32 configuration header"
        VERBATIM)
else()
    add_custom_command(
        OUTPUT ${ESP32_RUNTIME_CONFIG_HPP}
        COMMAND ${Python3_EXECUTABLE} ${ESP32_CONFIG_GENERATOR}
                --kconfig ${ESP32_KCONFIG}
                --config ${ESP32_MASTER_CONFIG}
                --runtime-out ${ESP32_RUNTIME_CONFIG_HPP}
        DEPENDS ${ESP32_KCONFIG} ${ESP32_MASTER_CONFIG} ${ESP32_CONFIG_GENERATOR}
        COMMENT "Generating ESP32 configuration header"
        VERBATIM)
endif()

add_custom_target(generate_esp32_config DEPENDS ${ESP32_RUNTIME_CONFIG_HPP})
add_dependencies(${COMPONENT_LIB} generate_esp32_config)

target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-address-of-packed-member")
