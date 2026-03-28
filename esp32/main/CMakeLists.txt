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

target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-address-of-packed-member")
