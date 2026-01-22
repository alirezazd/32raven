idf_component_register(
    SRCS
        "main.cpp"
        "ctx.cpp"
        "states.cpp"
        "../drivers/led.cpp"
        "../drivers/button.cpp"
        "../drivers/wifi.cpp"
        "../services/tcp_server.cpp"
        "../services/system.cpp"
        "../services/timebase.cpp"
        "../drivers/uart.cpp"
        "../drivers/programmer.cpp"
        "../services/flight_controller.cpp"
        "../services/mavlink.cpp"
        "../drivers/ep2_receiver.cpp"
    INCLUDE_DIRS
        "."
        "../drivers"
        "../services"
        "../../libs/include"
        "../../libs/mavlink/common"

)

target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-address-of-packed-member")
