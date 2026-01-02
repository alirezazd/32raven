idf_component_register(
    SRCS
        "main.cpp"
        "states.cpp"
        "system.cpp"
        "../drivers/led.cpp"
        "../drivers/button.cpp"
        "../drivers/wifi.cpp"
        "../utils/http_server.cpp"
        "../utils/timebase.cpp"
        "../drivers/uart.cpp"
    INCLUDE_DIRS
        "."
        "../drivers"
        "../utils"

)
