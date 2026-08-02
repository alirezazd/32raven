# Flashing the firmware

`TBD(#4)` — toolchain, `menuconfig`, and bringing up both MCUs.

Only one board is ever plugged into a computer. The bridge is flashed over its own USB, and
from then on it programs the flight computer itself, over the air.

It does that by holding `BOOT0` high and pulsing `NRST`, so the STM32 starts in its built-in
ROM bootloader instead of the application. That bootloader listens on USART1 — the same
`PA9`/`PA10` pair FcLink already uses, which is why the link sits on those pins. The image
then goes down the wire FcLink normally carries telemetry on: `make flash-wifi-stm32`.

Both lines are wired in [Bootloader control lines](build/boards.md#bootloader-control-lines).

!!! warning

    **Plug in one USB at a time — never both.** With the 5 V rails joined, plugging both
    boards into USB at once connects two host USB rails to each other through the flight
    computer's `+5V` pins, which have no diode and no current limiting.

    In practice this build never asks you to: **the flight computer's USB port is not used at
    all.**
