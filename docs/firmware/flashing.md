# Flashing the firmware

Only one board is ever plugged into a computer. The bridge is flashed over its own USB, and
from then on it programs the flight computer itself, over the air.

It does that by holding `BOOT0` high and pulsing `NRST`, so the STM32 starts in its built-in
ROM bootloader instead of the application. That bootloader listens on USART1 — the same
`PA9`/`PA10` pair FcLink already uses, which is why the link sits on those pins. The image
then goes down the wire FcLink normally carries telemetry on.

Both lines are wired in [Bootloader control lines](../build/boards.md#bootloader-control-lines).

!!! warning

    **Plug in one USB at a time — never both.** With the 5 V rails joined, plugging both
    boards into USB at once connects two host USB rails to each other through the flight
    computer's `+5V` pins, which have no diode and no current limiting.

    In practice this build never asks you to: **the flight computer's USB port is not used at
    all.**

## The bridge, over USB

```bash
make flash-esp32
```

Builds the ESP32 firmware and flashes it over the USB-C port. The port and baud resolve
from the USB descriptor; to pin them, see
[local overrides](setup.md#local-overrides-user_configcmake). To watch it boot, use
`make monitor-esp32` — or `make flash-monitor-esp32` for both in one command.

## The bridge, over WiFi

Once the bridge is running, it takes its own updates over the air — no cable at all:

```bash
make flash-wifi-esp32
```

## The flight computer, through the bridge

```bash
make flash-wifi-stm32
```

The freshly built image is uploaded over WiFi to the bridge, which drops the STM32 into its
ROM bootloader and forwards the image over the inter-MCU UART.

Both WiFi targets need your computer on the bridge's network — it broadcasts its own access
point (SSID `32Raven` in the reference config, `CONFIG_ESP32_WIFI_AP_SSID`), where its
address is the default `192.168.4.1`. Reaching it some other way, pass the address
explicitly: `make flash-wifi-stm32 ESP_IP=<address>`.
