# Bill of materials

## Chosen

| Part | Search for | USD | Checked |
|---|---|---|---|
| Flight computer | `STM32F407VGT6 DevEBox` — black board, silkscreen `STM32F4XX_M` | ~$10 | Aug 2026 |
| Bridge | `ESP32-C3 0.42 OLED` — small purple board, ceramic antenna | ~$3 | Aug 2026 |
| Mounting | Double-sided **foam** tape, **1.2 mm or thicker** | ~$2 | Aug 2026 |

**Running subtotal: `TBD(#2)`.** These three are the entire brain of the aircraft — both
microcontrollers, the telemetry radio, and the status display. Almost all of the cost is still
ahead of them: the frame, motors, ESCs and battery are what a build like this actually spends
money on.

!!! tip

    **VG or VE — either is fine.** The DevEBox `STM32F4XX_M` ships with either an
    **STM32F407VGT6** (1 MB flash) or an **STM32F407VET6** (512 KB). They are pin-compatible.
    The reference build happens to use the VGT6 and the extra flash costs essentially nothing,
    but nothing here needs it.

    One detail if you end up with the VE: the linker script declares 1 MB, so it would not
    catch an image that grew past 512 KB. At the current size that is a very long way off.

## Tools

| | |
|---|---|
| Soldering iron | Fine tip. Mostly through-hole and castellated, plus [surface-mount taps](boards.md#bootloader-control-lines) |
| Magnification | Loupe, headset or a phone camera — for the SMD joints |
| Flux | Makes those joints far easier than they look |
| Multimeter | Continuity and DC volts. Non-negotiable for the divider and the SMD pads |
| USB-C cable | Data, not charge-only. The bridge is the only board ever plugged in |
| LiPo charger | 6S capable, with balance leads |
| Helping hands or a vice | Optional, but makes assembly much easier — holds the board still while both your hands are busy |
