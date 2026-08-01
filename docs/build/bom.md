# Bill of materials

What to buy. Detail on the two dev boards — pinouts, what is already wired, what will bite you
— lives on [The two boards](boards.md); this page is the shopping list.

**Filled in as the prototype is built.** Rows marked `TBD` are not yet chosen or not yet
priced.

!!! note "About the prices"

    Prices are **USD**, single unit, from AliExpress, with the month checked. That is where
    both boards come from and what most people building this will pay. Shipping is excluded —
    it varies more than the parts do.

    Deep product links are deliberately omitted: they rot within months, and a dead link in a
    parts list is worse than no link. Search terms are given instead.

## Chosen

| Part | Search for | USD | Checked |
|---|---|---|---|
| Flight controller | `STM32F407VGT6 DevEBox` — black board, silkscreen `STM32F4XX_M` | ~$10 | Aug 2026 |
| Bridge + display | `ESP32-C3 0.42 OLED` — small purple board, ceramic antenna | ~$3 | Aug 2026 |
| Mounting | Double-sided **foam** tape — thickness matters, see [mounting](boards.md#mounting-the-stack) | ~$2 | Aug 2026 |

**Running subtotal: ~$15.** That is the entire brain of the aircraft — both microcontrollers,
the telemetry radio, and the status display. Almost all of the cost is still ahead of it: the
frame, motors, ESCs and battery are what a build like this actually spends money on.

!!! tip "VG or VE — either is fine"

    The board ships with either an **STM32F407VGT6** (1 MB flash) or an **STM32F407VET6**
    (512 KB). They are pin-compatible, and the firmware uses about 62 KB — so both have room
    to spare many times over. The reference build happens to use the VGT6 and the extra flash
    costs essentially nothing, but nothing here needs it.

    One detail if you end up with the VE: the linker script declares 1 MB, so it would not
    catch an image that grew past 512 KB. At the current size that is a very long way off.

    Sellers mix the two up in listing titles, so the chip marking is the only reliable way to
    know which one turned up.

## Still to choose

Everything below is required by [the wiring reference](wiring.md) but not yet decided. Each
becomes a row above as it is settled.

| Part | What it has to satisfy | |
|---|---|---|
| IMU | ICM42688P on SPI2, with `CS` and an interrupt line | `TBD` |
| GPS | u-blox M10 on USART2, 3.3 V logic | `TBD` |
| RC receiver | CRSF / ELRS on USART6, 3.3 V logic | `TBD` |
| ESCs | AM32-capable, DShot, telemetry output for `PB11` | `TBD` |
| Motors | Sized for the frame and a 6S pack | `TBD` |
| Propellers | Matched to motors and frame | `TBD` |
| Frame | Sets the airframe class and the weight budget | `TBD` |
| Battery | 6S — capacity and C rating | `TBD` |
| Voltage divider | Scales pack voltage to ≤ 3.3 V at `PC0` | `TBD` |
| Current sensor | Analogue output to `PC1` | `TBD` |
| Buzzer | Driven from `GPIO10` — active vs passive changes the firmware | `TBD` |
| Power distribution | 6S in, BEC rails out | `TBD` |
| Connectors | XT60 or XT30, plus motor bullets | `TBD` |
| Wire | Gauges for power and for signal | `TBD` |
| Hardware | Standoffs, screws, zip ties, heat-shrink | `TBD` |

!!! danger "The divider is the part that can destroy the board"

    A 6S pack is roughly 25 V charged. `PC0` is a 3.3 V ADC input. The divider ratio is not a
    detail to get approximately right — get it wrong and the STM32 is gone the first time a
    battery is connected. It gets measured with a meter before it is ever connected to the
    pin, and that check has its own step in [the wiring reference](wiring.md).

## Tools

| | |
|---|---|
| Soldering iron | Fine tip. Everything is through-hole or castellated — no SMD rework |
| Multimeter | Continuity and DC volts. Non-negotiable for the divider |
| USB-C and micro-USB cables | Data, not charge-only. One of each — the boards differ |
| LiPo charger | 6S capable, with balance leads |
| Helping hands or a vice | `TBD` |

No oscilloscope, no hot-air station, no PCB fabrication.
