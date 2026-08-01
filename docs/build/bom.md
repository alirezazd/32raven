# Bill of materials

The parts list for the reference build. **This page is filled in as the prototype is
assembled** — entries marked `TBD` are not yet decided or not yet priced. Confirmed parts
carry the detail that matters for wiring them, not just a product name.

!!! note "Prices"

    Prices are **USD from AliExpress**, with the month they were checked. That is where both
    dev boards come from and what most people building this will pay. Where Amazon differs
    enough to matter it is noted separately — typically 2–3× for speed and returnability.

    Deep product links are deliberately omitted. They rot within months, and a dead link in a
    parts list is worse than none. Search terms are given instead.

## Confirmed

| Part | What to search for | Price | Checked |
|---|---|---|---|
| Flight controller board | `STM32F407VET6 DevEBox` — black board, "STM32F4XX_M" | `TBD` | `TBD` |
| Bridge board | `ESP32-C3 0.42 OLED` — the small pink board | `TBD` | `TBD` |

### STM32F407VET6 — DevEBox "STM32F4XX_M" v2.0

The flight controller. Sold as a generic dev board; the reference build uses it unmodified.

| | |
|---|---|
| MCU | STM32F407VET6 — Cortex-M4 @ 168 MHz, 512 KiB flash, 192 KiB SRAM |
| HSE crystal | 8 MHz |
| SPI flash | W25Q16BV, 2 MiB — **populated from the factory** |
| Debug | SWD header: `PA13` SWDIO, `PA14` SWCLK, plus `BOOT0` |
| Regulator | AMS1117, 3.3 V at 1 A |

Three things about this board are load-bearing for the firmware:

**The 8 MHz crystal is why the checked-in clock configuration works.** `CONFIG_STM32_SYSTEM_PLL_M`
is 8 and `PLL_N` is 336: 8 MHz ÷ 8 = 1 MHz into the PLL, × 336 = 336 MHz VCO, ÷ 2 = 168 MHz.
A board with a different crystal needs `PLL_M` changed to keep the VCO input at 1 MHz, or the
core runs at the wrong speed and every timing in the firmware is wrong with it.

**The SPI flash is already wired to the pins the firmware expects.** `CS` `PA15`, `CLK` `PB3`,
`DI` `PB5`, `DO` `PB4` — exactly the SPI1 assignment in [the wiring reference](wiring.md). No
wiring needed; it is on the board.

**The onboard LED and button are the ones the firmware drives.** LED `D2` on `PA1` is wired in
sink mode, so it is **active low**. Button `K1` on `PA0` is **active high**.

!!! danger "The 5 V pins have no protection"

    The board's `+5V` pins connect **directly** to the USB connector's `+5V`, with no diode
    and no current limiting. Powering the board from a BEC while USB is plugged in shorts your
    BEC against the host's USB rail.

    During bring-up, power from **USB only**. Once a BEC is fitted, unplug USB before
    connecting the battery.

!!! warning "Leave the LCD header (J4) empty"

    `J4` carries `SDI` `PB15`, `SCL` `PB13`, `CS` `PB12`, `SDO` `PB14` — the same SPI2 bus the
    IMU uses. The IMU has its own chip select (`PA4`), so an empty header is harmless, but
    anything plugged into `J4` is sharing a bus with the sensor the aircraft flies on.

The microSD slot (`PC12`, `PD2`, `PC8`–`PC11`) is unused by this build. It is also where the
bridge board gets taped down, which is why none of those pins appear in the wiring reference.

### ESP32-C3 0.42" OLED

The connectivity bridge, with the status display already on it. About 25 × 20 mm.

| | |
|---|---|
| MCU | ESP32-C3, 4 MB flash, WiFi + BLE 5.0 |
| Display | SSD1306 at I²C address `0x3C`, on `GPIO5` (SDA) / `GPIO6` (SCL) |
| Panel | 72 × 40 visible, driven by a 128 × 64 controller — column offset 28 |
| LED | `GPIO8`, plain LED, **inverted** — HIGH is off |
| Buttons | `BOOT` on `GPIO9`, `RESET` on `EN` |
| USB | USB-C, native — 5 V passes through a 1N5819 diode |

The firmware's panel geometry (`kDisplayPanelWidth = 72`, `kDisplayPanelHeight = 40`,
`kDisplayPanelControllerWidth = 128`, `kDisplayPanelColumnOffset = 28`) is written for this
exact panel, and `tools/png_conv.py` defaults to 72 × 40 for the same reason. A different
ESP32-C3 without this display is a firmware change, not a substitution.

!!! danger "The vendor pinout diagram is wrong about I²C"

    Most listings ship a pinout image labelling `GPIO8` as **I2C SDA** and `GPIO9` as
    **I2C SCL**. They are not. Those are the generic ESP32-C3 defaults printed on a template,
    not this board's wiring.

    On this board `GPIO8` is the onboard LED and `GPIO9` is the BOOT button. The OLED is on
    `GPIO5` and `GPIO6`, and because the I²C peripheral is routed to the display, there is no
    second I²C bus available for external devices.

!!! warning "RX and TX labels are inverted relative to this firmware"

    The board labels `GPIO20` as **RX** and `GPIO21` as **TX**, following the chip's default
    UART0 assignment. This firmware drives them the other way:
    `CONFIG_ESP32_PINMAP_TELEM_UART_TX_GPIO_NUM` is 20 and `..._RX_GPIO_NUM` is 21.

    That is legal — the C3 routes any UART to any pin through the GPIO matrix — but it means
    **the silkscreen will mislead you**. Wire telemetry by GPIO number, not by the printed
    label. Verify on the bench before trusting it.

#### Which pins are already used

| Pin | Used by | Free to wire? |
|---|---|---|
| `GPIO5`, `GPIO6` | Onboard OLED (I²C) | No — internal |
| `GPIO8` | Onboard LED | No — internal |
| `GPIO9` | BOOT button | No — internal |
| `GPIO12`–`GPIO17` | SPI flash | No — reserved by the chip |
| `GPIO18`, `GPIO19` | USB D− / D+ | No — reserved by the chip |
| `GPIO0`, `GPIO1` | STM32 `BOOT0` and `NRST` | Yes — wired in this build |
| `GPIO3`, `GPIO4` | FcLink to the STM32 | Yes — wired in this build |
| `GPIO10` | Buzzer | Yes — wired in this build |
| `GPIO20`, `GPIO21` | Telemetry UART | Yes — wired in this build |
| `GPIO2`, `GPIO7` | — | Yes — unused, available |

Every pin this build needs is broken out. Nothing has to be scraped off a pad.

### Mounting

The bridge board is stuck to the top of the flight controller's microSD slot with
double-sided tape. `TBD` — tape type and thickness, and a photograph.

## Still to specify

Everything below is required by [the wiring reference](wiring.md) but not yet chosen. Each
becomes a row in the table above as it is decided.

| Part | Constraint it has to satisfy | Status |
|---|---|---|
| IMU module | ICM42688P on SPI2; needs `CS` and an interrupt line | `TBD` |
| GPS module | u-blox M10 on USART2, 3.3 V logic | `TBD` |
| RC receiver | CRSF (ELRS) on USART6, 3.3 V logic | `TBD` |
| ESCs | AM32-capable, DShot, telemetry output for `PB11` | `TBD` |
| Motors | Sized for the frame and a 6S pack | `TBD` |
| Propellers | To match motors and frame | `TBD` |
| Frame | Sets the airframe class and the whole weight budget | `TBD` |
| Battery | 6S — capacity and C rating | `TBD` |
| Voltage divider | Scales pack voltage to ≤ 3.3 V for `PC0` | `TBD` |
| Current sensor | Analogue output to `PC1` | `TBD` |
| Buzzer | Driven from `GPIO10` — active or passive changes the firmware | `TBD` |
| Power distribution | 6S in, BEC rails out | `TBD` |
| Connectors | XT60 or XT30, and motor bullets | `TBD` |
| Wire | Gauges for power and signal | `TBD` |
| Hardware | Standoffs, screws, zip ties, heat-shrink | `TBD` |

!!! danger "The divider is the one that can destroy the board"

    A 6S pack is roughly 25 V fully charged. `PC0` is a 3.3 V ADC input. The divider ratio is
    not a detail to get approximately right — get it wrong and the STM32 is gone the first
    time you connect a battery. It gets measured with a meter before it is ever connected to
    the pin, and that check has its own step in [the wiring reference](wiring.md).
