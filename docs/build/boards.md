# The two boards

Everything flies on two off-the-shelf dev boards. Neither is modified — no cut traces, no
reflow, no piggybacked chips. This page is the reference for what is already on them, which
pins are therefore spoken for, and the three ways they will bite you.

## Specifications

<div class="grid" markdown>

!!! abstract "Flight controller — DevEBox `STM32F4XX_M`"

    | | |
    |---|---|
    | MCU | STM32F407VGT6, Cortex-M4F |
    | Clock | 168 MHz (8 MHz HSE × PLL) |
    | Memory | 1 MB flash, 192 KB SRAM — the VET6 variant has 512 KB |
    | Storage | W25Q16 — 2 MB SPI flash, fitted |
    | Debug | SWD, 5-pin header |
    | Power | AMS1117, 3.3 V @ 1 A |
    | Size | 40.89 × 68.59 mm |
    | Mounting | 3 mm holes, 34.79 × 62.48 mm centres |

!!! abstract "Bridge — ESP32-C3 0.42″ OLED"

    | | |
    |---|---|
    | MCU | ESP32-C3, RISC-V single core |
    | Clock | 160 MHz |
    | Memory | 4 MB flash |
    | Radio | WiFi 2.4 GHz b/g/n, BLE 5.0 |
    | Display | 0.42″ SSD1306, 72 × 40 visible |
    | USB | USB-C, native (no UART bridge chip) |
    | Size | 24.8 × 20.45 mm |
    | GPIO | 13 broken out, 6 free after this build |

</div>

The flight controller uses about **6 % of its flash** and **9 % of its RAM** with the current
firmware, so there is room to grow. The bridge is the tighter of the two at roughly half its
flash budget, because it carries the WiFi stack, MAVLink, and the display.

## Flight controller — DevEBox STM32F4XX_M

![DevEBox STM32F4XX_M pin map and board dimensions](../assets/pinout-stm32f407-devebox.webp)

/// caption
Board dimensions and pin map. Drawing by the board vendor (mcudev), hosted by
[stm32-base.org](https://stm32-base.org/boards/STM32F407VET6-STM32F4XX-M).
///

### What is already wired

Three things on this board are not accidents of choice — the firmware expects them:

**The SPI flash is on the pins the firmware uses.** `CS` `PA15`, `CLK` `PB3`, `DI` `PB5`,
`DO` `PB4` — exactly the SPI1 assignment in [the wiring reference](wiring.md#onboard-flash-spi1).
The W25Q16 is fitted from the factory. Nothing to wire.

**The LED and button are the ones the firmware drives.** `D2` on `PA1` is wired in sink mode,
so it is **active low**. `K1` on `PA0` is **active high**.

**The 8 MHz crystal is why the clock configuration works.** `CONFIG_STM32_SYSTEM_PLL_M` is 8
and `PLL_N` is 336: 8 MHz ÷ 8 = 1 MHz into the PLL, × 336 = 336 MHz, ÷ 2 = 168 MHz exactly. A
board with a different crystal needs `PLL_M` changed to keep the PLL input at 1 MHz, or the
core runs at the wrong speed and every timing in the firmware moves with it.

!!! danger "The 5 V pins have no protection"

    The `+5V` pins connect **directly** to the USB connector's `+5V` — no diode, no current
    limiting. Powering the board from a BEC while USB is plugged in shorts the BEC against
    your computer's USB rail.

    Bring-up runs on **USB only**. Once a BEC is fitted, unplug USB before connecting the
    battery.

!!! warning "Leave the TFT/OLED header (J4) empty"

    `J4` carries `SDI` `PB15`, `SCL` `PB13`, `CS` `PB12`, `SDO` `PB14` — the same SPI2 bus the
    IMU sits on. The IMU has its own chip select (`PA4`), so an empty header is harmless. But
    anything plugged into `J4` shares a bus with the sensor the aircraft flies on.

The microSD slot (`PC12`, `PD2`, `PC8`–`PC11`) is unused, which is convenient: it is flat, it
is in the middle of the board, and it is where the bridge gets mounted.

## Bridge — ESP32-C3 0.42″ OLED

![ESP32-C3 0.42 inch OLED board pinout](../assets/pinout-esp32c3-oled.webp)

/// caption
Vendor pinout for the bridge board, from the
[Fritzing forum part thread](https://forum.fritzing.org/t/esp32-c3-oled-0-42-mini-board-part/25830).
**Its I²C labels are wrong** — see below.
///

The firmware's panel geometry is written for this exact display: `kDisplayPanelWidth = 72`,
`kDisplayPanelHeight = 40`, `kDisplayPanelControllerWidth = 128`, `kDisplayPanelColumnOffset = 28`.
`tools/png_conv.py` defaults to 72 × 40 for the same reason. A different ESP32-C3 without this
panel is a firmware change, not a substitution.

!!! danger "The vendor pinout above is wrong about I²C"

    It labels `GPIO8` as **I2C SDA** and `GPIO9` as **I2C SCL**. They are not. Those are the
    generic ESP32-C3 defaults printed on a template, not this board's wiring.

    On this board `GPIO8` is the onboard LED and `GPIO9` is the BOOT button. The display is on
    `GPIO5` (SDA) and `GPIO6` (SCL) at address `0x3C`. Because the I²C peripheral is routed to
    the panel, **there is no second I²C bus** for external sensors.

!!! warning "RX and TX are labelled backwards for this firmware"

    The silkscreen marks `GPIO20` as **RX** and `GPIO21` as **TX**, following the chip's
    default UART0 assignment. This firmware drives them the other way round: `GPIO20` is
    telemetry **TX**, `GPIO21` is **RX**.

    The C3 routes any UART to any pin through its GPIO matrix, so this is legal and works. It
    also means wiring by the printed label gives a dead link with no error and nothing in the
    logs. Wire by GPIO number.

### Pin budget

| Pin | Used by | Yours to wire? |
|---|---|---|
| `GPIO5`, `GPIO6` | Onboard display (I²C) | No — internal |
| `GPIO8` | Onboard LED | No — internal |
| `GPIO9` | BOOT button | No — internal |
| `GPIO12`–`GPIO17` | SPI flash | No — reserved by the chip |
| `GPIO18`, `GPIO19` | USB D− / D+ | No — reserved by the chip |
| `GPIO0`, `GPIO1` | STM32 `BOOT0` and `NRST` | Yes — used by this build |
| `GPIO3`, `GPIO4` | FcLink to the flight controller | Yes — used by this build |
| `GPIO10` | Buzzer | Yes — used by this build |
| `GPIO20`, `GPIO21` | Telemetry UART | Yes — used by this build |
| `GPIO2`, `GPIO7` | — | Free |

Every signal this build needs is on a header. Nothing has to be scraped off a pad.

## Mounting the stack

The bridge sits on top of the flight controller, over the unused microSD slot, on double-sided
tape. It is the first physical step of the build.

![The bridge board mounted on top of the flight controller, over the microSD slot](../assets/board-stack-mounted.webp)

/// caption
The stack. The bridge's USB-C faces off the edge so both boards stay flashable, and the
castellated pads on both rows are left clear. The flight controller's 8.000 MHz crystal (`Y2`)
and the `STM32F407VG` marking are both visible here.

**Ignore the wires already on the bridge** — they are the buzzer, which runs from `GPIO10`
(see [ESP32-C3 peripherals](wiring.md#esp32-c3-peripherals)). Nothing needs to be wired at the
mounting stage.
///

!!! warning "The tape is a spacer, not just adhesive"

    The underside of the bridge has exposed pads and via tails on it. The microSD slot it
    mounts over is a **metal shell**. Pressed together, they short.

    This is why the tape is specified as **foam** rather than the thin transfer kind: it is
    doing two jobs, and holding the two boards apart is the more important one. Thin
    double-sided tape will stick perfectly well and leave almost nothing between a live board
    and a grounded can.

    Before power goes anywhere near the stack, look along the seam and confirm nothing on the
    bridge's underside is touching the slot.

Two more things to get right while the tape is still repositionable:

- **Keep the tape off the castellations.** Both header rows have to stay solderable from
  above, and the pads run right to the board edge.
- **Leave the USB-C connector clear.** Both boards get flashed over USB during bring-up, and
  the bridge's connector faces off the edge of the stack.

`TBD` — the measured tape thickness, which is the number that matters here, and the final
orientation once the frame exists.

## Wiring the core

Five wires join the two boards. Nothing else is connected at this stage — no sensors, no
power distribution, no battery.

The tables below are the same ones on [the wiring reference](wiring.md); they are pulled in
from that page rather than copied, so the two cannot drift apart.

### The link

--8<-- "docs/build/wiring.md:fclink"

### Reflashing lines

--8<-- "docs/build/wiring.md:programming"

`BOOT0` is on the `J1` header next to SWD.

### Order of work

1. **Ground first.** Everything else references it.
2. **FcLink, crossed.** `GPIO4` to `PA10`, `GPIO3` to `PA9`. Twist the pair with ground even
   on the bench — the habit costs nothing and 921600 baud will punish you later.
3. **`BOOT0` and `NRST`.** These only matter for over-the-air STM32 updates, but they are far
   easier to solder now than once the stack is in a frame.

!!! warning "No power rail between the boards yet"

    On the bench, run each board from its own USB. Do **not** wire 5 V between them: the
    flight controller's `+5V` pins go straight to its USB connector with no protection, so a
    5 V link plus two plugged-in USB ports ties two host rails together.

    How the boards are powered in flight belongs to the power distribution stage.

`TBD` — photographs of the finished core wiring.
