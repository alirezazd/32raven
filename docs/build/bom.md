# Bill of materials

## Chosen

Prices were last checked in **August 2026**, and are what the parts cost then — not quotes.
Shipping is not included, and on an order this small it is a real fraction of the total.

### Airframe

| Part | Search for | Used for | USD |
|---|---|---|---|
| Frame | `LX350 PRO frame 350` — 4-axis quadcopter frame, carbon fibre, with landing gear | The airframe everything else mounts to | ~$57 |

### Propulsion

| Part | Search for | Used for | USD |
|---|---|---|---|
| Motors | `EMAX ECO II 3115 800KV`, set of 4 | Thrust | ~$145 |
| ESC | `Holybro Tekko32 F4 4in1 65A` — AM32, 4–6S, 30.5 × 30.5 mm | Drives the four motors over DShot, and reports telemetry back | ~$125 |
| Propellers | `Gemfan 9045 carbon nylon`, 4 pairs | 9 inch, 4.5 pitch — one spare set | ~$12 |
| Prop balancer — *optional* | `magnetic suspension prop balancer`, carbon fibre | Balancing props before they go on, so the IMU sees less of them | ~$4.59 |

!!! note

    **The motors are what this build happens to have, not a recommendation.** A 3115 stator on
    a 350 mm frame is a large motor for the airframe — the pairing suits a slow, efficient
    cruiser on big props rather than anything agile, and it is heavier than the frame needs.
    It is recorded here because it is what the reference build flies, and the numbers on this
    page are only honest if they match the aircraft that exists.

### Power

| Part | Search for | Used for | USD |
|---|---|---|---|
| Battery | `6S 6000mAh 100C LiPo` | The flight pack — 22.2 V nominal, 25.2 V charged | ~$68 |
| Buck converter | `DC-DC buck module`, 5–30 V in, 5 V out | Drops pack voltage to the 5 V rail both boards share | ~$2 |
| IMU regulator | `LT3045 3.3V module`, 1 pc | Low-noise 3.3 V for the IMU, `TBD(#4)` | ~$15 |
| Battery lead | `12 AWG silicone wire`, plus an `XT60` or `XT90` pair | Pack to ESC — the one run that carries full current | `TBD(#2)` |
| Heat-shrink | Assorted, including a size that fits the battery lead | Insulating the pack connector joints | `TBD(#2)` |

### Boards and sensors

| Part | Search for | Used for | USD |
|---|---|---|---|
| Flight computer | `STM32F407VGT6 DevEBox` — black board, silkscreen `STM32F4XX_M` | Runs the flight code | ~$10 |
| Bridge | `ESP32-C3 0.42 OLED` — small purple board, ceramic antenna | Telemetry, display, and flashing the flight computer | ~$3 |
| IMU | `ICM-42688-P` — breakout board, SPI | Attitude and rate — the sensor it flies on | ~$20 |
| GPS | `u-blox M10` module — UART, patch antenna | Position, velocity and time | ~$21.50 |
| RC receiver | `HappyModel EP1 ELRS 2.4G` — or any ExpressLRS receiver | Pilot control, and telemetry back to the transmitter | ~$20 |
| TCXO — *optional* | `DSK321STD 32.768K TCXO` — 3.2 × 2.5 mm, ±5 ppm | Drives the IMU's [external crystal](peripherals.md#external-crystal-optional) | ~$3 |

!!! tip

    **VG or VE — either is fine.** The DevEBox `STM32F4XX_M` ships with either an
    **STM32F407VGT6** (1 MB flash) or an **STM32F407VET6** (512 KB). They are pin-compatible.
    The reference build happens to use the VGT6 and the extra flash costs essentially nothing,
    but nothing here needs it.

    One detail if you end up with the VE: the linker script declares 1 MB, so it would not
    catch an image that grew past 512 KB. At the current size that is a very long way off.

### Mounting and adhesives

| Part | Search for | Used for | USD |
|---|---|---|---|
| Foam tape | Double-sided **foam** tape, **1.2 mm or thicker** | Holds the bridge on the flight computer, and beds the IMU, GPS and receiver down. The thickness is a spacer for the stack, not a preference | ~$2 |
| Battery strap | Hook-and-loop strap, sized to the pack | Holds the pack to the frame | `TBD(#2)` |
| Silicone adhesive — *optional* | Neutral-cure silicone | Beds the TCXO down, and comes off again | ~$4 |
| Kapton tape — *optional* | Polyimide film tape | Holds the TCXO's runs flat | ~$1 |

### Wire

| Part | Search for | Used for | USD |
|---|---|---|---|
| Hookup wire | `30 AWG silicone wire kit` — 10 colours, ~300 cm each | Every signal run between the boards and the sensors | ~$10 |
| Enamelled wire — *optional* | `0.25 mm enamelled copper wire` — sold by the spool, 210 m being the smallest easily found | The TCXO's three runs — stiff enough to hold its own shape | ~$8 |

**Running subtotal: ~$531.09**, or ~$510.50 without the optional rows — airframe, motors, ESCs,
props, pack, and the whole brain of it. The three rows still marked `TBD(#2)` are all small
hardware, and none of them are in that figure yet.

## Tools

| | |
|---|---|
| Soldering iron | Fine tip. Mostly through-hole and castellated, plus [surface-mount taps](boards.md#bootloader-control-lines) |
| Magnification | Loupe, headset or a phone camera — for the SMD joints |
| Flux | Makes those joints far easier than they look |
| Multimeter | Continuity and DC volts. Non-negotiable for the divider and the SMD pads |
| USB-C cable | The bridge is the only board ever plugged in |
| LiPo charger | 6S capable, with balance leads |
| Helping hands or a vice | Optional, but makes assembly much easier — holds the board still while both your hands are busy |
