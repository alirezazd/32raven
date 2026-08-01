# 32Raven Handbook

**32Raven** is a bare-metal flight-control stack split across two MCUs: an **STM32F407**
running the flight-critical loop and an **ESP32-C3** carrying the connectivity surface.
They share nothing but a versioned wire protocol.

This handbook is the practical layer — how to **build one**, wire it, flash it, bring it up,
and fly it. The repository README covers the architecture; these pages cover the hardware in
front of you.

![The assembled 32Raven prototype](assets/prototype-placeholder.svg)

## At a glance

<div class="grid cards" markdown>

-   :material-quadcopter:{ .lg .middle } **Airframe**

    ---

    `TBD(#3)`

-   :material-chip:{ .lg .middle } **Flight controller**

    ---

    STM32F407 — control loop, IMU, GPS, RC, mixer

-   :material-wifi:{ .lg .middle } **Connectivity**

    ---

    ESP32-C3 — MAVLink/CRSF, WiFi, OTA, on-device UI

-   :material-battery-high:{ .lg .middle } **Power**

    ---

    6S pack — `TBD(#3)` for capacity and all-up weight

-   :material-cash-multiple:{ .lg .middle } **Parts cost**

    ---

    `TBD(#2)`

-   :material-timer-outline:{ .lg .middle } **Build time**

    ---

    `TBD(#2)`

</div>

Anything marked `TBD(#N)` is an open item on the [roadmap](roadmap.md) — the number is its
entry there. The build fails if a marker points at an item that does not exist.

## What you need to be able to do

- **Solder** a 0.1-inch header and tin stranded wire. Everything is through-hole or
  module-level — there is no PCB to fabricate and no SMD rework.
- **Use a multimeter** for continuity and DC voltage. One measurement in this guide is
  load-bearing: the battery divider, before it ever reaches an ADC pin.
- **Flash over USB and read a serial console.**

No prior flight-controller experience is assumed, and no oscilloscope is required.

!!! warning "Status: active development"

    Interfaces, configs, and pin assignments change quickly. Every pin and tunable quoted
    here is checked against `config/Kconfig` on each build, but a page can still describe a
    stage that has moved on. Build from a tagged release if you want stability.

!!! danger "This aircraft can injure you"

    A 6S quadcopter with props on is a serious hazard. Nothing in this handbook is a
    substitute for your own judgement: props stay **off** the motors until the bench-test
    stage explicitly says otherwise, and the battery stays disconnected while you wire.

## Where this handbook stands

It is being written **right now**, alongside the first from-scratch build of the prototype —
the aircraft and its documentation are being assembled together. [Wiring](build/wiring.md) is
complete and every pin in it is checked against the reference config on each build. The
remaining stages are stubs, filled in as that aircraft reaches them.

**[Start with the build guide →](build/index.md)**
