# Sensors and peripherals

Everything that hangs off the core once the two boards are joined — the IMU, the GPS, the RC
receiver, the ESC telemetry line, and the buzzer. Pins for all of them are in
[the wiring reference](wiring.md); this page covers the physical side.

Most of it is `TBD(#4)`, written as each stage is reached.

## IMU — ICM42688P

!!! info

    **Under construction.** Being written as the sensor goes on. Treat everything below as
    unverified until this box goes away.

The one sensor the aircraft cannot fly without, and the one where how you mount it matters
more than how you wire it. It sits **directly on top of the STM32 package** — the centre of
the board, and about as close to the rotational axes as the stack allows.

Pins are in [the wiring reference](wiring.md#imu-icm42688p-spi2): SPI2 plus a data-ready
interrupt on `PB10`. That interrupt is what clocks the control loop, so an IMU that is wired
but has no `INT` gives you a flight controller that never runs a single iteration.

**Orientation.** `TBD(#4)` — which way the chip's X axis points, and how that is squared with
the front of the airframe.

**Vibration isolation.** `TBD(#4)` — what, if anything, goes between the chip and the sensor.

**Adhesive.** `TBD(#4)` — what holds it down, and how to get it off again without lifting the
STM32's lid.

<!-- TODO(build): photo of the IMU mounted on the STM32, axis arrow visible. -->

## Buzzer

`TBD(#6)` — the buzzer runs from `GPIO10` on the bridge. It is already fitted in the
photographs on [The brain](boards.md); wiring it is a stage of its own.
