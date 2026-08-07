# 32Raven — Roadmap

The single source of truth for **32Raven firmware and handbook work**. Every `TBD(#N)` marker
in the handbook points to an item here, and `scripts/lint/check_docs.py` fails the build if it
points at an item that does not exist — so a placeholder cannot be quietly forgotten.

**Scope — this repository.** Firmware and handbook work only.

**Priority legend**

- 🎯 **CRITICAL** — blocks the prototype build, blocks the handbook being usable by someone
  who is not the author, or leaves the firmware in a shape that other queued work has to
  build around.
- 🟢 **SUPPORTING** — real work; do it when the area is being touched anyway.
- 🧊 **DEFERRED** — real work, parked deliberately.

---

## 1. Handbook

The build guide is being written alongside the first from-scratch prototype assembly. These
are the gaps that assembly is expected to close.

### #1 — Prototype photograph — 🎯 CRITICAL

`docs/assets/prototype-placeholder.svg` is a drawn placeholder. Replace it with a photograph
of the assembled aircraft.

Highest-value single addition to the site: it answers "what am I building?" faster than any
paragraph, and it is the one thing a reader deciding whether to start actually looks for.

- Shoot the finished aircraft against a plain background, props off.
- Downscale to ~1600 px wide and save as WebP or JPEG under `docs/assets/`.
- Update the image reference in `docs/index.md`.

### #2 — Cost and build time — 🎯 CRITICAL

The **Parts cost** and **Build time** cards on the handbook index are `TBD(#2)`.

Nearly every DIY build guide omits both, and they are the first two questions a reader has.
Fill them from the actual build rather than an estimate:

- Total parts cost, with the currency and the month priced — component prices move.
- Assembly time and bring-up time as separate figures. They are different activities and
  people budget them differently.

### #3 — Airframe specifications — 🎯 CRITICAL

The **Airframe** card is `TBD(#3)`, and **Power** is missing capacity and all-up weight.

Needed: frame class and prop size, motor and ESC selection, battery capacity, all-up weight,
and measured flight time. Weight and flight time must come from the built aircraft — an
estimate here is worse than the blank, because a reader will size their own pack from it.

### #4 — The remaining stubbed build stages — 🎯 CRITICAL

Materials, boards and wiring have pages. Still to write: frame and motors, power, toolchain,
configure and flash, smoke test, sensors, RC link, bench test, and first flight.

Write each as its stage is reached during the real build, while the details are fresh and
the mistakes are still visible. A stage documented from memory six months later is the kind
that omits the step that actually caused the problem.

### #6 — Buzzer — 🟢 SUPPORTING

The buzzer is visible in the photographs on **The brain** and wired to `GPIO10`, but its
section on **Sensors and peripherals** is a `TBD(#6)` stub.

Needed: the part used and whether it is an active or passive type, how it is wired to
`GPIO10`, where it mounts, and what the firmware actually sounds it for.

---

## 2. Firmware

### #7 — Divide the STM32 into real states — 🎯 CRITICAL

`IdleState` is the only `IState<AppContext>` implementation, and it is not idle. It runs the
rate controller on the fast tick, and RC, GPS, battery, ESC telemetry, the button, USB status
and diagnostics on the slow one — roughly 450 lines across `OnFastTick` and `StepSlow`. The
name is left over from a state machine that was intended and never built.

The cost is not the file length. With a single state, every mode has to be expressed as a flag
checked somewhere inside a service. ESC-configurator passthrough runs with the mixer still
live, and what keeps a flashing session safe is `OnPrivilegedArm` *refusing* an arm — a check
that must be remembered at every future arming path, rather than a state in which the mixer
simply does not run.

The ESP32 is the contrast: it already has disjoint states, because its modes are genuinely
different activities.

- Split at minimum into disarmed, armed, and ESC-config passthrough. Add failsafe when it
  exists — `failsafe_flags` is hardcoded to `0` today, with a `TODO(fc)` in
  `BuildVehicleStatusMsg`.
- Rename `IdleState` as part of the split. The name misdescribes the code whatever else
  changes.
- Sensor reads and telemetry are common to every state, so decide where the shared work lives
  before splitting rather than after — see #8.
- Do it once four-way passthrough has been confirmed on hardware. Restructuring the loop that
  drives the motors is not something to debug alongside an ESC path that has never run.

### #8 — Rehome StatPublisher's scheduling state — 🟢 SUPPORTING

`StatPublisher` has two publish entry points whose schedules live in two different places:

- `PublishTelemetry` is gated by its caller at 1 Hz, through `last_status_send_us_` — a member
  of the state machine.
- `PublishUsbStatus` gates itself, using three members: `usb_status_sent_us_`,
  `usb_status_rx_frames_`, `usb_status_tx_frames_`.

The asymmetry is currently correct rather than accidental. The USB interval is derived from the
message itself — a third of the exchange interval while the frame counters are moving — so the
caller cannot compute it without first building the message, and the two frame members hold the
counts *as last published*, which is a fact that exists nowhere else.

What makes it worth revisiting is #7. Once `IdleState` splits, the 1 Hz telemetry gate has no
state left to live in: it is a property of the loop, not of any one state. That is the moment
to settle three things together:

- Where a loop-wide cadence belongs once the loop is no longer a single state.
- Whether two publish entry points are still the right shape, or whether one entry point
  carrying a per-message cadence reads better.
- `usb_status_rx_frames_` and `usb_status_tx_frames_` are absolute counts truncated to
  `uint8_t`, so they are wrapping change-detectors rather than counts. A single stored value
  would detect change just as well, since both counters only ever increase.

### #9 — Share the FcLink frame parser — 🧊 DEFERRED

The byte-at-a-time receive state machine exists twice, in `stm32/Services/Src/fc_link.cpp` and
`esp32/services/fc_link.cpp` — same seven states, same transitions, same field accumulation,
plus an `RxState` enum and an `rx_pkt_internal_` struct declared identically in both headers.

The reason to fix it is not the line count. **CRC verification is implemented twice,
differently.** The STM32 rebuilds header and payload into one contiguous buffer and runs
`checksum::XModem` over it; the ESP32 feeds `XModemUpdate` byte by byte across magic, id, length
and payload. The two agree only for as long as `message::Header` stays packed as exactly
`{magic[2], id, len}`, and a divergence would not fail loudly — the link would simply stop
carrying packets.

What differs between the two sides is policy, not parsing. The STM32 resyncs silently and
dispatches inline; the ESP32 counts invalid frames, logs them, sounds the error tone, panics
past a threshold, and queues rather than dispatching.

- Add a policy-free parser to `libs/inc/`, alongside `ring_buffer.hpp` and `dispatcher.hpp`:
  one byte in, one verdict out — need-more, bad magic, bad length, bad CRC, packet complete —
  so each firmware keeps its own reaction to each verdict.
- Leave `Poll`, transmit, handshake, the ring buffers and the read budgets where they are.
  Those genuinely differ: an interrupt-fed byte ring on one side, block reads from the ESP-IDF
  driver on the other.
- Folds away the `should_alert_invalid` lambda the ESP32 currently writes out twice in the same
  file.

Deferred rather than supporting because it rewrites the receive hot path on both firmwares. Do
it once the ESC configurator work has been confirmed on hardware, so that a misbehaving bench
session has one candidate cause instead of two.
