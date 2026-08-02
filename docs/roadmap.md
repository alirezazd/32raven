# 32Raven — Roadmap

The single source of truth for **32Raven firmware and handbook work**. Every `TBD(#N)` marker
in the handbook points to an item here, and `scripts/lint/check_docs.py` fails the build if it
points at an item that does not exist — so a placeholder cannot be quietly forgotten.

**Scope — this repository.** Firmware and handbook work only.

**Priority legend**

- 🎯 **CRITICAL** — blocks the prototype build, or blocks the handbook being usable by
  someone who is not the author.
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

### #5 — USART1 and USART6 are absent from `PINMAP_ENTRIES` — 🟢 SUPPORTING

The inter-MCU link (USART1, `FcLink`) and the RC receiver input (USART6) have no entries in
`PINMAP_ENTRIES`. Two consequences:

- Their pins are not menuconfig-tunable — they are fixed in `board.hpp`.
- `scripts/lint/check_pinmap.py` does not validate them against ST's silicon data, so a
  typo'd alternate function on either would reach hardware without the build objecting.

Every other peripheral in the pin map is validated. The wiring page flags both explicitly
so the gap does not silently mislead a reader, but the real fix is entries in the pin map.
