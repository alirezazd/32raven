# Configure and build

## Configure

The reference configuration ships checked in as `config/32raven.config` — the same pins the
[wiring pages](../build/wiring.md) document — so nothing needs configuring before a first
build. To change a tunable:

```bash
make 32raven-menuconfig
```

One menu tree covers both MCUs — **Common**, **STM32** and **ESP32** at the top level — and
saving updates `config/32raven.config`, which the build turns into generated headers for
each target. Pin assignments, baud rates, loop rates, WiFi credentials: if a value is meant
to vary between builds, it lives in this tree rather than in the source.

!!! note

    `make esp32-menuconfig` is a different menu — ESP-IDF's own tree, for SDK internals.
    Every 32Raven tunable is under `make 32raven-menuconfig`.

## Build

```bash
make all       # both targets
make stm32     # flight computer only
make esp32     # bridge only
```

`make help` lists every target with a one-line description.

The two builds are independent — a change under `stm32/` needs only `make stm32`. Shared
code under `libs/` compiles into both, so validate a change there against both targets.

The build is also where the checking happens:

- The STM32 build validates every pin assignment against ST's silicon data
  (`scripts/lint/check_pinmap.py`) — a typo'd pin or impossible alternate-function pair
  aborts the build instead of reaching the aircraft.
- The ESP32 build finishes with a flash/RAM size report and a static stack-depth check on
  its FreeRTOS tasks.

## Generators and output

The top level drives CMake with **Ninja** by default, and build outputs are
generator-specific: `build/Ninja/stm32/`, `build/Ninja/esp32/`. To use another generator,
pass `GEN` — `make configure GEN="Unix Makefiles"` — and when switching generators or
toolchains, run `make clean` first so a stale CMake cache does not confuse the new one.

Next: [flash it](flashing.md).
