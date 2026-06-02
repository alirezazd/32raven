# Scripts Guide

Use these instructions when editing code generators, templates, lint hooks, or build helpers under `scripts/`.

## Layout

```
scripts/
├── kconfig_gen.py               ── shared helpers: sym_*, choice_value, run()
├── generate_<target>_config.py  ── Kconfig → Jinja2 → header generators
├── generate_ee_schema.py
├── generate_firmware_ver.py
├── pin_constraints.py           ── shared helper (used by generator + linter)
├── 32raven_menuconfig.py        ── menuconfig wrapper
├── setup-vscode.sh              ── VSCode workspace settings merger
├── lint/
│   ├── check_pinmap.py          ── stm32_config.hpp pin map vs ST silicon
│   ├── check_error_codes.py     ── unused-enumerator detector (--fix to prune)
│   ├── check_forbidden.sh       ── banned-construct grep
│   └── forbidden_exceptions.txt ── allowlist for the above
└── templates/                   ── Jinja2 templates, one per generated header
```

## What Each File Does

- `kconfig_gen.py`: shared module imported by both per-target generators. Pinned semantics for `sym_int` (`int(value, 0)`, hex-aware), `sym_bool` (`tri_value == 2`), `choice_value`, plus the Jinja env, `cpp_bool` filter, `autogen_warning(source)` helper, and the `run(...)` entrypoint that handles argparse / Kconfig load / template render. Adding a new helper used by both generators goes here, not in either generator.
- `generate_stm32_config.py` / `generate_esp32_config.py`: read `config/32raven.config` (kconfiglib via the helpers above), evaluate `config/Kconfig` symbols, render `<target>_config.hpp` and `<target>_limits.hpp` from Jinja2 templates. Each `main()` is a one-liner that hands `validate_fn`, `runtime_context_fn`, `limits_context_fn`, and template names to `kconfig_gen.run`.
- `generate_ee_schema.py`: emits `stm32/Drivers/Inc/ee_schema.hpp` from `config/ee.toml`.
- `generate_firmware_ver.py`: stamps a build-id header for ESP32.
- `pin_constraints.py`: parses ST's open-pin-data XML (vendored under `third_party/stm32_open_pin_data/`) and exposes `PinConstraints.load_default()` with `is_valid_signal_on_pin`, `pins_for_signal`, `af_for`, etc. Lives at `scripts/` root because both the generator and `lint/check_pinmap.py` import it.
- `lint/check_pinmap.py`: runs `pin_constraints` against the freshly-generated `stm32/Drivers/Inc/stm32_config.hpp` (the only source of `BoardPin` declarations). Wired as a `COMMAND` of the STM32 config-generation `add_custom_command` — there is no standalone target.
- `lint/check_error_codes.py`: parses `libs/inc/error_code.hpp` and fails the build if any enumerator has zero callsites. Run with `--fix` to also prune them from `error_code.hpp` + the matching switch case in `error_code.cpp`.
- `lint/check_forbidden.sh` + `lint/forbidden_exceptions.txt`: pre-build static check for banned constructs (RTTI, exceptions, dynamic alloc in hot paths). Allowlist exceptions live in the `.txt`.
- `setup-vscode.sh`: idempotent merge into `.vscode/settings.json` (preserves user customizations).
- `32raven_menuconfig.py`: thin wrapper around `kconfiglib`'s curses menuconfig, pointed at `config/Kconfig` + `config/32raven.config`.

## Generator Conventions

- Generators take `--kconfig`, `--config`, `--runtime-out` (config header), and `--limits-out` (limits header) on the CLI. Don't reimplement the boilerplate; call `kconfig_gen.run(validate_fn=..., runtime_context_fn=..., limits_context_fn=..., runtime_template_name=..., limits_template_name=...)` from `main()`.
- Read symbols via the shared helpers in `kconfig_gen` (`sym_int`, `sym_bool`, `sym_str`, `sym_hex_literal`, `choice_value`). They share one canonical interpretation across both targets.
- Build the Jinja context as a single nested `dict` (`{"button": {...}, "battery": {...}, ...}`). Templates address fields like `{{ button.debounce_ms }}` — keep the per-component grouping. Each peripheral block lives in its own `_foo_context(kconf)` helper; `_runtime_context` just stitches the helpers together.
- Use `{{ autogen_warning }}` (rendered via `autogen_warning(source)`) at the top of every template. The header it emits tells humans "do not edit by hand" and points at the generator.

## Limits vs Config Split

Two generated headers per target — be deliberate about which one a value belongs in:

- `<target>_config.hpp` carries **runtime configuration** the firmware passes to drivers — Config structs (`kBatteryConfig`, `kSpi2Config`, `kMavlinkConfig`, …), board-tunable runtime constants (`kMavlinkSysAutostart`, `kPanicTaskStackDepthWords`), and pin map entries. Mostly file-scope (no namespace).
- `<target>_limits.hpp` carries **silicon / hardware-fixed bounds and compile-time sizes** used as non-type template parameters (`std::array<T, N>`, `RingBuffer<T, N>`). Wrapped in a `<target>_limits::` namespace. Examples: `kBatteryAdcResolutionBits` (F407 silicon), `kDisplayPanelWidth` (panel hardware), `kTcpServerDownloadBufferBytes` (RingBuffer compile-time size).
- If you can't decide: ask "could a downstream board legitimately want this different?" If yes, it's likely a config; if it's silicon/protocol-fixed or a buffer size template parameter, it's a limit.

## Templates

- One template per generated header. Don't merge templates across targets.
- Generated C++ must compile with both target flag sets (`-fno-exceptions -fno-rtti`, etc.). No exceptions, no RTTI, no dynamic alloc.
- Templates declare `namespace board { ... }` and the `BoardPin` struct inline; the pin map entries are then `inline const BoardPin k... = {...};` lines inside that block. There is no other source of `BoardPin` definitions in the codebase.
- After generation, `clang-format -i` runs on the output (when available). Don't try to hand-format inside the template — keep template logic readable and let clang-format normalize.

## Pin Map Generation

- `PINMAP_ENTRIES` in `generate_stm32_config.py` is the registry of generated `BoardPin` entries. Four flavors:
  - `_SignalPin`: AF-constrained (e.g. UART2_TX, SPI1_SCK). Generator validates `(pin, signal)` against `pin_constraints` and looks up the AF.
  - `_GpioPin`: plain GPIO (port + pin number, no AF). Used for User LED, User Button, chip-selects.
  - `_ExtiPin`: GPIO input wired to an EXTI line (e.g. IMU INT). Same surface as `_GpioPin`; the NVIC IRQn is silicon-derived from the pin number.
  - `_AnalogPin`: ADC1 input (battery sense). Same surface as `_GpioPin`; the matching ADC channel constant (`kFooAdcChannel`) is emitted alongside the BoardPin entry.
- Adding a peripheral pin to menuconfig:
  1. Add a `choice` block under `STM32 -> Pin Map -> ...` in `config/Kconfig`. Option names should encode the pin (e.g. `STM32_UART2_TX_PA2`).
  2. Add the matching entry to `PINMAP_ENTRIES`, ordered to mirror the Kconfig menu. Choose `direction` / `pull` / `speed` / `active_low{,_sym}` per the entry's role — the generator emits both the `BoardPin` and its `GPIO::PinConfig` line in `kGpioDefault`.
  3. Re-render — `lint/check_pinmap.py` will reject any combo ST never published.

## Lint Hooks

- `lint/check_forbidden.sh` runs against `esp32/` + `libs/` for ESP32 builds, and `stm32/` + `libs/` for STM32. The CMake stamps in `build/.../forbidden_check_*.stamp` track inputs so re-runs are incremental.
- Allowlist additions (one per line in `lint/forbidden_exceptions.txt`) should justify the exception in a comment. Bare allowlists rot fast.
- `lint/check_pinmap.py` and `lint/check_error_codes.py` run as `COMMAND` steps of the STM32 config-generation `add_custom_command`, so any change to `error_code.hpp` or the Kconfig / generator inputs re-triggers them automatically.

## When Touching A Generator

- Re-run the relevant `make <target>` after every change. The generator runs at configure-time of the inner CMake invocation — a syntax error there breaks the build, not just the script.
- Keep the script's output deterministic. No timestamps in generated files, no environment-dependent ordering. Ordered iteration over dicts is fine in Python 3.7+.
- New generators register their outputs as `OUTPUT` of an `add_custom_command` and add the script + template paths to `DEPENDS` so changes trigger regeneration.
