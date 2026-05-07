# Scripts Guide

Use these instructions when editing code generators, templates, lint hooks, or build helpers under `scripts/`.

## What's Here

- `generate_stm32_config.py` / `generate_esp32_config.py`: read `config/32raven.config` (kconfiglib), evaluate `config/Kconfig` symbols, render `<target>_config.hpp` and `<target>_limits.hpp` from Jinja2 templates.
- `generate_ee_schema.py`: emits `stm32/Drivers/USER/inc/ee_schema.hpp` from `config/ee.toml`.
- `generate_firmware_ver.py`: stamps a build-id header for ESP32.
- `pin_constraints.py`: parses ST's open-pin-data XML (vendored under `third_party/stm32_open_pin_data/`) and exposes `PinConstraints.load_default()` with `is_valid_signal_on_pin`, `pins_for_signal`, `af_for`, etc.
- `check_pinmap.py`: runs `pin_constraints` against `stm32/Core/Inc/board.hpp` + the freshly-generated `stm32/Drivers/USER/inc/stm32_config.hpp`. Wired as the third `COMMAND` of the STM32 config-generation `add_custom_command` — there is no standalone target.
- `templates/`: Jinja2 templates consumed by the generators. One template per generated header.
- `lint/check_forbidden.sh` + `lint/forbidden_exceptions.txt`: pre-build static check for banned constructs (RTTI, exceptions, dynamic alloc in hot paths). Allowlist exceptions live in the `.txt`.
- `setup-vscode.sh`: idempotent merge into `.vscode/settings.json` (preserves user customizations).
- `32raven_menuconfig.py`: thin wrapper around `kconfiglib`'s curses menuconfig, pointed at `config/Kconfig` + `config/32raven.config`.

## Generator Conventions

- Generators take `--kconfig`, `--config`, `--runtime-out` (config header), and `--limits-out` (limits header) on the CLI. Match the existing argparse signature — `stm32/CMakeLists.txt` and `esp32/main/CMakeLists.txt` invoke them with those flags.
- Read symbols via the helpers in the script (`_sym_int`, `_sym_bool`, `_sym`, `_choice_value`). They handle missing symbols and choice-of-bool patterns consistently.
- Build the Jinja context as a single nested `dict` (`{"button": {...}, "battery": {...}, ...}`). Templates address fields like `{{ button.debounce_ms }}` — keep the per-component grouping.
- Use `{{ autogen_warning }}` at the top of every template. The header it emits tells humans "do not edit by hand" and points at the generator.

## Templates

- One template per generated header. Don't merge templates across targets.
- Generated C++ must compile with both target flag sets (`-fno-exceptions -fno-rtti`, etc.). No exceptions, no RTTI, no dynamic alloc.
- Templates open `namespace board { ... }` (re-opening the namespace declared in `stm32/Core/Inc/board.hpp`) when emitting pin-map entries — don't redeclare the type, just add `inline const BoardPin k... = {...};` lines.
- After generation, `clang-format -i` runs on the output (when available). Don't try to hand-format inside the template — keep template logic readable and let clang-format normalize.

## Pin Map Generation

- `PINMAP_ENTRIES` in `generate_stm32_config.py` is the registry of generated `BoardPin` entries. Two flavors:
  - `_SignalPin`: AF-constrained (e.g. UART2_TX, SPI1_SCK). Generator validates `(pin, signal)` against `pin_constraints` and looks up the AF.
  - `_GpioPin`: plain GPIO (port + pin number, no AF). Used for User LED, User Button, etc.
- Adding a peripheral pin to menuconfig:
  1. Add a `choice` block under `STM32 -> Pin Map -> ...` in `config/Kconfig`. Option names should encode the pin (e.g. `STM32_UART2_TX_PA2`).
  2. Add a `_SignalPin` (or `_GpioPin`) to `PINMAP_ENTRIES`.
  3. Re-render — `check_pinmap.py` will reject any combo ST never published.

## Lint Hooks

- `check_forbidden.sh` runs against `esp32/` + `libs/` for ESP32 builds, and `stm32/` + `libs/` for STM32. The CMake stamps in `build/.../forbidden_check_*.stamp` track inputs so re-runs are incremental.
- Allowlist additions (one per line in `forbidden_exceptions.txt`) should justify the exception in a comment. Bare allowlists rot fast.

## When Touching A Generator

- Re-run the relevant `make <target>` after every change. The generator runs at configure-time of the inner CMake invocation — a syntax error there breaks the build, not just the script.
- Keep the script's output deterministic. No timestamps in generated files, no environment-dependent ordering. Ordered iteration over dicts is fine in Python 3.7+.
- New generators register their outputs as `OUTPUT` of an `add_custom_command` and add the script + template paths to `DEPENDS` so changes trigger regeneration.
