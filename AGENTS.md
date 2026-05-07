# 32Raven Agent Guide

Use this file for repository-wide guidance. More specific `AGENTS.md` files in subdirectories override this file for their area. When working in any of these subtrees, read the local guide first:

- [`stm32/AGENTS.md`](./stm32/AGENTS.md) — STM32 driver / system / pin-map conventions
- [`esp32/AGENTS.md`](./esp32/AGENTS.md) — ESP32 driver / service / FreeRTOS conventions
- [`libs/AGENTS.md`](./libs/AGENTS.md) — dual-target shared code (wire protocol, error codes)
- [`scripts/AGENTS.md`](./scripts/AGENTS.md) — code generators, templates, Kconfig integration
- [`config/AGENTS.md`](./config/AGENTS.md) — Kconfig + checked-in defaults

## Repo Layout

- `esp32/`: ESP32 firmware, split across `drivers/`, `services/`, `ui/`, and `main/`.
- `stm32/`: STM32 firmware, with application code in `Core/`, drivers in `Drivers/USER/`, and services in `Services/`.
- `libs/`: dual-target protocol and utility code (`libs/inc/message.hpp`, `libs/inc/error_code.hpp`).
- `config/`: user-tunable configuration inputs (`Kconfig`, `32raven.config`, `ee.toml`).
- `scripts/`: code generators (Kconfig → headers via Jinja2), pin-map validator, lint hooks.
- `third_party/`: vendored dependencies — `esp-idf`, MAVLink, Adafruit GFX, ST open-pin-data.

## Working Style

- Keep changes narrow and local. Do not refactor unrelated code while fixing one issue.
- Match the surrounding file’s style unless it conflicts with the enforced naming rules in `.clang-tidy`.
- Treat `.clang-tidy` as the source of truth for identifier naming.
- Prefer extending existing abstractions over adding parallel ones.
- Prefer true function-style APIs with explicit input parameters and return values. Avoid output-parameter/reference styles such as `bool BuildThing(Thing &out)` when the result can be returned directly as `Thing`, `std::optional<Thing>`, or another clear return type. Use non-const references for intentional in-place mutation of existing state, not as a substitute for return values.
- Do not edit vendored code under `third_party/` unless the task is explicitly to update or patch that dependency.
- Do not hand-edit generated files or build outputs.

## Generated And Derived Files

- Treat these as generated and regenerate them via scripts or the build:
  - `esp32/main/esp32_config.hpp`
  - `stm32/Drivers/USER/inc/stm32_config.hpp`
  - `stm32/Drivers/USER/inc/stm32_limits.hpp`
  - `stm32/Drivers/USER/inc/ee_schema.hpp`
- Do not commit edits to `build/`, `build/Ninja/`, or other derived output trees.

## Build And Verify

- Use the top-level `Makefile` from the repo root.
- Common commands:
  - `make stm32`
  - `make esp32`
  - `make all`
- When changing only one firmware target, run that target’s build before finishing.
- When touching shared code in `libs/` or shared config/protocol files, prefer validating both `make stm32` and `make esp32`.

## Config And Protocol Changes

- Put user-facing configuration in `config/Kconfig` and the related source config files under `config/`.
- If a change depends on generated config headers or schema, update the generator script instead of patching generated output.
- Keep configuration changes synchronized end-to-end: `config/Kconfig`, checked-in config files under `config/`, generator scripts in `scripts/`, and any generated-header templates they feed.
- Every user-facing `config` entry added or renamed in `config/Kconfig` must include accurate `help` text that explains the setting in terms of product behavior, not just the implementation detail.
- Treat `libs/inc/message.hpp` as the shared protocol contract. Changes there affect both ESP32 and STM32 and should be validated on both sides.

## Adding A New Tunable

When introducing a new constant, decide which bucket it belongs to **before** writing the code. Both buckets pass through code generators — there is no place in the source tree where you should hand-write a value that conceptually belongs to either of these.

- **Config (user-tunable, Kconfig-driven):** values someone changes via menuconfig without editing C++. Live wire: `config/Kconfig` → `scripts/generate_<target>_config.py` → `scripts/templates/<target>_config.hpp.j2` → generated `<target>_config.hpp` → consumed as `kFooConfig`. Anything user-facing (sample rates, timeouts, retry counts, pin assignments that vary by board variant) goes here.
- **Limits (developer-derived, compile-time bounds):** values computed from other inputs or deliberately not exposed to end users — array sizes, prescaler selections, ADC resolution. Live wire: `scripts/generate_<target>_config.py` (often derived inside the same generator) → `scripts/templates/<target>_limits.hpp.j2` → generated `<target>_limits.hpp` → consumed as `<target>_limits::kFoo`.

For a new **config** entry:
1. Add the symbol + `help` text to `config/Kconfig` in the appropriate menu.
2. Map it in `scripts/generate_<target>_config.py` (look for the existing `_sym_int` / `_sym_bool` / `_choice_value` patterns).
3. Add the field to the `*_config.hpp.j2` template so it lands in the generated `kFooConfig` struct.
4. **Run `make 32raven-menuconfig`** (or edit `config/32raven.config` directly) so the checked-in defaults file picks up the new symbol — otherwise the generator falls back to the Kconfig default and the file diverges.
5. Consume it via `#include "<target>_config.hpp"` and pass `kFooConfig` through `System::InitComponent(...)`.

For a new **limit**:
1. Compute it in the relevant generator script (or hard-code it inside the script if it's a fixed constant — but ask why it isn't a config first).
2. Render it into `*_limits.hpp.j2`.
3. Consume via `#include "<target>_limits.hpp"`.

Do not skip step 4 for configs. A config entry that exists in `Kconfig` but not in `32raven.config` produces silent default-value drift — the generator emits the Kconfig `default`, which may not match what menuconfig users see.

## Error Codes

- `libs/inc/error_code.hpp` exposes `struct ErrorCode { enum class Common; enum class Stm32; enum class Esp32; };` with disjoint numeric ranges (`0x0xxxx` / `0x1xxxx` / `0x2xxxx`) so a panic raised on one MCU can be decoded on the other via `GetMessage(uint32_t)`.
- Raise panics with `Panic(ErrorCode::<Domain>::kFoo)`. The `Panic` template forwards to `PanicImpl(uint32_t)` on each target.
- Wire-side surfaces (`message::PanicMsg::error_code`, `Programmer::LastErrorCode()`, `Mavlink::ReportPanic`, `Ui::SetErrorCode`) use `uint32_t` so they can carry codes from either MCU.
- Add new codes in their natural domain. The STM32 enum drops the redundant `kStm32` prefix (`Stm32::kSpiInitFailed`, not `Stm32::kStm32SpiInitFailed`).

## Done Means

- The requested behavior is implemented.
- Relevant target builds pass.
- Generated-file workflows remain intact.
- The diff is reviewable and does not include unrelated cleanup.
