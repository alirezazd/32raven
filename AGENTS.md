# 32Raven Agent Guide

Use this file for repository-wide guidance. More specific `AGENTS.md` files in subdirectories override this file for their area. In particular, STM32 work under `stm32/` must also follow [`stm32/AGENTS.md`](./stm32/AGENTS.md).

## Repo Layout

- `esp32/`: ESP32 firmware, split across `drivers/`, `services/`, and `main/`.
- `stm32/`: STM32 firmware, with application code in `Core/`, drivers in `Drivers/USER/`, and services in `Services/`.
- `libs/`: shared protocol and utility code used by both targets.
- `config/`: user-tunable configuration inputs.
- `scripts/`: code generation and project maintenance scripts.
- `third_party/`: vendored dependencies, including `esp-idf` and MAVLink headers.

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

## Done Means

- The requested behavior is implemented.
- Relevant target builds pass.
- Generated-file workflows remain intact.
- The diff is reviewable and does not include unrelated cleanup.
