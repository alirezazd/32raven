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
- Treat `libs/inc/message.hpp` as the shared protocol contract. Changes there affect both ESP32 and STM32 and should be validated on both sides.

## Done Means

- The requested behavior is implemented.
- Relevant target builds pass.
- Generated-file workflows remain intact.
- The diff is reviewable and does not include unrelated cleanup.
