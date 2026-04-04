# Config Area Guide

Use these instructions when editing files under `config/`.

## Kconfig Changes

- Keep naming aligned with the codebase. If a subsystem or class is renamed, update the related Kconfig menu names and symbols unless preserving backward compatibility is an explicit requirement.
- Every user-facing `config` entry must include a non-empty `help` block.
- Help text should describe what the setting changes in firmware behavior, not just restate the symbol name.

## Sync Requirements

- Keep `config/Kconfig`, checked-in config files such as `config/32raven.config`, generator scripts under `scripts/`, and the generated-header templates they feed in sync.
- Do not hand-edit generated headers to compensate for a config drift. Fix the source Kconfig/generator/template inputs instead.
- When renaming a config symbol, update both the Kconfig declaration and every generator lookup that reads it.

## Verification

- After ESP32 config changes, run `make esp32`.
- After STM32 config changes, run `make stm32`.
- If a config or protocol change affects shared code or both targets, prefer running both builds.
