# STM32 Driver Instructions

Use these instructions when writing or modifying STM32 drivers in this repository, especially MCU-facing code under `stm32/Drivers/USER`, `stm32/Core`, and closely related init wiring.

## Driver Layout

- Put driver headers in `stm32/Drivers/USER/inc` and implementations in `stm32/Drivers/USER/cc`.
- Follow the repository singleton pattern for MCU-facing drivers: `GetInstance()`, private constructor, deleted copy operations.
- Define typed config structs and enums in headers, then pass config through `Init(...)` instead of scattering register constants at call sites.
- Guard re-initialization with `initialized_` and fail consistently with `ErrorHandler()` or `Panic(ErrorCode::...)`, depending on severity.
- Keep CubeMX/HAL interop explicit. If ISR or HAL callback linkage requires global symbols or C linkage, keep that visible and deliberate.

## System Wiring

- Integrate new driver bring-up in `stm32/Core/Src/system.cpp` via `System::InitComponent(...)`.
- Preserve deterministic bring-up ordering.
- If a new component is introduced, update the component enum and switch handling in the STM32 system files.
- Prefer dependency injection in init calls, such as passing GPIO, SPI, or UART references, over hidden global lookups.

## Config Wiring

- Put user-tunable STM32 settings in `config/Kconfig` under the STM32 menus.
- Map each new Kconfig symbol in `scripts/generate_stm32_config.py`.
- Treat `stm32/Drivers/USER/inc/stm32_config.hpp` and `stm32/Drivers/USER/inc/stm32_limits.hpp` as generated files. Do not hand-edit them.
- Consume generated constants through `#include "stm32_config.hpp"` and pass `k...Config` objects from `System::InitComponent(...)`.

## Build Wiring

- Add new STM32 `.cpp` files to `target_sources(...)` in `stm32/CMakeLists.txt`.
- Stay within the existing include roots unless a new include root is genuinely needed: `Drivers/USER/inc`, `Core/Inc`, `Services/Inc`, `libs/inc`.
- Keep STM32 code compatible with the project build flags, including `-fno-exceptions` and `-fno-rtti`.

## DMA, IRQ, and Fault Handling

- Keep ISR paths bounded and deterministic. Move heavyweight work to deferred paths.
- On DMA fault paths, clear hardware flags, reinitialize stream state coherently, and keep software state synchronized before retrying.
- Reuse the repository's existing fault signaling patterns: error codes, counters, LED latch behavior, panic paths, and error handlers.
- Avoid ad-hoc logging from tight IRQ or DMA paths.

## Before Finishing

- Driver files are in the correct `Drivers/USER/inc` and `Drivers/USER/cc` locations.
- System integration is wired through the STM32 system/component init path.
- Every user-facing tuning parameter is wired end-to-end: `Kconfig` -> `generate_stm32_config.py` -> generated config object -> driver `Init(...)`.
- `stm32/CMakeLists.txt` includes any new source files.
