# STM32 Driver Instructions

Use these instructions when writing or modifying STM32 drivers in this repository, especially MCU-facing code under `stm32/Drivers/USER`, `stm32/Core`, and closely related init wiring.

## Driver Layout

- Put driver headers in `stm32/Drivers/USER/inc` and implementations in `stm32/Drivers/USER/cc`.
- Follow the repository singleton pattern for MCU-facing drivers: `GetInstance()`, private constructor, deleted copy operations.
- Define typed config structs and enums in headers, then pass config through `Init(...)` instead of scattering register constants at call sites.
- Guard re-initialization with `initialized_` and fail consistently with `Panic(ErrorCode::Stm32::...)`. Bridge any HAL `Error_Handler()` callsite through `extern "C" void ErrorHandler(void)` in `system.cpp`, which forwards to `Panic(ErrorCode::Stm32::kHalErrorHandler)`.
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

## Pin Map (board.hpp)

- The single source of truth for this PCB's pin assignments is `stm32/Core/Inc/board.hpp`. Drivers consume entries via `board::k<Name>` rather than reading globals from peripheral drivers.
- Each `BoardPin` bundles `port + pin + AF + EXTI IRQ`. Every entry is validated against ST's silicon constraint data on every build by `scripts/check_pinmap.py`, which is invoked as the third `COMMAND` of the `add_custom_command` that produces `stm32_config.hpp` — there is no standalone target. The vendored data lives at `third_party/stm32_open_pin_data/` (subset of ST's open-pin-data repo for the F407 + F417 GPIO IP version).
- The validator catches typo'd alternate functions (e.g. `GPIO_AF7_SPI2` doesn't exist), pins outside the F407V package, the same physical pin claimed by two `BoardPin` entries, and unreferenced/unknown `board::k*` names. Build aborts on any error — bad pin maps brick boards silently.
- **Adding a new peripheral pin:**
  1. Pick the signal you need (e.g. `CAN1_TX`). Run `python3 -c "from scripts.pin_constraints import PinConstraints as P; print(P.load_default().pins_for_signal('CAN1_TX'))"` from the repo root to see the valid pin/AF combos for the F407V package.
  2. Add the chosen entry to `board.hpp` as `inline const BoardPin kCan1Tx = {GPIOX, GPIO_PIN_n, GPIO_AFn_CAN1, NonMaskableInt_IRQn};`.
  3. Wire it into `kGpioDefault` in `stm32/Drivers/USER/inc/board_config.hpp`.
  4. Build — the validator will reject any combo ST never published.
- **Kconfig-tunable pins:** for pins that need menuconfig exposure (e.g. UART2 TX/RX, User LED/Button), add an entry to `PINMAP_ENTRIES` in `scripts/generate_stm32_config.py` and a matching `choice` block under `STM32 -> Pin Map` in `config/Kconfig`. The generator emits the resulting `inline const BoardPin k...` into `stm32_config.hpp` (which re-opens `namespace board`), and `check_pinmap.py` validates the chosen pin against ST's silicon data. Hand-coded entries in `board.hpp` and generated entries in `stm32_config.hpp` coexist — fix-vs-tunable is a per-peripheral call.

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
