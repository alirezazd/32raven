# ESP32 Driver Instructions

Use these instructions when writing or modifying ESP32 firmware under `esp32/`.

## Code Layout

- `esp32/drivers/`: bare-metal-ish wrappers around ESP-IDF peripherals (LED, buzzer, button, I²C, UART, SSD1306 panel).
- `esp32/services/`: longer-lived components that own state machines or background FreeRTOS tasks (UI, MAVLink, programmer, FC-link, TCP/UDP servers, panic, system).
- `esp32/ui/`: display rendering — `display_renderer.cpp`, widgets under `ui/widgets/`, the Adafruit-GFX adapter under `ui/adafruit_gfx_compat/`.
- `esp32/main/`: `app_main`, the top-level state machine (`states.cpp`), and `command_handler.cpp`. The component register lives in `esp32/main/CMakeLists.txt`.

## Singleton Pattern

- Every driver/service exposes `static T& GetInstance()` returning a function-local static. Constructor + destructor are private; copy/move deleted.
- `friend class System` lets `System::InitComponent(...)` call the private `Init(const Config&, ...)`. Anything else accesses the instance through `Sys().<Name>()`.
- Add new components by extending `Component` enum in `esp32/services/system.hpp` and a `case` in `esp32/services/system.cpp`. Keep init order deterministic — later components may rely on earlier ones.

## ESP-IDF Integration

- `esp32/CMakeLists.txt` invokes `idf_component_register()` indirectly via `esp32/main/CMakeLists.txt`. Add new sources to the `SRCS` list there. Headers don't go in `SRCS`.
- Shared code in `libs/` is wired via `EXTRA_COMPONENT_DIRS=../libs` (see `esp32/CMakeLists.txt`); the libs component re-exports `libs/inc` via `INCLUDE_DIRS`.
- ESP-IDF builds run through `idf.py build` (driven from the top-level CMake). Do not call `idf.py` directly for routine builds — use `make esp32` so the lint pass and size metrics run.
- `extern "C" { ... }` is required around ESP-IDF C headers (FreeRTOS, esp_log, driver/*). Project headers are C++ and stay outside that block.

## FreeRTOS Tasks

- Prefer `xTaskCreateStatic*` so stacks are file-scope `StackType_t` arrays — avoids heap fragmentation and matches the `_GLIBCXX_HAVE_POSIX_SEMAPHORE` build flags.
- Pin time-sensitive tasks to a specific core (`xTaskCreateStaticPinnedToCore(..., 0)` for the panic task, etc.). Defaults to PRO_CPU on ESP32-C3 (single-core), but be explicit.
- Use `taskENTER_CRITICAL(&lock)` + `portMUX_INITIALIZER_UNLOCKED` for short, ISR-safe critical sections; FreeRTOS mutexes for longer waits.

## Panic And Error Codes

- ESP32 codes live under `ErrorCode::Esp32::*` (range `0x20000+`). Cross-MCU codes shared with STM32 live under `ErrorCode::Common::*`.
- `Panic(ErrorCode::Esp32::kFoo)` forwards to `PanicImpl(uint32_t)` (defined in `esp32/services/panic.cpp`). The panic task picks it up via `xTaskNotify`, halts other subsystems, and runs the recoverable-loop UI.
- Wire-side APIs (`Mavlink::ReportPanic`, `Ui::SetErrorCode`, `Programmer::LastErrorCode`) take `uint32_t` so STM32-domain codes received over `fc_link` can be displayed/logged without re-typing.

## Config Wiring

- User-tunable ESP32 settings go in `config/Kconfig` under the ESP32 menus. Add the lookup to `scripts/generate_esp32_config.py`.
- `esp32/main/esp32_config.hpp` and `esp32/main/esp32_limits.hpp` are generated — never hand-edit. Consume `k...Config` constants from those files and pass them through `System::InitComponent(...)`.

## Forbidden / Lint

- `scripts/lint/check_forbidden.sh` runs as a pre-build dependency of `make esp32`. It rejects exception/RTTI usage, dynamic allocation in hot paths, and other patterns listed in `scripts/lint/forbidden_exceptions.txt` (with allowlist overrides for vendored or unavoidable spots).
- Build flags include `-fno-exceptions -fno-rtti -ffunction-sections -fdata-sections`. Do not regress them.

## Before Finishing

- New driver/service files are in the right `drivers/` vs `services/` bucket and listed in `esp32/main/CMakeLists.txt` `SRCS`.
- `System::InitComponent` includes the new component in deterministic order.
- Every tunable parameter is wired end-to-end: `Kconfig` → `generate_esp32_config.py` → `kFooConfig` → `Foo::Init(kFooConfig)`.
- `make esp32` succeeds (build, lint pass, size metrics).
