# Shared Libraries Guide

`libs/` is dual-target code that compiles on both `arm-none-eabi-g++` (STM32) and `riscv32-esp-elf-g++` (ESP32). Anything here is wire-protocol contract, error-code contract, or general-purpose utility used by both firmware images.

## Scope

- `libs/inc/message.hpp`: inter-MCU wire protocol — packed structs, message IDs, payload validators.
- `libs/inc/error_code.hpp` + `libs/src/error_code.cpp`: `struct ErrorCode { Common, Stm32, Esp32 };` enums + `GetMessage` overloads.
- `libs/inc/state_machine.hpp`: `IState<Ctx>` + `StateMachine<Ctx>` template used by both targets.
- `libs/inc/dispatcher.hpp`, `libs/inc/ring_buffer.hpp`: small dependency-free utilities.

## Hard Constraints

- **No platform-specific includes.** No `freertos/*.h`, no `stm32f4xx_hal.h`, no `driver/gpio.h`. If you need timing or tasking, take it as a template parameter or function reference.
- **No dynamic allocation in hot paths.** Stack/static only. Both build configs disable exceptions and RTTI (`-fno-exceptions -fno-rtti`); standard library code that throws is off-limits.
- **Trivially copyable wire types.** Every struct in `message.hpp` is `__attribute__((packed))` and must satisfy `std::is_trivially_copyable_v<T>`. `command_handler.cpp` static-asserts this on dispatch.
- **No floating-point in protocol structs.** Use scaled integers (`uint16_t batt_voltage_mv`, `int16_t batt_current_ca`) so wire encoding stays endian-trivial and decode is the same on both MCUs.

## Wire Protocol Changes

- Adding a new `MsgId` and `*Msg` struct: append to `message::MsgId`, define the packed struct, add `IsPayloadValid` / `IsPacketValid` cases, register handlers in both `esp32/main/command_handler.cpp` and `stm32/Services/Src/command_handler.cpp`.
- Renumbering existing `MsgId` values is a flag-day break — both firmwares must flash together. Avoid unless absolutely necessary.
- Field-layout changes (adding/removing/reordering members) silently corrupt deserialization on the unmodified side. Treat the same way as a renumber.
- `static_assert` payload sizes against `message::kMaxPayload` when you add anything bigger than a few bytes.

## Error Codes

- Three disjoint domains: `Common` (`0x0xxxx`), `Stm32` (`0x10000+`), `Esp32` (`0x20000+`). The numeric ranges let `GetMessage(uint32_t)` dispatch to the right domain on the receiving MCU.
- `Common` is for codes that can legitimately be raised on either side (`kOk`, `kUnknown`, `kCommandInvalidPacket`, the dual-use FcLink config codes).
- Add codes in their natural domain. The `Stm32` enum drops the redundant `kStm32` prefix — `Stm32::kSpiInitFailed`, not `Stm32::kStm32SpiInitFailed`.
- Every new value needs a matching `case ErrorCode::<Domain>::kFoo: return "...";` line in the corresponding `GetMessage(...)` overload in `error_code.cpp`. The compiler's `-Werror=switch` catches missing cases.

## Build Wiring

- STM32 picks up `libs/src/error_code.cpp` directly via `target_sources(... ../libs/src/error_code.cpp)` in `stm32/CMakeLists.txt`.
- ESP32 picks it up as a component — `EXTRA_COMPONENT_DIRS=../libs` in `esp32/CMakeLists.txt`, then `libs/CMakeLists.txt` registers it via `idf_component_register(INCLUDE_DIRS "inc" SRCS "")` and `esp32/main/CMakeLists.txt` adds the `.cpp`.

## Verification

- `make stm32` AND `make esp32` after any change here. Don't ship a wire change that only one target compiled.
- For an `error_code` change, also exercise the receiving side: trip a panic on STM32, confirm ESP32 displays the right message via `GetMessage(uint32_t)`.
