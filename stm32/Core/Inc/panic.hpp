#pragma once

#include <cstdint>

// uint32_t entry point so all ErrorCode domains share one panic loop without
// pulling error_code.hpp into this header. Callers use Panic<E> below.
void PanicImpl(uint32_t code);

// Disjoint underlying ranges (Common 0x0 / Stm32 0x1xxxx / Esp32 0x2xxxx)
// preserve the domain on the wire after the cast.
template <typename E>
inline void Panic(E code) {
  PanicImpl(static_cast<uint32_t>(code));
}
