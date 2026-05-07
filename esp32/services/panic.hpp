#pragma once

#include <cstdint>

[[noreturn]] void PanicImpl(uint32_t code);

template <typename E>
[[noreturn]] inline void Panic(E code) {
  PanicImpl(static_cast<uint32_t>(code));
}
