#pragma once

#include <cstdint>

// Wire-side panic — accepts uint32_t so the panic loop and serializer share
// one entry point regardless of which ErrorCode domain raised it.
// Callers pass typed ErrorCode::{Common,Stm32,Esp32} values via the Panic<E>
// template below; including error_code.hpp here would be a transitive smell.
void PanicImpl(uint32_t code);

// Type-safe helpers — usable with any ErrorCode::{Common,Stm32,Esp32} value.
// Forwards to PanicImpl after casting; the enum's underlying disjoint range
// (0x0 / 0x1xxxx / 0x2xxxx) preserves domain on the wire.
template <typename E>
inline void Panic(E code) {
  PanicImpl(static_cast<uint32_t>(code));
}
