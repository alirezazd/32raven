// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

// A bitmap plus the dimensions that describe it. Kept in its own header so the
// generated assets can name the type without pulling in the renderer.
//
// Column-page packing: bit (y % 8) of byte [(y / 8) * width + x]. That is the
// SSD1306's own layout, so a full-screen image reaches the panel unshuffled.
struct PackedBitmap {
  std::span<const uint8_t> data;
  size_t width = 0;
  size_t height = 0;

  [[nodiscard]] constexpr bool Valid() const {
    return width != 0 && height != 0 &&
           data.size() >= ((height + 7u) / 8u) * width;
  }

  // Out of range reads as unlit rather than failing: callers draw whole
  // rectangles and clip by asking.
  [[nodiscard]] constexpr bool Pixel(size_t x, size_t y) const {
    if (x >= width || y >= height) {
      return false;
    }
    const size_t index = ((y / 8u) * width) + x;
    return index < data.size() && (data[index] & (1u << (y % 8u))) != 0;
  }
};
