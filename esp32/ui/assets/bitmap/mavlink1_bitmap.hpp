#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "packed_bitmap.hpp"

// Generated from shared alpha bounds of mavlink0..3.png
namespace mavlink1_bitmap {

inline constexpr std::size_t kVisibleWidth = 18;
inline constexpr std::size_t kVisibleHeight = 18;
inline constexpr std::size_t kCanvasWidth = 18;
inline constexpr std::size_t kCanvasHeight = 24;
inline constexpr std::size_t kOffsetX = 0;
inline constexpr std::size_t kOffsetY = 0;
inline constexpr std::size_t kVisiblePageCount = 3;
inline constexpr std::array<std::uint8_t, 54> kBitmapData = {
    0x1C, 0x24, 0x68, 0x48, 0x98, 0x10, 0x90, 0x60, 0x18, 0x4C, 0x72,
    0x4A, 0x40, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0E,
    0x0E, 0x1B, 0x1D, 0x0E, 0x0B, 0x06, 0x06, 0x02, 0x02, 0x02, 0x02,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

// Draw through this, not the parts: a call site that restates the dimensions
// can restate them wrongly, and nothing downstream would notice.
inline constexpr PackedBitmap kBitmap{kBitmapData, kVisibleWidth,
                                      kVisibleHeight};
static_assert(kBitmap.Valid(), "packed data too small for its dimensions");

}  // namespace mavlink1_bitmap
