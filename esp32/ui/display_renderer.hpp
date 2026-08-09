// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include <cstddef>
#include <cstdint>

#include "packed_bitmap.hpp"
#include "timebase.hpp"

struct DisplayTextStyle {
  enum class Font {
    kDefault,
    kCompact,
  };

  uint8_t scale = 1;
  Font font = Font::kDefault;
};

struct DisplayTextBounds {
  int16_t x = 0;
  int16_t y = 0;
  uint16_t width = 0;
  uint16_t height = 0;
};

enum class Ink : uint16_t { kOff = 0, kOn = 1 };

class RenderCanvas {
 public:
  virtual ~RenderCanvas() = default;

  virtual void Clear() = 0;
  virtual void Fill(Ink ink) = 0;
  virtual size_t Width() const = 0;
  virtual size_t Height() const = 0;
  virtual bool SetPixel(size_t x, size_t y, Ink ink) = 0;
  virtual bool DrawPackedBitmap(const PackedBitmap &bitmap, size_t offset_x,
                                size_t offset_y) = 0;
};

class DisplayRenderer {
 public:
  explicit DisplayRenderer(RenderCanvas *canvas = nullptr) : canvas_(canvas) {}

  void Bind(RenderCanvas *canvas) { canvas_ = canvas; }

  void Clear();
  void Fill(Ink ink);
  size_t Width() const;
  size_t Height() const;
  bool SetPixel(size_t x, size_t y, Ink ink);
  bool DrawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1,
                Ink ink = Ink::kOn);
  bool DrawFastHLine(int16_t x, int16_t y, int16_t width, Ink ink = Ink::kOn);
  bool DrawFastVLine(int16_t x, int16_t y, int16_t height, Ink ink = Ink::kOn);
  bool DrawRect(size_t x, size_t y, size_t width, size_t height,
                Ink ink = Ink::kOn);
  bool FillRect(int16_t x, int16_t y, int16_t width, int16_t height,
                Ink ink = Ink::kOn);
  bool DrawCircle(int16_t x, int16_t y, int16_t radius, Ink ink = Ink::kOn);
  bool FillCircle(int16_t x, int16_t y, int16_t radius, Ink ink = Ink::kOn);
  bool DrawEllipse(int16_t x, int16_t y, int16_t radius_w, int16_t radius_h,
                   Ink ink = Ink::kOn);
  bool FillEllipse(int16_t x, int16_t y, int16_t radius_w, int16_t radius_h,
                   Ink ink = Ink::kOn);
  bool DrawTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2,
                    int16_t y2, Ink ink = Ink::kOn);
  bool FillTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2,
                    int16_t y2, Ink ink = Ink::kOn);
  bool DrawRoundRect(int16_t x, int16_t y, int16_t width, int16_t height,
                     int16_t radius, Ink ink = Ink::kOn);
  bool FillRoundRect(int16_t x, int16_t y, int16_t width, int16_t height,
                     int16_t radius, Ink ink = Ink::kOn);
  bool DrawBitmap(const PackedBitmap &bitmap, size_t offset_x, size_t offset_y);
  bool DrawMosaicBitmap(const PackedBitmap &bitmap, size_t offset_x,
                        size_t offset_y, uint8_t block_size_px);
  bool DrawMosaicTransition(const PackedBitmap &from, const PackedBitmap &to,
                            float progress, uint8_t max_block_size_px);
  bool DrawText(const char *text, int16_t x, int16_t y,
                const DisplayTextStyle &style = {});
  int16_t ScrollOffsetPx(int16_t content_width_px, int16_t available_width_px,
                         TimeMs now, uint16_t pixels_per_second = 24,
                         TimeMs pause_ms = 1000) const;
  bool DrawScrollingText(const char *text, int16_t left_px, int16_t top_px,
                         int16_t available_width_px, TimeMs now,
                         const DisplayTextStyle &style = {},
                         uint16_t pixels_per_second = 24,
                         TimeMs pause_ms = 1000);
  DisplayTextBounds MeasureText(const char *text,
                                const DisplayTextStyle &style = {}) const;
  size_t AnimatedTextLength(TimeMs start_ms, TimeMs now, TimeMs duration_ms,
                            const char *text) const;

 private:
  RenderCanvas *canvas_ = nullptr;
};
