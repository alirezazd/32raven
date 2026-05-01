#pragma once

#include <cstddef>
#include <cstdint>

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

class RenderCanvas {
 public:
  virtual ~RenderCanvas() = default;

  virtual void Clear() = 0;
  virtual void Fill(bool on) = 0;
  virtual size_t Width() const = 0;
  virtual size_t Height() const = 0;
  virtual bool SetPixel(size_t x, size_t y, bool on) = 0;
  virtual bool DrawPackedBitmap(const uint8_t *bitmap_data, size_t width,
                                size_t height, size_t offset_x,
                                size_t offset_y) = 0;
};

class DisplayRenderer {
 public:
  explicit DisplayRenderer(RenderCanvas *canvas = nullptr) : canvas_(canvas) {}

  void Bind(RenderCanvas *canvas) { canvas_ = canvas; }

  void Clear();
  void Fill(bool on);
  size_t Width() const;
  size_t Height() const;
  bool SetPixel(size_t x, size_t y, bool on);
  bool DrawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, bool on = true);
  bool DrawFastHLine(int16_t x, int16_t y, int16_t width, bool on = true);
  bool DrawFastVLine(int16_t x, int16_t y, int16_t height, bool on = true);
  bool DrawRect(size_t x, size_t y, size_t width, size_t height,
                bool on = true);
  bool FillRect(int16_t x, int16_t y, int16_t width, int16_t height,
                bool on = true);
  bool DrawCircle(int16_t x, int16_t y, int16_t radius, bool on = true);
  bool FillCircle(int16_t x, int16_t y, int16_t radius, bool on = true);
  bool DrawEllipse(int16_t x, int16_t y, int16_t radius_w, int16_t radius_h,
                   bool on = true);
  bool FillEllipse(int16_t x, int16_t y, int16_t radius_w, int16_t radius_h,
                   bool on = true);
  bool DrawTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2,
                    int16_t y2, bool on = true);
  bool FillTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2,
                    int16_t y2, bool on = true);
  bool DrawRoundRect(int16_t x, int16_t y, int16_t width, int16_t height,
                     int16_t radius, bool on = true);
  bool FillRoundRect(int16_t x, int16_t y, int16_t width, int16_t height,
                     int16_t radius, bool on = true);
  bool DrawBitmap(const uint8_t *bitmap_data, size_t width, size_t height,
                  size_t offset_x, size_t offset_y);
  bool DrawMosaicBitmap(const uint8_t *bitmap_data, size_t width, size_t height,
                        size_t offset_x, size_t offset_y,
                        uint8_t block_size_px);
  bool DrawMosaicTransition(const uint8_t *from_bitmap_data,
                            const uint8_t *to_bitmap_data, size_t width,
                            size_t height, float progress,
                            uint8_t max_block_size_px);
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
