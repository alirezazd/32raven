#include "display_renderer.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

#include <Adafruit_GFX.h>
#include <Fonts/Org_01.h>

namespace {

inline constexpr uint16_t kPixelOff = 0;
inline constexpr uint16_t kPixelOn = 1;

class GfxRenderer : public Adafruit_GFX {
 public:
  explicit GfxRenderer(RenderCanvas *canvas)
      : Adafruit_GFX(static_cast<int16_t>(canvas != nullptr ? canvas->Width() : 0),
                     static_cast<int16_t>(canvas != nullptr ? canvas->Height()
                                                            : 0)),
        canvas_(canvas) {}

  void ConfigureText(const DisplayTextStyle &style) {
    switch (style.font) {
      case DisplayTextStyle::Font::kCompact:
        setFont(&Org_01);
        break;
      case DisplayTextStyle::Font::kDefault:
      default:
        setFont(nullptr);
        break;
    }
    setTextColor(kPixelOn);
    setTextSize(std::max<uint8_t>(style.scale, 1));
    setTextWrap(false);
  }

  void drawPixel(int16_t x, int16_t y, uint16_t color) override {
    if (canvas_ == nullptr || x < 0 || y < 0) {
      return;
    }

    canvas_->SetPixel(static_cast<size_t>(x), static_cast<size_t>(y),
                      color != kPixelOff);
  }

 private:
  RenderCanvas *canvas_ = nullptr;
};

bool ReadPackedPixel(const uint8_t *bitmap_data, size_t width, size_t height,
                     size_t x, size_t y) {
  if (bitmap_data == nullptr || x >= width || y >= height) {
    return false;
  }
  const size_t page = y / 8;
  const size_t bit = y % 8;
  const uint8_t value = bitmap_data[(page * width) + x];
  return (value & (1u << bit)) != 0;
}

}  // namespace

void DisplayRenderer::Clear() {
  if (canvas_ != nullptr) {
    canvas_->Clear();
  }
}

void DisplayRenderer::Fill(bool on) {
  if (canvas_ != nullptr) {
    canvas_->Fill(on);
  }
}

size_t DisplayRenderer::Width() const {
  return (canvas_ != nullptr) ? canvas_->Width() : 0;
}

size_t DisplayRenderer::Height() const {
  return (canvas_ != nullptr) ? canvas_->Height() : 0;
}

bool DisplayRenderer::SetPixel(size_t x, size_t y, bool on) {
  return canvas_ != nullptr && canvas_->SetPixel(x, y, on);
}

bool DisplayRenderer::DrawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1,
                               bool on) {
  if (canvas_ == nullptr) {
    return false;
  }

  GfxRenderer renderer(canvas_);
  renderer.drawLine(x0, y0, x1, y1, on ? kPixelOn : kPixelOff);
  return true;
}

bool DisplayRenderer::DrawFastHLine(int16_t x, int16_t y, int16_t width,
                                    bool on) {
  if (canvas_ == nullptr || width <= 0) {
    return false;
  }

  GfxRenderer renderer(canvas_);
  renderer.drawFastHLine(x, y, width, on ? kPixelOn : kPixelOff);
  return true;
}

bool DisplayRenderer::DrawFastVLine(int16_t x, int16_t y, int16_t height,
                                    bool on) {
  if (canvas_ == nullptr || height <= 0) {
    return false;
  }

  GfxRenderer renderer(canvas_);
  renderer.drawFastVLine(x, y, height, on ? kPixelOn : kPixelOff);
  return true;
}

bool DisplayRenderer::DrawRect(size_t x, size_t y, size_t width, size_t height,
                               bool on) {
  if (canvas_ == nullptr || width == 0 || height == 0 || x >= Width() ||
      y >= Height()) {
    return false;
  }

  GfxRenderer renderer(canvas_);
  renderer.drawRect(static_cast<int16_t>(x), static_cast<int16_t>(y),
                    static_cast<int16_t>(width),
                    static_cast<int16_t>(height),
                    on ? kPixelOn : kPixelOff);
  return true;
}

bool DisplayRenderer::FillRect(int16_t x, int16_t y, int16_t width,
                               int16_t height, bool on) {
  if (canvas_ == nullptr || width <= 0 || height <= 0) {
    return false;
  }

  GfxRenderer renderer(canvas_);
  renderer.fillRect(x, y, width, height, on ? kPixelOn : kPixelOff);
  return true;
}

bool DisplayRenderer::DrawCircle(int16_t x, int16_t y, int16_t radius,
                                 bool on) {
  if (canvas_ == nullptr || radius < 0) {
    return false;
  }

  GfxRenderer renderer(canvas_);
  renderer.drawCircle(x, y, radius, on ? kPixelOn : kPixelOff);
  return true;
}

bool DisplayRenderer::FillCircle(int16_t x, int16_t y, int16_t radius,
                                 bool on) {
  if (canvas_ == nullptr || radius < 0) {
    return false;
  }

  GfxRenderer renderer(canvas_);
  renderer.fillCircle(x, y, radius, on ? kPixelOn : kPixelOff);
  return true;
}

bool DisplayRenderer::DrawEllipse(int16_t x, int16_t y, int16_t radius_w,
                                  int16_t radius_h, bool on) {
  if (canvas_ == nullptr || radius_w < 0 || radius_h < 0) {
    return false;
  }

  GfxRenderer renderer(canvas_);
  renderer.drawEllipse(x, y, radius_w, radius_h, on ? kPixelOn : kPixelOff);
  return true;
}

bool DisplayRenderer::FillEllipse(int16_t x, int16_t y, int16_t radius_w,
                                  int16_t radius_h, bool on) {
  if (canvas_ == nullptr || radius_w < 0 || radius_h < 0) {
    return false;
  }

  GfxRenderer renderer(canvas_);
  renderer.fillEllipse(x, y, radius_w, radius_h, on ? kPixelOn : kPixelOff);
  return true;
}

bool DisplayRenderer::DrawTriangle(int16_t x0, int16_t y0, int16_t x1,
                                   int16_t y1, int16_t x2, int16_t y2,
                                   bool on) {
  if (canvas_ == nullptr) {
    return false;
  }

  GfxRenderer renderer(canvas_);
  renderer.drawTriangle(x0, y0, x1, y1, x2, y2, on ? kPixelOn : kPixelOff);
  return true;
}

bool DisplayRenderer::FillTriangle(int16_t x0, int16_t y0, int16_t x1,
                                   int16_t y1, int16_t x2, int16_t y2,
                                   bool on) {
  if (canvas_ == nullptr) {
    return false;
  }

  GfxRenderer renderer(canvas_);
  renderer.fillTriangle(x0, y0, x1, y1, x2, y2, on ? kPixelOn : kPixelOff);
  return true;
}

bool DisplayRenderer::DrawRoundRect(int16_t x, int16_t y, int16_t width,
                                    int16_t height, int16_t radius, bool on) {
  if (canvas_ == nullptr || width <= 0 || height <= 0 || radius < 0) {
    return false;
  }

  GfxRenderer renderer(canvas_);
  renderer.drawRoundRect(x, y, width, height, radius,
                         on ? kPixelOn : kPixelOff);
  return true;
}

bool DisplayRenderer::FillRoundRect(int16_t x, int16_t y, int16_t width,
                                    int16_t height, int16_t radius, bool on) {
  if (canvas_ == nullptr || width <= 0 || height <= 0 || radius < 0) {
    return false;
  }

  GfxRenderer renderer(canvas_);
  renderer.fillRoundRect(x, y, width, height, radius,
                         on ? kPixelOn : kPixelOff);
  return true;
}

bool DisplayRenderer::DrawBitmap(const uint8_t *bitmap_data, size_t width,
                                 size_t height, size_t offset_x,
                                 size_t offset_y) {
  return canvas_ != nullptr &&
         canvas_->DrawPackedBitmap(bitmap_data, width, height, offset_x,
                                   offset_y);
}

bool DisplayRenderer::DrawMosaicBitmap(const uint8_t *bitmap_data, size_t width,
                                       size_t height, size_t offset_x,
                                       size_t offset_y,
                                       uint8_t block_size_px) {
  if (canvas_ == nullptr || bitmap_data == nullptr || width == 0 ||
      height == 0 || block_size_px == 0 || width > Width() ||
      height > Height() || offset_x + width > Width() ||
      offset_y + height > Height()) {
    return false;
  }

  const size_t block =
      static_cast<size_t>(std::max<uint8_t>(block_size_px, 1u));
  for (size_t src_y = 0; src_y < height; src_y += block) {
    for (size_t src_x = 0; src_x < width; src_x += block) {
      const size_t sample_x = std::min(src_x + block / 2u, width - 1u);
      const size_t sample_y = std::min(src_y + block / 2u, height - 1u);
      if (!ReadPackedPixel(bitmap_data, width, height, sample_x, sample_y)) {
        continue;
      }

      const size_t dst_x_begin = offset_x + src_x;
      const size_t dst_y_begin = offset_y + src_y;
      const size_t dst_x_end = std::min(dst_x_begin + block, offset_x + width);
      const size_t dst_y_end =
          std::min(dst_y_begin + block, offset_y + height);

      for (size_t dst_y = dst_y_begin; dst_y < dst_y_end; ++dst_y) {
        for (size_t dst_x = dst_x_begin; dst_x < dst_x_end; ++dst_x) {
          canvas_->SetPixel(dst_x, dst_y, true);
        }
      }
    }
  }

  return true;
}

bool DisplayRenderer::DrawMosaicTransition(const uint8_t *from_bitmap_data,
                                           const uint8_t *to_bitmap_data,
                                           size_t width, size_t height,
                                           float progress,
                                           uint8_t max_block_size_px) {
  if (canvas_ == nullptr || from_bitmap_data == nullptr ||
      to_bitmap_data == nullptr || width != Width() || height != Height()) {
    return false;
  }

  progress = std::clamp(progress, 0.0f, 1.0f);
  const uint8_t max_block =
      static_cast<uint8_t>(std::max<uint8_t>(max_block_size_px, 1u));

  const auto interpolated_block_size = [&](float local_progress,
                                           bool descending) {
    const float start = descending ? static_cast<float>(max_block) : 1.0f;
    const float end = descending ? 1.0f : static_cast<float>(max_block);
    return static_cast<uint8_t>(std::clamp(
        std::lround(start + (end - start) * std::clamp(local_progress, 0.0f, 1.0f)),
        1l, static_cast<long>(max_block)));
  };

  Clear();
  if (progress < 0.5f) {
    const float local_progress = progress * 2.0f;
    return DrawMosaicBitmap(from_bitmap_data, width, height, 0, 0,
                            interpolated_block_size(local_progress, false));
  }

  const float local_progress = (progress - 0.5f) * 2.0f;
  return DrawMosaicBitmap(to_bitmap_data, width, height, 0, 0,
                          interpolated_block_size(local_progress, true));
}

bool DisplayRenderer::DrawText(const char *text, int16_t x, int16_t y,
                               const DisplayTextStyle &style) {
  if (canvas_ == nullptr || text == nullptr) {
    return false;
  }

  GfxRenderer renderer(canvas_);
  renderer.ConfigureText(style);
  renderer.setCursor(x, y);
  return renderer.print(text) != 0;
}

int16_t DisplayRenderer::ScrollOffsetPx(int16_t content_width_px,
                                        int16_t available_width_px, TimeMs now,
                                        uint16_t pixels_per_second,
                                        TimeMs pause_ms) const {
  if (available_width_px <= 0 || pixels_per_second == 0) {
    return 0;
  }

  const int16_t overflow_px = content_width_px - available_width_px;
  if (overflow_px <= 0) {
    return 0;
  }

  const TimeMs scroll_duration_ms = std::max<TimeMs>(
      1, (static_cast<TimeMs>(overflow_px) * 1000u +
          (pixels_per_second - 1u)) /
             pixels_per_second);
  const TimeMs cycle_duration_ms = pause_ms + scroll_duration_ms + pause_ms;
  const TimeMs phase_ms = now % cycle_duration_ms;
  if (phase_ms < pause_ms) {
    return 0;
  }
  if (phase_ms >= (pause_ms + scroll_duration_ms)) {
    return overflow_px;
  }

  return static_cast<int16_t>(
      ((static_cast<uint32_t>(phase_ms - pause_ms) * pixels_per_second) /
       1000u));
}

bool DisplayRenderer::DrawScrollingText(const char *text, int16_t left_px,
                                        int16_t top_px,
                                        int16_t available_width_px, TimeMs now,
                                        const DisplayTextStyle &style,
                                        uint16_t pixels_per_second,
                                        TimeMs pause_ms) {
  if (canvas_ == nullptr || text == nullptr || available_width_px <= 0) {
    return false;
  }

  const DisplayTextBounds bounds = MeasureText(text, style);
  if (bounds.width == 0 || bounds.height == 0) {
    return false;
  }

  const int16_t scroll_offset_px =
      ScrollOffsetPx(static_cast<int16_t>(bounds.width), available_width_px,
                     now, pixels_per_second, pause_ms);
  const int16_t cursor_x =
      static_cast<int16_t>(left_px - bounds.x - scroll_offset_px);
  const int16_t cursor_y = static_cast<int16_t>(top_px - bounds.y);
  return DrawText(text, cursor_x, cursor_y, style);
}

DisplayTextBounds DisplayRenderer::MeasureText(const char *text,
                                               const DisplayTextStyle &style)
    const {
  if (canvas_ == nullptr || text == nullptr) {
    return {};
  }

  GfxRenderer renderer(canvas_);
  renderer.ConfigureText(style);

  int16_t x1 = 0;
  int16_t y1 = 0;
  uint16_t width = 0;
  uint16_t height = 0;
  renderer.getTextBounds(text, 0, 0, &x1, &y1, &width, &height);

  return DisplayTextBounds{
      .x = x1,
      .y = y1,
      .width = width,
      .height = height,
  };
}

size_t DisplayRenderer::AnimatedTextLength(TimeMs start_ms, TimeMs now,
                                           TimeMs duration_ms,
                                           const char *text) const {
  if (text == nullptr) {
    return 0;
  }

  const size_t text_len = std::strlen(text);
  if (text_len == 0) {
    return 0;
  }
  if (duration_ms == 0) {
    return text_len;
  }
  if (now <= start_ms) {
    return 1;
  }

  const TimeMs elapsed = now - start_ms;
  if (elapsed >= duration_ms) {
    return text_len;
  }

  const uint32_t scaled =
      static_cast<uint32_t>(elapsed) * static_cast<uint32_t>(text_len);
  const size_t visible_chars = static_cast<size_t>(
      (scaled + duration_ms - 1) / duration_ms);
  return std::clamp<size_t>(visible_chars, static_cast<size_t>(1), text_len);
}
