#include "matrix_widget.hpp"

#include <algorithm>

#include "system.hpp"
#include "timebase.hpp"

extern "C" {
#include "esp_random.h"
}

namespace {

constexpr DisplayTextStyle kTextStyle{
    .scale = 1,
};
constexpr char kSampleGlyph[] = "0";
constexpr char kGlyphCharset[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";
constexpr TimeMs kStepPeriodMs = 16;
constexpr int16_t kColumnGapPx = 3;
constexpr int16_t kDefaultGlyphWidthPx = 6;
constexpr int16_t kDefaultGlyphHeightPx = 8;

char RandomGlyph() {
  constexpr size_t kCharsetSize = sizeof(kGlyphCharset) - 1;
  return kGlyphCharset[esp_random() % kCharsetSize];
}

bool RandomDirectionUp() { return (esp_random() & 0x1u) != 0u; }

}  // namespace

void MatrixWidget::OnEnter(WidgetContext &ctx) {
  if (ctx.renderer == nullptr) {
    return;
  }

  ConfigureLayout(*ctx.renderer);
  InitializeColumns();
  Render(ctx);
  next_step_ms_ = TimeAfter(Sys().Timebase().NowMs(), kStepPeriodMs);
}

void MatrixWidget::OnStep(WidgetContext &ctx, TimeMs now) {
  if (ctx.display == nullptr || column_count_ == 0 || glyph_count_ == 0) {
    return;
  }

  bool needs_render = false;
  uint8_t iterations = 0;
  while (TimeReached(now, next_step_ms_)) {
    AdvanceColumns();
    next_step_ms_ = TimeAfter(next_step_ms_, kStepPeriodMs);
    needs_render = true;
    ++iterations;
    if (iterations >= 8) {
      next_step_ms_ = TimeAfter(now, kStepPeriodMs);
      break;
    }
  }

  if (needs_render) {
    Render(ctx);
  }
}

void MatrixWidget::ConfigureLayout(DisplayRenderer &renderer) {
  DisplayTextBounds bounds = renderer.MeasureText(kSampleGlyph, kTextStyle);
  if (bounds.width == 0 || bounds.height == 0) {
    bounds.x = 0;
    bounds.y = 0;
    bounds.width = kDefaultGlyphWidthPx;
    bounds.height = kDefaultGlyphHeightPx;
  }

  glyph_bounds_ = bounds;
  glyph_width_px_ = std::max<int16_t>(kDefaultGlyphWidthPx,
                                      static_cast<int16_t>(bounds.width));
  glyph_height_px_ = std::max<int16_t>(kDefaultGlyphHeightPx,
                                       static_cast<int16_t>(bounds.height));
  glyph_step_y_px_ = glyph_height_px_;
  column_pitch_px_ = static_cast<int16_t>(glyph_width_px_ + kColumnGapPx);

  const size_t available_columns =
      (renderer.Width() + static_cast<size_t>(kColumnGapPx)) /
      static_cast<size_t>(column_pitch_px_);
  column_count_ = std::clamp<size_t>(available_columns, 1u,
                                     static_cast<size_t>(kMaxColumns));

  const int16_t total_width = static_cast<int16_t>(
      (static_cast<int16_t>(column_count_) * glyph_width_px_) +
      (static_cast<int16_t>(column_count_ - 1u) * kColumnGapPx));
  left_margin_px_ = std::max<int16_t>(
      0, (static_cast<int16_t>(renderer.Width()) - total_width) / 2);

  const size_t required_glyphs =
      (renderer.Height() / static_cast<size_t>(glyph_step_y_px_)) + 3u;
  glyph_count_ = std::clamp<size_t>(required_glyphs, 3u,
                                    static_cast<size_t>(kMaxGlyphsPerColumn));
}

void MatrixWidget::InitializeColumns() {
  for (size_t column_index = 0; column_index < column_count_; ++column_index) {
    Column &column = columns_[column_index];
    column.scroll_up = RandomDirectionUp();
    column.offset_px = static_cast<uint8_t>(
        esp_random() % static_cast<uint32_t>(glyph_step_y_px_));

    for (size_t glyph_index = 0; glyph_index < glyph_count_; ++glyph_index) {
      column.glyphs[glyph_index] = RandomGlyph();
    }
  }
}

void MatrixWidget::AdvanceColumns() {
  for (size_t column_index = 0; column_index < column_count_; ++column_index) {
    Column &column = columns_[column_index];
    column.offset_px = static_cast<uint8_t>(column.offset_px + 1u);

    if (column.offset_px < glyph_step_y_px_) {
      continue;
    }

    column.offset_px = 0;
    if (column.scroll_up) {
      for (size_t glyph_index = 0; glyph_index + 1u < glyph_count_;
           ++glyph_index) {
        column.glyphs[glyph_index] = column.glyphs[glyph_index + 1u];
      }
      column.glyphs[glyph_count_ - 1u] = RandomGlyph();
      continue;
    }

    for (size_t glyph_index = glyph_count_ - 1u; glyph_index > 0u;
         --glyph_index) {
      column.glyphs[glyph_index] = column.glyphs[glyph_index - 1u];
    }
    column.glyphs[0] = RandomGlyph();
  }
}

void MatrixWidget::Render(WidgetContext &ctx) const {
  if (ctx.renderer == nullptr) {
    return;
  }

  DisplayRenderer &renderer = *ctx.renderer;
  renderer.Clear();

  for (size_t column_index = 0; column_index < column_count_; ++column_index) {
    const Column &column = columns_[column_index];
    const int16_t column_left = static_cast<int16_t>(
        left_margin_px_ +
        (static_cast<int16_t>(column_index) * column_pitch_px_));
    const int16_t cursor_x =
        static_cast<int16_t>(column_left - glyph_bounds_.x);

    for (size_t glyph_index = 0; glyph_index < glyph_count_; ++glyph_index) {
      const int16_t base_top = static_cast<int16_t>(
          (-glyph_step_y_px_) +
          (static_cast<int16_t>(glyph_index) * glyph_step_y_px_));
      const int16_t glyph_top = static_cast<int16_t>(
          base_top + (column.scroll_up ? -column.offset_px : column.offset_px));
      if (glyph_top >= static_cast<int16_t>(renderer.Height()) ||
          (glyph_top + glyph_height_px_) <= 0) {
        continue;
      }

      const int16_t cursor_y =
          static_cast<int16_t>(glyph_top - glyph_bounds_.y);
      const char text[2] = {column.glyphs[glyph_index], '\0'};
      renderer.DrawText(text, cursor_x, cursor_y, kTextStyle);
    }
  }
}
