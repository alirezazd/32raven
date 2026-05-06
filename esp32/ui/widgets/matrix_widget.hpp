#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "ui.hpp"

class MatrixWidget : public IWidget {
 public:
  const char *Name() const override { return "matrix"; }

  void OnEnter(WidgetContext &ctx) override;
  void OnStep(WidgetContext &ctx, TimeMs now) override;

 private:
  static constexpr size_t kMaxColumns = 16;
  static constexpr size_t kMaxGlyphsPerColumn = 12;

  struct Column {
    bool scroll_up = false;
    uint8_t offset_px = 0;
    std::array<char, kMaxGlyphsPerColumn> glyphs{};
  };

  void ConfigureLayout(DisplayRenderer &renderer);
  void InitializeColumns();
  void AdvanceColumns();
  void Render(WidgetContext &ctx) const;

  std::array<Column, kMaxColumns> columns_{};
  DisplayTextBounds glyph_bounds_{};
  TimeMs next_step_ms_ = 0;
  size_t column_count_ = 0;
  size_t glyph_count_ = 0;
  int16_t glyph_width_px_ = 0;
  int16_t glyph_height_px_ = 0;
  int16_t glyph_step_y_px_ = 0;
  int16_t column_pitch_px_ = 0;
  int16_t left_margin_px_ = 0;
};
