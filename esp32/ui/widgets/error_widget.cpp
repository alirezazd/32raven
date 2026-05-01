#include "error_widget.hpp"

#include <algorithm>
#include <cstdio>

#include "system.hpp"

extern "C" {
#include "freertos/FreeRTOS.h"
}

namespace {

constexpr DisplayTextStyle kTitleStyle{
    .scale = 1,
    .font = DisplayTextStyle::Font::kDefault,
};
constexpr DisplayTextStyle kBodyStyle{
    .scale = 1,
    .font = DisplayTextStyle::Font::kDefault,
};
constexpr TimeMs kPanicBlinkPeriodMs = 300;
constexpr int16_t kInsetX = 1;
constexpr int16_t kTopY = 0;
constexpr int16_t kRibbonGapPx = 4;
constexpr int16_t kRibbonStripePitchPx = 12;
constexpr int16_t kRibbonStripeWidthPx = 5;
constexpr uint16_t kRibbonPixelsPerSecond = 28;
constexpr char kRecoverableHint[] = ". Hold button0 to enter DFU.";

void DrawConstructionRibbon(DisplayRenderer &renderer, int16_t top,
                            int16_t height, TimeMs now) {
  if (height <= 0) {
    return;
  }

  const int16_t width = static_cast<int16_t>(renderer.Width());
  if (width <= 0) {
    return;
  }

  renderer.FillRect(0, top, width, height, true);

  const int16_t phase_px = static_cast<int16_t>(
      ((static_cast<uint32_t>(now) * kRibbonPixelsPerSecond) / 1000u) %
      kRibbonStripePitchPx);
  for (int16_t y = 0; y < height; ++y) {
    for (int16_t x = 0; x < width; ++x) {
      const int16_t pattern_pos =
          static_cast<int16_t>((x + y + phase_px) % kRibbonStripePitchPx);
      if (pattern_pos < kRibbonStripeWidthPx) {
        renderer.SetPixel(static_cast<size_t>(x), static_cast<size_t>(top + y),
                          false);
      }
    }
  }
}

}  // namespace

void ErrorWidget::SetErrorCode(ErrorCode code) {
  taskENTER_CRITICAL(&lock_);
  error_code_ = code;
  taskEXIT_CRITICAL(&lock_);
}

void ErrorWidget::SetRecoverable(bool recoverable) {
  taskENTER_CRITICAL(&lock_);
  recoverable_ = recoverable;
  taskEXIT_CRITICAL(&lock_);
}

ErrorCode ErrorWidget::CurrentErrorCode() const {
  taskENTER_CRITICAL(&lock_);
  const ErrorCode code = error_code_;
  taskEXIT_CRITICAL(&lock_);
  return code;
}

bool ErrorWidget::CurrentRecoverable() const {
  taskENTER_CRITICAL(&lock_);
  const bool recoverable = recoverable_;
  taskEXIT_CRITICAL(&lock_);
  return recoverable;
}

void ErrorWidget::OnEnter(WidgetContext &ctx) {
  Render(ctx, Sys().Timebase().NowMs());
}

void ErrorWidget::OnStep(WidgetContext &ctx, TimeMs now) { Render(ctx, now); }

void ErrorWidget::Render(WidgetContext &ctx, TimeMs now) {
  if (ctx.renderer == nullptr) {
    return;
  }

  DisplayRenderer &renderer = *ctx.renderer;
  const ErrorCode error_code = CurrentErrorCode();
  const char *error_message = GetMessage(error_code);
  const bool recoverable = CurrentRecoverable();

  char error_line[192]{};
  std::snprintf(error_line, sizeof(error_line), "%03lu: %s%s",
                static_cast<unsigned long>(error_code),
                error_message != nullptr ? error_message : "",
                recoverable ? kRecoverableHint : "");

  const DisplayTextBounds title_bounds =
      renderer.MeasureText("PANIC!", kTitleStyle);
  const DisplayTextBounds error_bounds =
      renderer.MeasureText(error_line, kBodyStyle);

  const int16_t line1_top = kTopY;
  const int16_t title_left =
      std::max<int16_t>(0, (static_cast<int16_t>(renderer.Width()) -
                            static_cast<int16_t>(title_bounds.width)) /
                               2);
  const int16_t line2_top =
      std::max<int16_t>(0, static_cast<int16_t>(renderer.Height()) -
                               static_cast<int16_t>(error_bounds.height));
  const int16_t error_width =
      static_cast<int16_t>(renderer.Width() - static_cast<size_t>(kInsetX));
  const int16_t ribbon_top = static_cast<int16_t>(
      line1_top + static_cast<int16_t>(title_bounds.height) + kRibbonGapPx);
  const int16_t ribbon_bottom = static_cast<int16_t>(line2_top - kRibbonGapPx);
  const int16_t ribbon_height =
      std::max<int16_t>(0, static_cast<int16_t>(ribbon_bottom - ribbon_top));
  const bool show_title = ((now / kPanicBlinkPeriodMs) % 2u) == 0u;

  renderer.Clear();
  if (show_title) {
    renderer.DrawText(
        "PANIC!", static_cast<int16_t>(title_left - title_bounds.x),
        static_cast<int16_t>(line1_top - title_bounds.y), kTitleStyle);
  }
  DrawConstructionRibbon(renderer, ribbon_top, ribbon_height, now);
  if (error_bounds.width > 0 && error_bounds.height > 0) {
    renderer.DrawScrollingText(
        error_line, kInsetX, line2_top,
        static_cast<int16_t>(std::max<int16_t>(0, error_width)), now,
        kBodyStyle);
  }
}
