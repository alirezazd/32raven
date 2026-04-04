#include "main_ui_widget.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "bitmap/gear0_bitmap.hpp"
#include "bitmap/gear1_bitmap.hpp"
#include "system.hpp"

extern "C" {
#include "freertos/FreeRTOS.h"
}

namespace {

constexpr DisplayTextStyle kStatusStyle{
    .scale = 1,
    .font = DisplayTextStyle::Font::kDefault,
};
constexpr float kPi = 3.14159265358979323846f;
constexpr int16_t kTextInsetX = 1;
constexpr int16_t kTextInsetY = 0;
constexpr TimeMs kDotStepPeriodMs = 300;
constexpr TimeMs kEntryTextAnimationDurationMs = 200;
constexpr TimeMs kGearRotationPeriodMs = 1600;
constexpr TimeMs kLargeGearRotationPeriodMs = kGearRotationPeriodMs * 2;
constexpr char kServingStatus[] = "Serving";
constexpr char kDfuStatus[] = "DFU";
portMUX_TYPE g_main_ui_widget_lock = portMUX_INITIALIZER_UNLOCKED;

bool CopyBoundedString(char *dst, size_t dst_size, const char *src) {
  if (dst == nullptr || dst_size == 0) {
    return false;
  }

  const char *safe_src = (src != nullptr) ? src : "";
  const size_t src_len = strnlen(safe_src, dst_size - 1);
  std::memcpy(dst, safe_src, src_len);
  dst[src_len] = '\0';
  return true;
}

uint8_t DotCount(TimeMs now) {
  const uint8_t phase = static_cast<uint8_t>((now / kDotStepPeriodMs) % 4u);
  return (phase == 3u) ? 0u : static_cast<uint8_t>(phase + 1u);
}

bool IsServingStatus(const char *status) {
  return status != nullptr && std::strcmp(status, kServingStatus) == 0;
}

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

float RotationAngle(TimeMs now, TimeMs period_ms, bool clockwise) {
  const TimeMs cycle_ms = (period_ms > 0) ? period_ms : 1;
  const float cycle_pos =
      static_cast<float>(now % cycle_ms) / static_cast<float>(cycle_ms);
  const float angle = 2.0f * kPi * cycle_pos;
  return clockwise ? angle : -angle;
}

void DrawRotatingBitmap(DisplayRenderer &renderer, const uint8_t *bitmap_data,
                        size_t bitmap_width, size_t bitmap_height, TimeMs now,
                        size_t offset_x, size_t offset_y, bool clockwise,
                        TimeMs period_ms = kGearRotationPeriodMs) {
  if (bitmap_data == nullptr || bitmap_width == 0 || bitmap_height == 0) {
    return;
  }

  const float angle = RotationAngle(now, period_ms, clockwise);
  const float cos_angle = std::cos(angle);
  const float sin_angle = std::sin(angle);
  const float center_x = (static_cast<float>(bitmap_width) - 1.0f) * 0.5f;
  const float center_y = (static_cast<float>(bitmap_height) - 1.0f) * 0.5f;

  for (size_t dst_y = 0; dst_y < bitmap_height; ++dst_y) {
    for (size_t dst_x = 0; dst_x < bitmap_width; ++dst_x) {
      const float rel_x = static_cast<float>(dst_x) - center_x;
      const float rel_y = static_cast<float>(dst_y) - center_y;
      const long src_x =
          std::lround((cos_angle * rel_x) + (sin_angle * rel_y) + center_x);
      const long src_y =
          std::lround((-sin_angle * rel_x) + (cos_angle * rel_y) + center_y);
      if (src_x < 0 || src_y < 0) {
        continue;
      }
      if (!ReadPackedPixel(bitmap_data, bitmap_width, bitmap_height,
                           static_cast<size_t>(src_x),
                           static_cast<size_t>(src_y))) {
        continue;
      }
      renderer.SetPixel(offset_x + dst_x, offset_y + dst_y, true);
    }
  }
}

bool ShouldAnimateEntryText(const char *status) {
  return status != nullptr && (std::strcmp(status, kServingStatus) == 0 ||
                               std::strcmp(status, kDfuStatus) == 0);
}

}  // namespace

void MainUiWidget::SetStatus(const char *status) {
  taskENTER_CRITICAL(&g_main_ui_widget_lock);
  CopyBoundedString(status_.data(), status_.size(), status);
  taskEXIT_CRITICAL(&g_main_ui_widget_lock);
}

std::array<char, MainUiWidget::kStatusBufferSize> MainUiWidget::CurrentStatus()
    const {
  std::array<char, kStatusBufferSize> status{};
  taskENTER_CRITICAL(&g_main_ui_widget_lock);
  CopyBoundedString(status.data(), status.size(), status_.data());
  taskEXIT_CRITICAL(&g_main_ui_widget_lock);
  return status;
}

void MainUiWidget::OnEnter(WidgetContext &ctx) {
  const TimeMs now = Sys().Timebase().NowMs();
  has_rendered_ = false;
  text_animation_active_ = false;
  text_animation_start_ms_ = now;
  last_gear_step_ms_ = now;
  gear_animation_ms_ = 0;
  last_dot_count_ = 0;
  last_status_.fill('\0');

  if (ctx.display == nullptr || ctx.renderer == nullptr) {
    return;
  }

  const auto status = CurrentStatus();
  BeginTextPhase(ctx, now, status.data());
}

void MainUiWidget::OnStep(WidgetContext &ctx, TimeMs now) {
  if (ctx.display == nullptr || ctx.renderer == nullptr) {
    return;
  }

  const auto status = CurrentStatus();
  AdvanceGearAnimation(ctx, now, IsServingStatus(status.data()));

  if (std::strncmp(last_status_.data(), status.data(), last_status_.size()) !=
      0) {
    BeginTextPhase(ctx, now, status.data());
    return;
  }

  const bool serving_animation_active = IsServingStatus(status.data());
  if (!has_rendered_ || text_animation_active_ ||
      last_dot_count_ != DotCount(now) || serving_animation_active) {
    RenderTextStatus(ctx, now, status.data());
  }
}

void MainUiWidget::BeginTextPhase(WidgetContext &ctx, TimeMs now,
                                  const char *status) {
  CopyStatus(status);
  text_animation_start_ms_ = now;
  last_gear_step_ms_ = now;
  text_animation_active_ = ShouldAnimateEntryText(status);
  RenderTextStatus(ctx, now, status);
}

void MainUiWidget::CopyStatus(const char *status) {
  std::snprintf(last_status_.data(), last_status_.size(), "%s",
                status != nullptr ? status : "");
}

void MainUiWidget::AdvanceGearAnimation(WidgetContext &ctx, TimeMs now,
                                        bool active) {
  const TimeMs previous_step_ms = last_gear_step_ms_;
  last_gear_step_ms_ = now;
  if (!active || previous_step_ms == 0 || now <= previous_step_ms) {
    return;
  }

  TimeMs step_ms = now - previous_step_ms;
  if (ctx.display != nullptr) {
    step_ms = std::min(step_ms, ctx.display->GetFrameIntervalMs());
  }
  gear_animation_ms_ += step_ms;
}

void MainUiWidget::RenderTextStatus(WidgetContext &ctx, TimeMs now,
                                    const char *status) {
  if (ctx.display == nullptr || ctx.renderer == nullptr || status == nullptr) {
    return;
  }

  DisplayRenderer &renderer = *ctx.renderer;
  const size_t visible_chars =
      text_animation_active_
          ? renderer.AnimatedTextLength(text_animation_start_ms_, now,
                                        kEntryTextAnimationDurationMs, status)
          : std::strlen(status);
  const uint8_t dot_count = text_animation_active_ ? 0 : DotCount(now);
  char animated_status[kStatusBufferSize]{};
  if (text_animation_active_) {
    const size_t animated_chars =
        std::min(visible_chars, sizeof(animated_status) - 1);
    std::snprintf(animated_status, sizeof(animated_status), "%.*s",
                  static_cast<int>(animated_chars), status);
  } else {
    std::snprintf(animated_status, sizeof(animated_status), "%s", status);
  }

  char status_line[kStatusLineBufferSize]{};
  const int written =
      std::snprintf(status_line, sizeof(status_line), ">%s", animated_status);
  if (written < 0) {
    return;
  }

  const size_t base_len =
      std::min<size_t>(static_cast<size_t>(written), sizeof(status_line) - 1);
  for (uint8_t index = 0;
       index < dot_count && (base_len + index + 1) < sizeof(status_line);
       ++index) {
    status_line[base_len + index] = '.';
    status_line[base_len + index + 1] = '\0';
  }

  const DisplayTextBounds status_bounds =
      renderer.MeasureText(status_line, kStatusStyle);
  if (status_bounds.width == 0 || status_bounds.height == 0) {
    return;
  }

  const int16_t status_top = kTextInsetY;
  const int16_t status_y = static_cast<int16_t>(status_top - status_bounds.y);

  renderer.Clear();
  renderer.DrawText(status_line, kTextInsetX, status_y, kStatusStyle);
  if (IsServingStatus(status)) {
    const int16_t center_line_x = static_cast<int16_t>(renderer.Width() / 2);
    const int16_t right_gear_x = center_line_x;
    const size_t right_gear_y =
        (renderer.Height() > gear1_bitmap::kVisibleHeight)
            ? (renderer.Height() - gear1_bitmap::kVisibleHeight)
            : 0;
    if (right_gear_x >= 0 &&
        (static_cast<size_t>(right_gear_x) + gear1_bitmap::kVisibleWidth) <=
            renderer.Width() &&
        (right_gear_y + gear1_bitmap::kVisibleHeight) <= renderer.Height()) {
      DrawRotatingBitmap(
          renderer, gear1_bitmap::kBitmapData.data(),
          gear1_bitmap::kVisibleWidth, gear1_bitmap::kVisibleHeight,
          gear_animation_ms_,
          static_cast<size_t>(right_gear_x),
          right_gear_y, true);
    }

    const int16_t left_gear_x =
        center_line_x - static_cast<int16_t>(gear0_bitmap::kVisibleWidth) + 1;
    const size_t left_gear_y =
        (renderer.Height() > gear0_bitmap::kVisibleHeight)
            ? (renderer.Height() - gear0_bitmap::kVisibleHeight)
            : 0;
    if (left_gear_x >= 0 &&
        (left_gear_y + gear0_bitmap::kVisibleHeight) <= renderer.Height()) {
      DrawRotatingBitmap(renderer, gear0_bitmap::kBitmapData.data(),
                         gear0_bitmap::kVisibleWidth,
                         gear0_bitmap::kVisibleHeight, gear_animation_ms_,
                         static_cast<size_t>(left_gear_x), left_gear_y, false,
                         kLargeGearRotationPeriodMs);
    }
  }

  if (text_animation_active_ && visible_chars >= std::strlen(status)) {
    text_animation_active_ = false;
  }
  last_dot_count_ = dot_count;
  has_rendered_ = true;
}
