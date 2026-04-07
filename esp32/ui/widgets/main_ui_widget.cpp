#include "main_ui_widget.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "bitmap/chip_bitmap.hpp"
#include "bitmap/chip_verify_bitmap.hpp"
#include "bitmap/gear0_bitmap.hpp"
#include "bitmap/gear1_bitmap.hpp"
#include "bitmap/magnifying_glass_bitmap.hpp"
#include "bitmap/mavlink0_bitmap.hpp"
#include "bitmap/mavlink1_bitmap.hpp"
#include "bitmap/mavlink2_bitmap.hpp"
#include "bitmap/mavlink3_bitmap.hpp"
#include "bitmap/pc_bitmap.hpp"
#include "bitmap/wifi_bitmap.hpp"
#include "system.hpp"

extern "C" {
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
}

namespace {

using WidgetMode = MainUiWidget::Mode;

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
constexpr char kMavlinkWifiStatus[] = "MAVLink";
constexpr size_t kDfuIconLeftX = 0;
constexpr size_t kDfuIconRightInsetX = 0;
constexpr size_t kDfuWifiRightInsetX = 0;
constexpr size_t kDfuWifiTopY = 0;
constexpr int16_t kDfuWifiWarningGapPx = 1;
constexpr int16_t kDfuLinkGlyphGapPx = 1;
constexpr int16_t kDfuCredentialLineGapPx = 2;
constexpr TimeMs kMavlinkIconFramePeriodMs = 250;
constexpr char kProgrammingViewportSample[] = "1234567";
constexpr TimeMs kDefaultDfuLinkStepPeriodMs = 16;
constexpr uint16_t kDfuLinkSubpixelScale = 256;
constexpr uint16_t kDfuLinkPixelsPerSecond = 48;
constexpr uint16_t kMavlinkPacketSubpixelScale = 256;
constexpr uint16_t kMavlinkPacketPixelsPerSecond = 36;
constexpr uint16_t kVerifyMagnifierSubpixelScale = 256;
constexpr uint16_t kVerifyMagnifierPixelsPerSecond = 12;
constexpr int16_t kVerifyMagnifierLensCenterX = 8;
constexpr int16_t kVerifyMagnifierLensCenterY = 8;
constexpr int16_t kVerifyMagnifierLensRadiusPx = 8;
constexpr int16_t kVerifyDigitGlyphWidthPx = 5;
constexpr int16_t kVerifyDigitGlyphHeightPx = 7;
constexpr int16_t kVerifyDigitGlyphGapPx = 0;
constexpr DisplayTextStyle kDfuSleepStyle{
    .scale = 1,
    .font = DisplayTextStyle::Font::kDefault,
};
portMUX_TYPE g_main_ui_widget_lock = portMUX_INITIALIZER_UNLOCKED;

constexpr std::array<uint8_t, kVerifyDigitGlyphHeightPx> kVerifyDigitZero = {
    0b01110, 0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b01110,
};
constexpr std::array<uint8_t, kVerifyDigitGlyphHeightPx> kVerifyDigitOne = {
    0b00100, 0b01100, 0b00100, 0b00100, 0b00100, 0b00100, 0b01110,
};

uint8_t DotCount(TimeMs now) {
  const uint8_t phase = static_cast<uint8_t>((now / kDotStepPeriodMs) % 4u);
  return (phase == 3u) ? 0u : static_cast<uint8_t>(phase + 1u);
}

char RandomBinaryGlyph() { return ((esp_random() & 0x1u) != 0u) ? '1' : '0'; }

const char *StatusTextForMode(WidgetMode mode) {
  switch (mode) {
    case WidgetMode::kBooting:
      return "Booting";
    case WidgetMode::kServing:
      return kServingStatus;
    case WidgetMode::kDfuDisconnected:
    case WidgetMode::kDfuIdleConnected:
      return kDfuStatus;
    case WidgetMode::kMavlinkWifiDisconnected:
    case WidgetMode::kMavlinkWifiConnected:
      return kMavlinkWifiStatus;
    case WidgetMode::kProgramming:
      return "Programming";
    case WidgetMode::kVerifying:
      return "Verifying";
    default:
      return "";
  }
}

bool IsServingMode(WidgetMode mode) { return mode == WidgetMode::kServing; }

bool IsDfuVisualMode(WidgetMode mode) {
  return mode == WidgetMode::kDfuDisconnected ||
         mode == WidgetMode::kDfuIdleConnected ||
         mode == WidgetMode::kMavlinkWifiDisconnected ||
         mode == WidgetMode::kMavlinkWifiConnected ||
         mode == WidgetMode::kProgramming || mode == WidgetMode::kVerifying;
}

bool IsScrollableProgressMode(WidgetMode mode) {
  return mode == WidgetMode::kProgramming || mode == WidgetMode::kVerifying;
}

bool IsWifiCredentialsMode(WidgetMode mode) {
  return mode == WidgetMode::kDfuDisconnected ||
         mode == WidgetMode::kMavlinkWifiDisconnected;
}

bool IsMavlinkMode(WidgetMode mode) {
  return mode == WidgetMode::kMavlinkWifiDisconnected ||
         mode == WidgetMode::kMavlinkWifiConnected;
}

struct IconAsset {
  const uint8_t *data = nullptr;
  size_t width = 0;
  size_t height = 0;
};

IconAsset LeftIconForMode(WidgetMode mode, TimeMs now) {
  if (!IsMavlinkMode(mode)) {
    return {
        .data = pc_bitmap::kBitmapData.data(),
        .width = pc_bitmap::kVisibleWidth,
        .height = pc_bitmap::kVisibleHeight,
    };
  }

  switch ((now / kMavlinkIconFramePeriodMs) % 4u) {
    case 1:
      return {
          .data = mavlink1_bitmap::kBitmapData.data(),
          .width = mavlink1_bitmap::kVisibleWidth,
          .height = mavlink1_bitmap::kVisibleHeight,
      };
    case 2:
      return {
          .data = mavlink2_bitmap::kBitmapData.data(),
          .width = mavlink2_bitmap::kVisibleWidth,
          .height = mavlink2_bitmap::kVisibleHeight,
      };
    case 3:
      return {
          .data = mavlink3_bitmap::kBitmapData.data(),
          .width = mavlink3_bitmap::kVisibleWidth,
          .height = mavlink3_bitmap::kVisibleHeight,
      };
    case 0:
    default:
      return {
          .data = mavlink0_bitmap::kBitmapData.data(),
          .width = mavlink0_bitmap::kVisibleWidth,
          .height = mavlink0_bitmap::kVisibleHeight,
      };
  }
}

bool ShouldAnimateEntryText(WidgetMode mode) {
  return mode == WidgetMode::kServing || mode == WidgetMode::kDfuDisconnected ||
         mode == WidgetMode::kDfuIdleConnected ||
         mode == WidgetMode::kMavlinkWifiDisconnected ||
         mode == WidgetMode::kMavlinkWifiConnected;
}

template <size_t N>
void DrawMaskedVerifyDigits(DisplayRenderer &renderer, int16_t center_x,
                            int16_t center_y, const std::array<char, N> &digits,
                            int16_t lens_center_x, int16_t lens_center_y,
                            bool inside_lens, int16_t glyph_width,
                            int16_t glyph_height, int16_t glyph_gap,
                            const uint8_t *zero_pattern,
                            const uint8_t *one_pattern) {
  const int16_t digit_count = static_cast<int16_t>(digits.size());
  const int16_t total_width = static_cast<int16_t>(
      (digit_count * glyph_width) + ((digit_count - 1) * glyph_gap));
  const int16_t top = static_cast<int16_t>(center_y - (glyph_height / 2));
  const int16_t left = static_cast<int16_t>(center_x - (total_width / 2));
  const int16_t lens_radius_sq = static_cast<int16_t>(
      kVerifyMagnifierLensRadiusPx * kVerifyMagnifierLensRadiusPx);

  for (size_t digit_index = 0; digit_index < digits.size(); ++digit_index) {
    const uint8_t *pattern =
        (digits[digit_index] == '1') ? one_pattern : zero_pattern;
    const int16_t glyph_left = static_cast<int16_t>(
        left + static_cast<int16_t>(digit_index * (glyph_width + glyph_gap)));

    for (int16_t row = 0; row < glyph_height; ++row) {
      for (int16_t col = 0; col < glyph_width; ++col) {
        const uint8_t row_mask = pattern[row];
        const uint8_t bit = static_cast<uint8_t>(1u << (glyph_width - 1 - col));
        if ((row_mask & bit) == 0u) {
          continue;
        }

        const int16_t px = static_cast<int16_t>(glyph_left + col);
        const int16_t py = static_cast<int16_t>(top + row);
        const int16_t dx = static_cast<int16_t>(px - lens_center_x);
        const int16_t dy = static_cast<int16_t>(py - lens_center_y);
        const bool pixel_in_lens = ((dx * dx) + (dy * dy)) <= lens_radius_sq;
        if (inside_lens ? !pixel_in_lens : pixel_in_lens) {
          continue;
        }
        if (px < 0 || py < 0 || static_cast<size_t>(px) >= renderer.Width() ||
            static_cast<size_t>(py) >= renderer.Height()) {
          continue;
        }

        renderer.SetPixel(static_cast<size_t>(px), static_cast<size_t>(py),
                          true);
      }
    }
  }
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

int16_t VerifyMagnifierLeftOverlapPx() {
  return std::max<int16_t>(
      1, static_cast<int16_t>(chip_verify_bitmap::kVisibleWidth / 2));
}

int16_t VerifyMagnifierRightLimitPx() {
  return std::max<int16_t>(
      1, static_cast<int16_t>((chip_verify_bitmap::kVisibleWidth * 65) / 100));
}

int16_t VerifyMagnifierTravelPx() {
  const int16_t left_overlap_px = VerifyMagnifierLeftOverlapPx();
  const int16_t right_limit_px = VerifyMagnifierRightLimitPx();
  return std::max<int16_t>(
      0, static_cast<int16_t>(magnifying_glass_bitmap::kVisibleWidth) +
             right_limit_px - left_overlap_px);
}

}  // namespace

void MainUiWidget::SetMode(Mode mode) {
  taskENTER_CRITICAL(&g_main_ui_widget_lock);
  mode_ = mode;
  taskEXIT_CRITICAL(&g_main_ui_widget_lock);
}

MainUiWidget::Mode MainUiWidget::CurrentMode() const {
  taskENTER_CRITICAL(&g_main_ui_widget_lock);
  const Mode mode = mode_;
  taskEXIT_CRITICAL(&g_main_ui_widget_lock);
  return mode;
}

void MainUiWidget::OnEnter(WidgetContext &ctx) {
  const TimeMs now = Sys().Timebase().NowMs();
  has_rendered_ = false;
  text_animation_active_ = false;
  text_animation_start_ms_ = now;
  last_gear_step_ms_ = now;
  gear_animation_ms_ = 0;
  last_dot_count_ = 0;
  dfu_link_offset_px_ = 0;
  dfu_link_last_step_ms_ = now;
  dfu_link_subpixel_offset_ = 0;
  dfu_link_initialized_ = false;
  mavlink_packet_last_step_ms_ = now;
  ResetMavlinkPacketAnimation();
  ResetVerifyMagnifierAnimation(now);
  last_mode_ = CurrentMode();

  if (ctx.ui == nullptr || ctx.renderer == nullptr) {
    return;
  }

  BeginTextPhase(ctx, now, last_mode_);
}

void MainUiWidget::OnStep(WidgetContext &ctx, TimeMs now) {
  if (ctx.ui == nullptr || ctx.renderer == nullptr) {
    return;
  }

  const Mode mode = CurrentMode();
  AdvanceGearAnimation(ctx, now, IsServingMode(mode));
  const bool dfu_binary_link_active = mode == Mode::kProgramming;
  const bool progress_status_scroll_active = IsScrollableProgressMode(mode);
  const bool dfu_credentials_animation_active = IsWifiCredentialsMode(mode);
  const TimeMs dfu_link_step_period_ms =
      (ctx.ui != nullptr && ctx.ui->GetFrameIntervalMs() > 0)
          ? ctx.ui->GetFrameIntervalMs()
          : kDefaultDfuLinkStepPeriodMs;
  const bool verify_magnifier_changed = AdvanceVerifyMagnifierAnimation(
      now, mode == Mode::kVerifying, dfu_link_step_period_ms);
  const bool mavlink_packet_changed = AdvanceMavlinkPacketAnimation(
      *ctx.renderer, now, IsMavlinkMode(mode), dfu_link_step_period_ms);
  const bool mavlink_packet_active =
      mavlink_tx_lane_.active_count > 0 || mavlink_rx_lane_.active_count > 0;
  const bool mode_changed = mode != last_mode_;
  const bool dfu_link_changed = AdvanceDfuLinkAnimation(
      *ctx.renderer, now, dfu_binary_link_active, dfu_link_step_period_ms);

  if (mode_changed) {
    if (IsMavlinkMode(mode) != IsMavlinkMode(last_mode_)) {
      ResetMavlinkPacketAnimation();
    }
    const bool status_changed = std::strcmp(StatusTextForMode(last_mode_),
                                            StatusTextForMode(mode)) != 0;
    last_mode_ = mode;
    if (status_changed) {
      BeginTextPhase(ctx, now, mode);
    } else {
      RenderMode(ctx, now, mode);
    }
    return;
  }

  const bool serving_animation_active = IsServingMode(mode);
  if (!has_rendered_ || text_animation_active_ ||
      last_dot_count_ != DotCount(now) || serving_animation_active ||
      dfu_link_changed || verify_magnifier_changed || mavlink_packet_changed ||
      mavlink_packet_active || dfu_credentials_animation_active ||
      progress_status_scroll_active) {
    RenderMode(ctx, now, mode);
  }
}

void MainUiWidget::EnsureLinkGlyphMetrics(DisplayRenderer &renderer) {
  if (link_glyph_metrics_initialized_) {
    return;
  }

  const DisplayTextBounds bounds = renderer.MeasureText("0", kDfuSleepStyle);
  dfu_link_glyph_x_ = bounds.x;
  dfu_link_glyph_y_ = bounds.y;
  dfu_link_glyph_width_px_ = std::max<int16_t>(6, bounds.width);
  dfu_link_glyph_height_px_ = std::max<int16_t>(8, bounds.height);
  dfu_link_glyph_pitch_px_ =
      static_cast<int16_t>(dfu_link_glyph_width_px_ + kDfuLinkGlyphGapPx);
  link_glyph_metrics_initialized_ = true;
}

void MainUiWidget::BeginTextPhase(WidgetContext &ctx, TimeMs now, Mode mode) {
  text_animation_start_ms_ = now;
  last_gear_step_ms_ = now;
  ResetVerifyMagnifierAnimation(now);
  text_animation_active_ = ShouldAnimateEntryText(mode);
  RenderMode(ctx, now, mode);
}

void MainUiWidget::AdvanceGearAnimation(WidgetContext &ctx, TimeMs now,
                                        bool active) {
  const TimeMs previous_step_ms = last_gear_step_ms_;
  last_gear_step_ms_ = now;
  if (!active || previous_step_ms == 0 || now <= previous_step_ms) {
    return;
  }

  TimeMs step_ms = now - previous_step_ms;
  if (ctx.ui != nullptr) {
    step_ms = std::min(step_ms, ctx.ui->GetFrameIntervalMs());
  }
  gear_animation_ms_ += step_ms;
}

void MainUiWidget::InitializeDfuLinkAnimation(DisplayRenderer &renderer,
                                              TimeMs now,
                                              TimeMs step_period_ms) {
  (void)step_period_ms;
  EnsureLinkGlyphMetrics(renderer);

  const int16_t gap_width = static_cast<int16_t>(
      renderer.Width() - chip_bitmap::kVisibleWidth - pc_bitmap::kVisibleWidth);
  const size_t required_glyphs =
      static_cast<size_t>(std::max<int16_t>(1, gap_width) /
                          std::max<int16_t>(1, dfu_link_glyph_pitch_px_)) +
      3u;
  dfu_link_glyph_count_ = std::clamp(required_glyphs, static_cast<size_t>(4u),
                                     kDfuLinkGlyphCapacity);
  for (size_t index = 0; index < dfu_link_glyph_count_; ++index) {
    dfu_link_glyphs_[index] = RandomBinaryGlyph();
  }
  dfu_link_offset_px_ = 0;
  dfu_link_subpixel_offset_ = 0;
  dfu_link_last_step_ms_ = now;
  dfu_link_initialized_ = true;
}

bool MainUiWidget::AdvanceDfuLinkAnimation(DisplayRenderer &renderer,
                                           TimeMs now, bool active,
                                           TimeMs step_period_ms) {
  if (!active) {
    return false;
  }
  const TimeMs effective_step_period_ms =
      (step_period_ms > 0) ? step_period_ms : kDefaultDfuLinkStepPeriodMs;
  if (!dfu_link_initialized_) {
    InitializeDfuLinkAnimation(renderer, now, effective_step_period_ms);
    return true;
  }

  const TimeMs previous_step_ms = dfu_link_last_step_ms_;
  dfu_link_last_step_ms_ = now;
  if (previous_step_ms == 0 || now <= previous_step_ms) {
    return false;
  }

  const TimeMs step_ms =
      std::min(now - previous_step_ms, effective_step_period_ms);
  const uint32_t advanced_subpixels =
      (static_cast<uint32_t>(step_ms) * kDfuLinkPixelsPerSecond *
       kDfuLinkSubpixelScale) /
      1000u;
  dfu_link_subpixel_offset_ =
      static_cast<uint16_t>(dfu_link_subpixel_offset_ + advanced_subpixels);

  uint8_t pixel_steps =
      static_cast<uint8_t>(dfu_link_subpixel_offset_ / kDfuLinkSubpixelScale);
  dfu_link_subpixel_offset_ =
      static_cast<uint16_t>(dfu_link_subpixel_offset_ % kDfuLinkSubpixelScale);
  if (pixel_steps == 0) {
    return false;
  }

  while (pixel_steps-- > 0u) {
    ++dfu_link_offset_px_;
    if (dfu_link_offset_px_ >= dfu_link_glyph_pitch_px_) {
      dfu_link_offset_px_ = 0;
      for (size_t index = dfu_link_glyph_count_ - 1u; index > 0u; --index) {
        dfu_link_glyphs_[index] = dfu_link_glyphs_[index - 1u];
      }
      dfu_link_glyphs_[0] = RandomBinaryGlyph();
    }
  }

  return true;
}

void MainUiWidget::ResetMavlinkPacketAnimation() {
  mavlink_tx_lane_ = {};
  mavlink_rx_lane_ = {};
  mavlink_tx_lane_.last_seen_packet_count = Sys().Mavlink().UdpTxPacketCount();
  mavlink_rx_lane_.last_seen_packet_count = Sys().Mavlink().UdpRxPacketCount();
  mavlink_packet_last_step_ms_ = Sys().Timebase().NowMs();
}

bool MainUiWidget::AdvanceMavlinkPacketAnimation(DisplayRenderer &renderer,
                                                 TimeMs now, bool active,
                                                 TimeMs step_period_ms) {
  if (!active) {
    return false;
  }

  EnsureLinkGlyphMetrics(renderer);

  const int16_t gap_width =
      static_cast<int16_t>(renderer.Width() - chip_bitmap::kVisibleWidth -
                           mavlink0_bitmap::kVisibleWidth);
  const int16_t travel_px =
      std::max<int16_t>(0, gap_width - dfu_link_glyph_width_px_);
  if (travel_px <= 0) {
    return false;
  }

  const uint32_t travel_subpixels =
      static_cast<uint32_t>(travel_px) * kMavlinkPacketSubpixelScale;
  const uint32_t min_spawn_progress_subpx =
      static_cast<uint32_t>(std::max<int16_t>(1, dfu_link_glyph_pitch_px_)) *
      kMavlinkPacketSubpixelScale;
  const TimeMs effective_step_period_ms =
      (step_period_ms > 0) ? step_period_ms : kDefaultDfuLinkStepPeriodMs;
  const TimeMs previous_step_ms = mavlink_packet_last_step_ms_;
  mavlink_packet_last_step_ms_ = now;

  uint32_t advanced_subpixels = 0;
  if (previous_step_ms != 0 && now > previous_step_ms) {
    const TimeMs step_ms =
        std::min(now - previous_step_ms, effective_step_period_ms);
    advanced_subpixels =
        (static_cast<uint32_t>(step_ms) * kMavlinkPacketPixelsPerSecond *
         kMavlinkPacketSubpixelScale) /
        1000u;
  }

  bool changed = false;
  const auto advance_lane = [&](MavlinkPacketLane &lane) {
    size_t write_index = 0;
    for (size_t read_index = 0; read_index < lane.active_count; ++read_index) {
      MavlinkPacketGlyph glyph = lane.packets[read_index];
      glyph.progress_subpx = static_cast<uint16_t>(std::min<uint32_t>(
          travel_subpixels, glyph.progress_subpx + advanced_subpixels));
      if (glyph.progress_subpx >= travel_subpixels) {
        changed = true;
        continue;
      }
      lane.packets[write_index++] = glyph;
    }
    if (write_index != lane.active_count) {
      changed = true;
    }
    lane.active_count = write_index;
  };

  if (advanced_subpixels > 0) {
    advance_lane(mavlink_tx_lane_);
    advance_lane(mavlink_rx_lane_);
    if (mavlink_tx_lane_.active_count > 0 ||
        mavlink_rx_lane_.active_count > 0) {
      changed = true;
    }
  }

  const auto maybe_spawn = [&](MavlinkPacketLane &lane, uint32_t packet_count) {
    if (packet_count == lane.last_seen_packet_count) {
      return;
    }
    const bool can_accept =
        lane.active_count == 0 ||
        lane.packets[lane.active_count - 1].progress_subpx >=
            min_spawn_progress_subpx;
    if (can_accept && lane.active_count < lane.packets.size()) {
      lane.packets[lane.active_count++] = {
          .glyph = RandomBinaryGlyph(),
          .progress_subpx = 0,
      };
      changed = true;
    }
    lane.last_seen_packet_count = packet_count;
  };

  maybe_spawn(mavlink_tx_lane_, Sys().Mavlink().UdpTxPacketCount());
  maybe_spawn(mavlink_rx_lane_, Sys().Mavlink().UdpRxPacketCount());

  return changed;
}

void MainUiWidget::RandomizeVerifyDigits() {
  for (char &digit : verify_digits_) {
    digit = RandomBinaryGlyph();
  }
}

void MainUiWidget::ResetVerifyMagnifierAnimation(TimeMs now) {
  verify_magnifier_last_step_ms_ = now;
  verify_magnifier_subpixel_offset_ = 0;
  verify_magnifier_offset_px_ = 0;
  verify_magnifier_moving_right_ = true;
  verify_magnifier_initialized_ = false;
  RandomizeVerifyDigits();
}

bool MainUiWidget::AdvanceVerifyMagnifierAnimation(TimeMs now, bool active,
                                                   TimeMs step_period_ms) {
  if (!active) {
    verify_magnifier_initialized_ = false;
    return false;
  }

  const int16_t travel_px = VerifyMagnifierTravelPx();
  if (travel_px <= 0) {
    verify_magnifier_initialized_ = true;
    verify_magnifier_offset_px_ = 0;
    return false;
  }

  const TimeMs effective_step_period_ms =
      (step_period_ms > 0) ? step_period_ms : kDefaultDfuLinkStepPeriodMs;
  if (!verify_magnifier_initialized_) {
    verify_magnifier_last_step_ms_ = now;
    verify_magnifier_subpixel_offset_ = 0;
    verify_magnifier_offset_px_ = 0;
    verify_magnifier_moving_right_ = true;
    verify_magnifier_initialized_ = true;
    return true;
  }

  const TimeMs previous_step_ms = verify_magnifier_last_step_ms_;
  verify_magnifier_last_step_ms_ = now;
  if (previous_step_ms == 0 || now <= previous_step_ms) {
    return false;
  }

  const TimeMs step_ms =
      std::min(now - previous_step_ms, effective_step_period_ms);
  const uint32_t advanced_subpixels =
      (static_cast<uint32_t>(step_ms) * kVerifyMagnifierPixelsPerSecond *
       kVerifyMagnifierSubpixelScale) /
      1000u;
  verify_magnifier_subpixel_offset_ = static_cast<uint16_t>(
      verify_magnifier_subpixel_offset_ + advanced_subpixels);

  uint16_t pixel_steps = static_cast<uint16_t>(
      verify_magnifier_subpixel_offset_ / kVerifyMagnifierSubpixelScale);
  verify_magnifier_subpixel_offset_ = static_cast<uint16_t>(
      verify_magnifier_subpixel_offset_ % kVerifyMagnifierSubpixelScale);
  if (pixel_steps == 0u) {
    return false;
  }

  while (pixel_steps-- > 0u) {
    if (verify_magnifier_moving_right_) {
      if (verify_magnifier_offset_px_ >= travel_px) {
        verify_magnifier_offset_px_ = travel_px;
        verify_magnifier_moving_right_ = false;
        RandomizeVerifyDigits();
        if (travel_px > 0) {
          --verify_magnifier_offset_px_;
        }
      } else {
        ++verify_magnifier_offset_px_;
      }
    } else {
      if (verify_magnifier_offset_px_ <= 0) {
        verify_magnifier_offset_px_ = 0;
        verify_magnifier_moving_right_ = true;
        RandomizeVerifyDigits();
        if (travel_px > 0) {
          ++verify_magnifier_offset_px_;
        }
      } else {
        --verify_magnifier_offset_px_;
      }
    }
  }

  return true;
}

void MainUiWidget::RenderMode(WidgetContext &ctx, TimeMs now, Mode mode) {
  if (ctx.ui == nullptr || ctx.renderer == nullptr) {
    return;
  }

  DisplayRenderer &renderer = *ctx.renderer;
  const char *status = StatusTextForMode(mode);
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
  const auto finish_render = [&]() {
    if (text_animation_active_ && visible_chars >= std::strlen(status)) {
      text_animation_active_ = false;
    }
    last_dot_count_ = dot_count;
    has_rendered_ = true;
  };

  renderer.Clear();
  if (IsScrollableProgressMode(mode)) {
    const DisplayTextBounds prefix_bounds =
        renderer.MeasureText(">", kStatusStyle);
    const DisplayTextBounds body_bounds =
        renderer.MeasureText(animated_status, kStatusStyle);
    const DisplayTextBounds dot_slot_bounds =
        renderer.MeasureText("...", kStatusStyle);
    const DisplayTextBounds viewport_bounds =
        renderer.MeasureText(kProgrammingViewportSample, kStatusStyle);
    const int16_t line_height = static_cast<int16_t>(status_bounds.height);
    const int16_t prefix_cursor_x =
        static_cast<int16_t>(kTextInsetX - prefix_bounds.x);
    const int16_t viewport_left = static_cast<int16_t>(
        kTextInsetX + static_cast<int16_t>(prefix_bounds.width));
    const int16_t viewport_width = static_cast<int16_t>(viewport_bounds.width);
    const int16_t viewport_right =
        static_cast<int16_t>(viewport_left + viewport_width);
    const int16_t content_width =
        static_cast<int16_t>(body_bounds.width + dot_slot_bounds.width);
    const int16_t body_cursor_x = static_cast<int16_t>(
        viewport_left - body_bounds.x -
        renderer.ScrollOffsetPx(content_width, viewport_width, now));

    renderer.DrawText(animated_status, body_cursor_x, status_y, kStatusStyle);

    char dot_text[4]{};
    for (uint8_t index = 0; index < dot_count && index < 3; ++index) {
      dot_text[index] = '.';
    }
    if (dot_text[0] != '\0') {
      const DisplayTextBounds active_dot_bounds =
          renderer.MeasureText(dot_text, kStatusStyle);
      const int16_t dot_cursor_x = static_cast<int16_t>(
          body_cursor_x + body_bounds.x +
          static_cast<int16_t>(body_bounds.width) - active_dot_bounds.x);
      renderer.DrawText(dot_text, dot_cursor_x, status_y, kStatusStyle);
    }

    renderer.FillRect(0, status_top, viewport_left, line_height, false);
    renderer.FillRect(viewport_right, status_top,
                      static_cast<int16_t>(renderer.Width()) - viewport_right,
                      line_height, false);
    renderer.DrawText(">", prefix_cursor_x, status_y, kStatusStyle);
  } else {
    renderer.DrawText(status_line, kTextInsetX, status_y, kStatusStyle);
  }
  if (mode == Mode::kServing) {
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
      DrawRotatingBitmap(renderer, gear1_bitmap::kBitmapData.data(),
                         gear1_bitmap::kVisibleWidth,
                         gear1_bitmap::kVisibleHeight, gear_animation_ms_,
                         static_cast<size_t>(right_gear_x), right_gear_y, true);
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
  } else if (IsDfuVisualMode(mode) &&
             wifi_bitmap::kVisibleWidth <= renderer.Width() &&
             wifi_bitmap::kVisibleHeight <= renderer.Height()) {
    const bool show_warning_icon =
        !IsWifiCredentialsMode(mode) || (((now / kDotStepPeriodMs) % 2u) == 0u);
    if (show_warning_icon) {
      const int16_t dfu_wifi_icon_x = static_cast<int16_t>(
          renderer.Width() - wifi_bitmap::kVisibleWidth - kDfuWifiRightInsetX);
      const int16_t dfu_wifi_icon_y = static_cast<int16_t>(kDfuWifiTopY);
      renderer.DrawBitmap(
          wifi_bitmap::kBitmapData.data(), wifi_bitmap::kVisibleWidth,
          wifi_bitmap::kVisibleHeight, static_cast<size_t>(dfu_wifi_icon_x),
          static_cast<size_t>(dfu_wifi_icon_y));

      if (IsWifiCredentialsMode(mode)) {
        const DisplayTextBounds exclamation_bounds =
            renderer.MeasureText("!", kDfuSleepStyle);
        if (exclamation_bounds.width > 0 && exclamation_bounds.height > 0) {
          const int16_t exclamation_top = static_cast<int16_t>(
              dfu_wifi_icon_y +
              std::max<int16_t>(
                  0, (static_cast<int16_t>(wifi_bitmap::kVisibleHeight) -
                      static_cast<int16_t>(exclamation_bounds.height)) /
                         2));
          const int16_t exclamation_x = static_cast<int16_t>(
              dfu_wifi_icon_x - kDfuWifiWarningGapPx -
              static_cast<int16_t>(exclamation_bounds.width) -
              exclamation_bounds.x);
          const int16_t exclamation_y =
              static_cast<int16_t>(exclamation_top - exclamation_bounds.y);
          renderer.DrawText("!", exclamation_x, exclamation_y, kDfuSleepStyle);
        }
      }
    }
    if (IsWifiCredentialsMode(mode)) {
      char ssid_line[96]{};
      char password_line[96]{};
      std::snprintf(ssid_line, sizeof(ssid_line), "AP: %s",
                    Sys().Wifi().ApSsid());
      std::snprintf(password_line, sizeof(password_line), "Password: %s",
                    Sys().Wifi().ApPassword());

      const DisplayTextBounds ssid_bounds =
          renderer.MeasureText(ssid_line, kDfuSleepStyle);
      const DisplayTextBounds password_bounds =
          renderer.MeasureText(password_line, kDfuSleepStyle);
      const int16_t line_height = static_cast<int16_t>(
          std::max(ssid_bounds.height, password_bounds.height));
      if (line_height > 0) {
        const int16_t lines_total_height =
            static_cast<int16_t>((2 * line_height) + kDfuCredentialLineGapPx);
        const int16_t credentials_top = static_cast<int16_t>(
            status_bounds.height +
            std::max<int16_t>(0, (static_cast<int16_t>(renderer.Height()) -
                                  static_cast<int16_t>(status_bounds.height) -
                                  lines_total_height) /
                                     2));
        const int16_t line1_top = credentials_top;
        const int16_t line2_top = static_cast<int16_t>(line1_top + line_height +
                                                       kDfuCredentialLineGapPx);
        const int16_t line_left = kTextInsetX;
        const int16_t line_width = static_cast<int16_t>(
            renderer.Width() - static_cast<size_t>(kTextInsetX));

        renderer.DrawScrollingText(ssid_line, line_left, line1_top, line_width,
                                   now, kDfuSleepStyle);
        renderer.DrawScrollingText(password_line, line_left, line2_top,
                                   line_width, now, kDfuSleepStyle);
      }
      finish_render();
      return;
    }
    if (mode == Mode::kVerifying &&
        chip_verify_bitmap::kVisibleWidth <= renderer.Width() &&
        chip_verify_bitmap::kVisibleHeight <= renderer.Height()) {
      const int16_t body_top = static_cast<int16_t>(status_bounds.height);
      const int16_t body_height = std::max<int16_t>(
          0, static_cast<int16_t>(renderer.Height()) - body_top);
      const int16_t verify_icon_x = std::max<int16_t>(
          0, (static_cast<int16_t>(renderer.Width()) -
              static_cast<int16_t>(chip_verify_bitmap::kVisibleWidth)) /
                 2);
      const int16_t verify_icon_y = static_cast<int16_t>(
          body_top +
          std::max<int16_t>(
              0, (body_height -
                  static_cast<int16_t>(chip_verify_bitmap::kVisibleHeight)) /
                     2));
      renderer.DrawBitmap(chip_verify_bitmap::kBitmapData.data(),
                          chip_verify_bitmap::kVisibleWidth,
                          chip_verify_bitmap::kVisibleHeight,
                          static_cast<size_t>(verify_icon_x),
                          static_cast<size_t>(verify_icon_y));

      const int16_t verify_focus_center_x = static_cast<int16_t>(
          verify_icon_x +
          static_cast<int16_t>(chip_verify_bitmap::kVisibleWidth / 2));
      const int16_t verify_focus_center_y = static_cast<int16_t>(
          verify_icon_y +
          static_cast<int16_t>(chip_verify_bitmap::kVisibleHeight / 2));
      const int16_t magnifier_left_overlap_px = VerifyMagnifierLeftOverlapPx();
      const int16_t magnifier_min_x = static_cast<int16_t>(
          verify_icon_x + magnifier_left_overlap_px -
          static_cast<int16_t>(magnifying_glass_bitmap::kVisibleWidth));
      const int16_t magnifier_x =
          static_cast<int16_t>(magnifier_min_x + verify_magnifier_offset_px_);
      const int16_t magnifier_y = static_cast<int16_t>(
          verify_focus_center_y - kVerifyMagnifierLensCenterY - 1);
      const int16_t lens_center_x =
          static_cast<int16_t>(magnifier_x + kVerifyMagnifierLensCenterX);
      const int16_t lens_center_y =
          static_cast<int16_t>(magnifier_y + kVerifyMagnifierLensCenterY);

      DrawMaskedVerifyDigits(renderer, verify_focus_center_x,
                             verify_focus_center_y, verify_digits_,
                             lens_center_x, lens_center_y, true,
                             kVerifyDigitGlyphWidthPx,
                             kVerifyDigitGlyphHeightPx, kVerifyDigitGlyphGapPx,
                             kVerifyDigitZero.data(), kVerifyDigitOne.data());

      if (magnifier_x >= 0 && magnifier_y >= 0 &&
          (magnifier_x +
           static_cast<int16_t>(magnifying_glass_bitmap::kVisibleWidth)) <=
              static_cast<int16_t>(renderer.Width()) &&
          (magnifier_y +
           static_cast<int16_t>(magnifying_glass_bitmap::kVisibleHeight)) <=
              static_cast<int16_t>(renderer.Height())) {
        renderer.DrawBitmap(magnifying_glass_bitmap::kBitmapData.data(),
                            magnifying_glass_bitmap::kVisibleWidth,
                            magnifying_glass_bitmap::kVisibleHeight,
                            static_cast<size_t>(magnifier_x),
                            static_cast<size_t>(magnifier_y));
      }
      finish_render();
      return;
    }

    const IconAsset left_icon = LeftIconForMode(mode, now);
    if (left_icon.data != nullptr && left_icon.width <= renderer.Width() &&
        chip_bitmap::kVisibleWidth <= renderer.Width() &&
        left_icon.height <= renderer.Height() &&
        chip_bitmap::kVisibleHeight <= renderer.Height()) {
      const size_t right_icon_x =
          renderer.Width() - chip_bitmap::kVisibleWidth - kDfuIconRightInsetX;
      const int16_t icon_region_height = static_cast<int16_t>(
          std::max(left_icon.height, chip_bitmap::kVisibleHeight));
      const int16_t icon_region_y =
          static_cast<int16_t>(renderer.Height() - icon_region_height);
      const int16_t left_icon_y =
          static_cast<int16_t>(renderer.Height() - left_icon.height);
      const int16_t chip_icon_y =
          static_cast<int16_t>(renderer.Height() - chip_bitmap::kVisibleHeight);
      const int16_t gap_left =
          static_cast<int16_t>(kDfuIconLeftX + left_icon.width);
      const int16_t gap_right = static_cast<int16_t>(right_icon_x);

      if (mode == Mode::kProgramming && gap_left < gap_right) {
        if (!dfu_link_initialized_) {
          const TimeMs dfu_link_step_period_ms =
              (ctx.ui != nullptr && ctx.ui->GetFrameIntervalMs() > 0)
                  ? ctx.ui->GetFrameIntervalMs()
                  : kDefaultDfuLinkStepPeriodMs;
          InitializeDfuLinkAnimation(renderer, now, dfu_link_step_period_ms);
        }
        const int16_t link_top = static_cast<int16_t>(
            icon_region_y +
            std::max<int16_t>(
                0, (icon_region_height - dfu_link_glyph_height_px_) / 2));
        const int16_t link_cursor_y =
            static_cast<int16_t>(link_top - dfu_link_glyph_y_);
        for (size_t index = 0; index < dfu_link_glyph_count_; ++index) {
          const int16_t glyph_left = static_cast<int16_t>(
              gap_left - dfu_link_glyph_pitch_px_ +
              (static_cast<int16_t>(index) * dfu_link_glyph_pitch_px_) +
              dfu_link_offset_px_);
          const int16_t glyph_right =
              static_cast<int16_t>(glyph_left + dfu_link_glyph_width_px_);
          if (glyph_right <= gap_left || glyph_left >= gap_right) {
            continue;
          }

          const char glyph[2] = {dfu_link_glyphs_[index], '\0'};
          renderer.DrawText(
              glyph, static_cast<int16_t>(glyph_left - dfu_link_glyph_x_),
              link_cursor_y, kDfuSleepStyle);
        }
      } else if (IsMavlinkMode(mode) && gap_left < gap_right) {
        EnsureLinkGlyphMetrics(renderer);
        const int16_t travel_px = std::max<int16_t>(
            0, gap_right - gap_left - dfu_link_glyph_width_px_);
        if (travel_px > 0) {
          const int16_t lane_top_y =
              std::max<int16_t>(0, std::min<int16_t>(left_icon_y, chip_icon_y));
          const int16_t lane_bottom_y = std::max<int16_t>(
              0, std::max<int16_t>(
                     left_icon_y + static_cast<int16_t>(left_icon.height),
                     chip_icon_y +
                         static_cast<int16_t>(chip_bitmap::kVisibleHeight)) -
                     dfu_link_glyph_height_px_);
          const auto draw_lane = [&](const MavlinkPacketLane &lane,
                                     int16_t start_x, int16_t direction_sign,
                                     int16_t lane_y) {
            const int16_t cursor_y =
                static_cast<int16_t>(lane_y - dfu_link_glyph_y_);
            for (size_t index = 0; index < lane.active_count; ++index) {
              const MavlinkPacketGlyph &glyph = lane.packets[index];
              const int16_t progress_px = static_cast<int16_t>(
                  glyph.progress_subpx / kMavlinkPacketSubpixelScale);
              const int16_t glyph_left = static_cast<int16_t>(
                  start_x + (direction_sign * progress_px));
              const char text[2] = {glyph.glyph, '\0'};
              renderer.DrawText(
                  text, static_cast<int16_t>(glyph_left - dfu_link_glyph_x_),
                  cursor_y, kDfuSleepStyle);
            }
          };

          draw_lane(mavlink_tx_lane_,
                    static_cast<int16_t>(gap_right - dfu_link_glyph_width_px_),
                    -1, lane_top_y);
          draw_lane(mavlink_rx_lane_, gap_left, 1, lane_bottom_y);
        }
      }

      renderer.DrawBitmap(left_icon.data, left_icon.width, left_icon.height,
                          kDfuIconLeftX, static_cast<size_t>(left_icon_y));
      renderer.DrawBitmap(chip_bitmap::kBitmapData.data(),
                          chip_bitmap::kVisibleWidth,
                          chip_bitmap::kVisibleHeight, right_icon_x,
                          static_cast<size_t>(chip_icon_y));

      if (mode == Mode::kDfuIdleConnected && dot_count > 0 &&
          (kDfuIconLeftX + left_icon.width) <= right_icon_x) {
        constexpr char kSleepChar[] = "z";
        const DisplayTextBounds z_bounds =
            renderer.MeasureText(kSleepChar, kDfuSleepStyle);
        const DisplayTextBounds zz_bounds =
            renderer.MeasureText("zz", kDfuSleepStyle);
        const DisplayTextBounds zzz_bounds =
            renderer.MeasureText("zzz", kDfuSleepStyle);
        if (z_bounds.width > 0 && z_bounds.height > 0 && zz_bounds.width > 0 &&
            zzz_bounds.width > 0) {
          const int16_t gap_left =
              static_cast<int16_t>(kDfuIconLeftX + left_icon.width);
          const int16_t gap_width =
              static_cast<int16_t>(right_icon_x - gap_left);
          const int16_t zzz_left = static_cast<int16_t>(
              gap_left +
              std::max<int16_t>(
                  0, (gap_width - static_cast<int16_t>(zzz_bounds.width)) / 2));
          const int16_t sleep_top = static_cast<int16_t>(
              icon_region_y +
              std::max<int16_t>(0, (icon_region_height -
                                    static_cast<int16_t>(z_bounds.height)) /
                                       2));
          const int16_t sleep_y = static_cast<int16_t>(sleep_top - z_bounds.y);

          const int16_t z_positions[] = {
              static_cast<int16_t>(zzz_left - z_bounds.x),
              static_cast<int16_t>(zzz_left + z_bounds.width - z_bounds.x),
              static_cast<int16_t>(zzz_left + zz_bounds.width - z_bounds.x),
          };
          const uint8_t visible_z_count = std::min<uint8_t>(dot_count, 3);
          for (uint8_t index = 0; index < visible_z_count; ++index) {
            renderer.DrawText(kSleepChar, z_positions[index], sleep_y,
                              kDfuSleepStyle);
          }
        }
      }
    }
  }

  finish_render();
}
