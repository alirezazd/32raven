#pragma once

#include <array>

#include "ui.hpp"

class MainUiWidget : public IWidget {
 public:
  using Mode = Ui::MainScreen;

  static MainUiWidget &GetInstance() {
    static MainUiWidget instance;
    return instance;
  }

  const char *Name() const override { return "main_ui"; }

  void SetMode(Mode mode);
  void OnEnter(WidgetContext &ctx) override;
  void OnStep(WidgetContext &ctx, TimeMs now) override;

 private:
  static constexpr size_t kStatusBufferSize = 24;
  static constexpr size_t kMavlinkLaneCapacity = 4;

  struct MavlinkPacketGlyph {
    char glyph = '0';
    uint16_t progress_subpx = 0;
  };

  struct MavlinkPacketLane {
    std::array<MavlinkPacketGlyph, kMavlinkLaneCapacity> packets{};
    size_t active_count = 0;
    uint32_t last_seen_packet_count = 0;
  };

  Mode CurrentMode() const;
  void BeginTextPhase(WidgetContext &ctx, TimeMs now, Mode mode);
  void RenderMode(WidgetContext &ctx, TimeMs now, Mode mode);
  void EnsureLinkGlyphMetrics(DisplayRenderer &renderer);
  void InitializeDfuLinkAnimation(DisplayRenderer &renderer, TimeMs now,
                                  TimeMs step_period_ms);
  bool AdvanceDfuLinkAnimation(DisplayRenderer &renderer, TimeMs now,
                               bool active, TimeMs step_period_ms);
  void ResetMavlinkPacketAnimation();
  bool AdvanceMavlinkPacketAnimation(DisplayRenderer &renderer, TimeMs now,
                                     bool active, TimeMs step_period_ms);
  void RandomizeVerifyDigits();
  void ResetVerifyMagnifierAnimation(TimeMs now);
  bool AdvanceVerifyMagnifierAnimation(TimeMs now, bool active,
                                       TimeMs step_period_ms);

  static constexpr size_t kStatusLineBufferSize = kStatusBufferSize + 5;
  static constexpr size_t kDfuLinkGlyphCapacity = 16;

  void AdvanceGearAnimation(WidgetContext &ctx, TimeMs now, bool active);
  mutable TimeMs text_animation_start_ms_ = 0;
  mutable TimeMs last_gear_step_ms_ = 0;
  mutable TimeMs gear_animation_ms_ = 0;
  mutable TimeMs dfu_link_last_step_ms_ = 0;
  mutable TimeMs mavlink_packet_last_step_ms_ = 0;
  mutable TimeMs verify_magnifier_last_step_ms_ = 0;
  mutable uint8_t last_dot_count_ = 0;
  mutable uint8_t dfu_link_offset_px_ = 0;
  mutable uint16_t dfu_link_subpixel_offset_ = 0;
  mutable bool link_glyph_metrics_initialized_ = false;
  mutable uint16_t verify_magnifier_subpixel_offset_ = 0;
  mutable std::array<char, kDfuLinkGlyphCapacity> dfu_link_glyphs_{};
  mutable std::array<char, 3> verify_digits_{};
  mutable MavlinkPacketLane mavlink_tx_lane_{};
  mutable MavlinkPacketLane mavlink_rx_lane_{};
  mutable size_t dfu_link_glyph_count_ = 0;
  mutable int16_t dfu_link_glyph_x_ = 0;
  mutable int16_t dfu_link_glyph_y_ = 0;
  mutable int16_t dfu_link_glyph_width_px_ = 0;
  mutable int16_t dfu_link_glyph_height_px_ = 0;
  mutable int16_t dfu_link_glyph_pitch_px_ = 0;
  mutable int16_t verify_magnifier_offset_px_ = 0;
  mutable bool has_rendered_ = false;
  mutable bool text_animation_active_ = false;
  mutable bool dfu_link_initialized_ = false;
  mutable bool verify_magnifier_moving_right_ = true;
  mutable bool verify_magnifier_initialized_ = false;
  mutable Mode last_mode_ = Mode::kBooting;
  Mode mode_ = Mode::kBooting;

  MainUiWidget() = default;
};
