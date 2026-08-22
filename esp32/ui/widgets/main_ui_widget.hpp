// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include <array>

#include "ui.hpp"

class MainUiWidget : public IWidget {
 public:
  using Mode = Ui::MainScreen;

  static MainUiWidget &GetInstance();

  const char *Name() const override { return "main_ui"; }

  void SetMode(Mode mode);
  void OnEnter(WidgetContext &ctx) override;
  void OnStep(WidgetContext &ctx, TimeMs now) override;

 private:
  static constexpr size_t kStatusBufferSize = 24;
  static constexpr size_t kLinkLaneCapacity = 4;

  struct LinkPacketGlyph {
    char glyph = '0';
    uint16_t progress_subpx = 0;
  };

  struct LinkPacketLane {
    std::array<LinkPacketGlyph, kLinkLaneCapacity> packets{};
    size_t active_count = 0;
    uint32_t last_seen_packet_count = 0;
    uint32_t last_seen_heartbeat_count = 0;
  };

  Mode CurrentMode() const;
  void BeginTextPhase(WidgetContext &ctx, TimeMs now, Mode mode);
  void RenderMode(WidgetContext &ctx, TimeMs now, Mode mode);
  void EnsureLinkGlyphMetrics(DisplayRenderer &renderer);
  void InitializeServiceLinkAnimation(DisplayRenderer &renderer, TimeMs now,
                                  TimeMs step_period_ms);
  bool AdvanceServiceLinkAnimation(DisplayRenderer &renderer, TimeMs now,
                               bool active, TimeMs step_period_ms);
  struct LinkPacketSource {
    bool active = false;
    size_t left_icon_width = 0;
    uint32_t rx_count = 0;
    uint32_t tx_count = 0;
    uint32_t rx_heartbeat_count = 0;
    uint32_t tx_heartbeat_count = 0;
  };

  static LinkPacketSource PacketSourceForMode(Mode mode, TimeMs now);
  void ResetLinkPacketAnimation(const LinkPacketSource &source);
  bool AdvanceLinkPacketAnimation(DisplayRenderer &renderer, TimeMs now,
                                  const LinkPacketSource &source,
                                  TimeMs step_period_ms);
  void RandomizeVerifyDigits();
  void ResetVerifyMagnifierAnimation(TimeMs now);
  bool AdvanceVerifyMagnifierAnimation(TimeMs now, bool active,
                                       TimeMs step_period_ms);

  static constexpr size_t kStatusLineBufferSize = kStatusBufferSize + 5;
  static constexpr size_t kServiceLinkGlyphCapacity = 16;

  void AdvanceGearAnimation(WidgetContext &ctx, TimeMs now, bool active);
  mutable TimeMs text_animation_start_ms_ = 0;
  mutable TimeMs last_gear_step_ms_ = 0;
  mutable TimeMs gear_animation_ms_ = 0;
  mutable TimeMs service_link_last_step_ms_ = 0;
  mutable TimeMs link_packet_last_step_ms_ = 0;
  mutable TimeMs verify_magnifier_last_step_ms_ = 0;
  mutable uint8_t last_dot_count_ = 0;
  mutable uint8_t service_link_offset_px_ = 0;
  mutable uint16_t service_link_subpixel_offset_ = 0;
  mutable bool link_glyph_metrics_initialized_ = false;
  mutable uint16_t verify_magnifier_subpixel_offset_ = 0;
  mutable std::array<char, kServiceLinkGlyphCapacity> service_link_glyphs_{};
  mutable std::array<char, 3> verify_digits_{};
  mutable LinkPacketLane link_tx_lane_{};
  mutable LinkPacketLane link_rx_lane_{};
  mutable size_t service_link_glyph_count_ = 0;
  mutable int16_t service_link_glyph_x_ = 0;
  mutable int16_t service_link_glyph_y_ = 0;
  mutable int16_t service_link_glyph_width_px_ = 0;
  mutable int16_t service_link_glyph_height_px_ = 0;
  mutable int16_t service_link_glyph_pitch_px_ = 0;
  mutable int16_t verify_magnifier_offset_px_ = 0;
  mutable bool has_rendered_ = false;
  mutable bool text_animation_active_ = false;
  mutable bool service_link_initialized_ = false;
  mutable bool verify_magnifier_moving_right_ = true;
  mutable bool verify_magnifier_initialized_ = false;
  mutable Mode last_mode_ = Mode::kBooting;
  Mode mode_ = Mode::kBooting;

  MainUiWidget() = default;

  MainUiWidget(const MainUiWidget &) = delete;

  MainUiWidget &operator=(const MainUiWidget &) = delete;
};
