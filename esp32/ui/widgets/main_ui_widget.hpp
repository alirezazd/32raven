#pragma once

#include <array>

#include "display_manager.hpp"

class MainUiWidget : public IWidget {
 public:
  static MainUiWidget &GetInstance() {
    static MainUiWidget instance;
    return instance;
  }

  const char *Name() const override { return "main_ui"; }

  void SetStatus(const char *status);
  void OnEnter(WidgetContext &ctx) override;
  void OnStep(WidgetContext &ctx, TimeMs now) override;

 private:
  static constexpr size_t kStatusBufferSize = 24;

  std::array<char, kStatusBufferSize> CurrentStatus() const;
  void BeginTextPhase(WidgetContext &ctx, TimeMs now, const char *status);
  void CopyStatus(const char *status);
  void RenderTextStatus(WidgetContext &ctx, TimeMs now, const char *status);

  static constexpr size_t kStatusLineBufferSize =
      kStatusBufferSize + 5;

  void AdvanceGearAnimation(WidgetContext &ctx, TimeMs now, bool active);
  mutable TimeMs text_animation_start_ms_ = 0;
  mutable TimeMs last_gear_step_ms_ = 0;
  mutable TimeMs gear_animation_ms_ = 0;
  mutable uint8_t last_dot_count_ = 0;
  mutable std::array<char, kStatusBufferSize> last_status_{};
  std::array<char, kStatusBufferSize> status_{};
  mutable bool has_rendered_ = false;
  mutable bool text_animation_active_ = false;

  MainUiWidget() = default;
};
