#pragma once

#include "ui.hpp"

class BootWidget : public IWidget {
 public:
  const char *Name() const override { return "boot"; }
  void OnEnter(WidgetContext &ctx) override;
  void OnStep(WidgetContext &ctx, TimeMs now) override;

  void SetNextWidget(IWidget *widget) { next_widget_ = widget; }
  void SetTimeoutMs(TimeMs timeout_ms) { timeout_ms_ = timeout_ms; }

 private:
  enum class Mode : uint8_t {
    kShowing,
    kTransitioning,
    kDone,
  };

  IWidget *next_widget_ = nullptr;
  TimeMs timeout_ms_ = 0;
  TimeMs deadline_ms_ = 0;
  Mode mode_ = Mode::kShowing;
};
