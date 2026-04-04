#pragma once

#include "display_manager.hpp"

class BootWidget : public StateMachineWidget {
 public:
  const char *Name() const override { return "boot"; }

  void SetNextWidget(IWidget *widget) { next_widget_ = widget; }
  void SetTimeoutMs(TimeMs timeout_ms) { timeout_ms_ = timeout_ms; }

 private:
  class ShowingState : public IState<WidgetContext> {
   public:
    explicit ShowingState(BootWidget &widget) : widget_(widget) {}

    const char *Name() const override { return "showing"; }
    void OnEnter(WidgetContext &ctx) override;
    void OnStep(WidgetContext &ctx, SmTick now) override;

   private:
    BootWidget &widget_;
  };

  class FadingState : public IState<WidgetContext> {
   public:
    explicit FadingState(BootWidget &widget) : widget_(widget) {}

    const char *Name() const override { return "fading"; }
    void OnEnter(WidgetContext &ctx) override;
    void OnStep(WidgetContext &ctx, SmTick now) override;

   private:
    BootWidget &widget_;
  };

  class DoneState : public IState<WidgetContext> {
   public:
    explicit DoneState(BootWidget &widget) : widget_(widget) {}

    const char *Name() const override { return "done"; }
    void OnEnter(WidgetContext &ctx) override;
    void OnStep(WidgetContext &ctx, SmTick now) override;

   private:
    BootWidget &widget_;
  };

  IState<WidgetContext> &InitialState() override;

  static constexpr TimeMs kFadeDurationMs = 1000;
  static constexpr uint8_t kFadeOutInterval = 0;
  IWidget *next_widget_ = nullptr;
  TimeMs timeout_ms_ = 0;
  TimeMs deadline_ms_ = 0;
  TimeMs fade_start_ms_ = 0;
  ShowingState showing_state_{*this};
  FadingState fading_state_{*this};
  DoneState done_state_{*this};
};
