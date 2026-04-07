#pragma once

#include "ui.hpp"

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

  class TransitioningState : public IState<WidgetContext> {
   public:
    explicit TransitioningState(BootWidget &widget) : widget_(widget) {}

    const char *Name() const override { return "transitioning"; }
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

  IWidget *next_widget_ = nullptr;
  TimeMs timeout_ms_ = 0;
  TimeMs deadline_ms_ = 0;
  ShowingState showing_state_{*this};
  TransitioningState transitioning_state_{*this};
  DoneState done_state_{*this};
};
