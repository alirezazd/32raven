#pragma once
#include "state_machine.hpp"

struct AppContext;

class ListenState; // forward

class IdleState : public IState<AppContext> {
public:
  const char *name() const override { return "Idle"; }

  void on_enter(AppContext &ctx, sm_tick_t now) override;
  void on_step(AppContext &ctx, sm_tick_t now) override;
  void on_exit(AppContext &ctx, sm_tick_t now) override;
};

class ListenState : public IState<AppContext> {
public:
  const char *name() const override { return "Listen"; }

  void SetBlinkPeriod(sm_tick_t ms) { period_ms_ = ms; }

  void on_enter(AppContext &ctx, sm_tick_t now) override;
  void on_step(AppContext &ctx, sm_tick_t now) override;
  void on_exit(AppContext &ctx, sm_tick_t now) override;

private:
  sm_tick_t period_ms_ = 300;
  sm_tick_t next_toggle_ = 0;
};
