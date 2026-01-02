#pragma once
#include "ctx.hpp"
#include "state_machine.hpp"

class ListenState; // forward

class IdleState : public IState<AppContext> {
public:
  const char *name() const override { return "Idle"; }

  void SetListen(IState<AppContext> *s) { listen_ = s; }

  void on_enter(AppContext &ctx, sm_tick_t now) override;
  void on_step(AppContext &ctx, sm_tick_t now) override;
  void on_exit(AppContext &ctx, sm_tick_t now) override;

private:
  IState<AppContext> *listen_ = nullptr;
};

class ListenState : public IState<AppContext> {
public:
  const char *name() const override { return "Listen"; }

  void SetIdle(IState<AppContext> *s) { idle_ = s; }
  void SetBlinkPeriod(sm_tick_t ms) { period_ms_ = ms; }

  void on_enter(AppContext &ctx, sm_tick_t now) override;
  void on_step(AppContext &ctx, sm_tick_t now) override;
  void on_exit(AppContext &ctx, sm_tick_t now) override;

private:
  IState<AppContext> *idle_ = nullptr;

  sm_tick_t period_ms_ = 300;
  sm_tick_t next_toggle_ = 0;
};
