#pragma once
#include "state_machine.hpp"
#include "system.hpp"

struct IdleState;
struct NotIdleState;

struct AppContext {
  System *sys;
  StateMachine<AppContext> *sm;

  IdleState *idle;
  NotIdleState *not_idle;
};

struct IdleState : public IState<AppContext> {
  const char *Name() const override { return "Idle"; }
  void OnEnter(AppContext &ctx, SmTick now) override;
  void OnStep(AppContext &ctx, SmTick now) override;
  void OnExit(AppContext &ctx, SmTick now) override;
};

struct NotIdleState : public IState<AppContext> {
  const char *Name() const override { return "NotIdle"; }
  void OnEnter(AppContext &ctx, SmTick now) override;
  void OnStep(AppContext &ctx, SmTick now) override;
  void OnExit(AppContext &ctx, SmTick now) override;
};
