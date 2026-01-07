#pragma once
#include "state_machine.hpp"

struct AppContext;

class IdleState : public IState<AppContext> {
public:
  const char *Name() const override { return "Idle"; }

  void OnEnter(AppContext &ctx, SmTick now) override;
  void OnStep(AppContext &ctx, SmTick now) override;
  void OnExit(AppContext &ctx, SmTick now) override;
};

class ListenState : public IState<AppContext> {
public:
  const char *Name() const override { return "Listen"; }

  void SetIdleBlinkPeriod(SmTick ms) { period_ms_ = ms; }
  void SetListenBlinkPeriod(SmTick ms) { period_ms_ = ms; }

  void OnEnter(AppContext &ctx, SmTick now) override;
  void OnStep(AppContext &ctx, SmTick now) override;
  void OnExit(AppContext &ctx, SmTick now) override;

private:
  SmTick period_ms_ = 300;
  SmTick next_toggle_ = 0;
};

class ProgramState : public IState<AppContext> {
public:
  const char *Name() const override { return "Program"; }

  void OnEnter(AppContext &ctx, SmTick now) override;
  void OnStep(AppContext &ctx, SmTick now) override;
  void OnExit(AppContext &ctx, SmTick now) override;
};

class ErrorState : public IState<AppContext> {
public:
  const char *Name() const override { return "Error"; }

  void OnEnter(AppContext &ctx, SmTick now) override;
  void OnStep(AppContext &ctx, SmTick now) override;
  void OnExit(AppContext &ctx, SmTick now) override;
}; // TODO: Implement