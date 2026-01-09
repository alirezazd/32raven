#pragma once
#include "state_machine.hpp"
#include <cstddef>

struct AppContext;

class IdleState : public IState<AppContext> {
public:
  const char *Name() const override { return "Idle"; }

  void OnEnter(AppContext &ctx, SmTick now) override;
  void OnStep(AppContext &ctx, SmTick now) override;
  void OnExit(AppContext &ctx, SmTick now) override;

private:
  char line_buf_[32]{};
  size_t line_idx_ = 0;
};

class ListenState : public IState<AppContext> {
public:
  const char *Name() const override { return "Listen"; }

  void SetIdleBlinkPeriod(SmTick ms) { period_ms_ = ms; }
  void SetListenBlinkPeriod(SmTick ms) { period_ms_ = ms; }

  void OnEnter(AppContext &ctx, SmTick now) override;
  void OnStep(AppContext &ctx, SmTick now) override;
  void OnExit(AppContext &ctx, SmTick now) override;

  enum class ListenMode { kHard, kSoft };
  void SetMode(ListenMode m) { mode_ = m; }

private:
  ListenMode mode_ = ListenMode::kHard;
  SmTick last_active_ = 0;
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
};