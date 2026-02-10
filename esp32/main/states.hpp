#pragma once
#include "state_machine.hpp"

struct AppContext;

class ServingState : public IState<AppContext> {
public:
  const char *Name() const override { return "Serving"; }

  void OnEnter(AppContext &ctx, SmTick now) override;
  void OnStep(AppContext &ctx, SmTick now) override;
  void OnExit(AppContext &ctx, SmTick now) override;

private:
  bool in_flight_ = false;
};

class DfuState : public IState<AppContext> {
public:
  const char *Name() const override { return "Dfu"; }

  void OnEnter(AppContext &ctx, SmTick now) override;
  void OnStep(AppContext &ctx, SmTick now) override;
  void OnExit(AppContext &ctx, SmTick now) override;
};

class ProgramState : public IState<AppContext> {
public:
  const char *Name() const override { return "Program"; }

  void OnEnter(AppContext &ctx, SmTick now) override;
  void OnStep(AppContext &ctx, SmTick now) override;
  void OnExit(AppContext &ctx, SmTick now) override;

private:
  SmTick last_activity_ = 0;
  uint32_t last_written_ = 0;
};

class HardErrorState : public IState<AppContext> {
public:
  const char *Name() const override { return "HardError"; }

  void OnEnter(AppContext &ctx, SmTick now) override;
  void OnStep(AppContext &ctx, SmTick now) override;
  void OnExit(AppContext &ctx, SmTick now) override;
};