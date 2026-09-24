// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once
#include "icm42688p.hpp"
#include "state_machine.hpp"

struct StateMachineContext;

struct IControlTickState {
  virtual ~IControlTickState() = default;
  virtual void OnControlTick(StateMachineContext &ctx) = 0;
};

struct StandbyState : public IState<StateMachineContext>,
                      public IControlTickState {
  const char *Name() const override { return "Standby"; }
  void OnEnter(StateMachineContext &ctx) override;
  void OnStep(StateMachineContext &ctx) override;
  void OnControlTick(StateMachineContext &ctx) override;
};

struct ArmedState : public IState<StateMachineContext>,
                    public IControlTickState {
  const char *Name() const override { return "Armed"; }
  void OnEnter(StateMachineContext &ctx) override;
  void OnExit(StateMachineContext &ctx) override;
  void OnStep(StateMachineContext &ctx) override;
  void OnControlTick(StateMachineContext &ctx) override;
};

struct EscConfigState : public IState<StateMachineContext> {
  const char *Name() const override { return "EscConfig"; }
  void OnEnter(StateMachineContext &ctx) override;
  void OnExit(StateMachineContext &ctx) override;
  void OnStep(StateMachineContext &ctx) override;

 private:
};

struct MscState : public IState<StateMachineContext> {
  const char *Name() const override { return "Msc"; }
  void OnEnter(StateMachineContext &ctx) override;
  void OnExit(StateMachineContext &ctx) override;
  void OnStep(StateMachineContext &ctx) override;
};
