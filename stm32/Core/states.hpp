// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once
#include "ctx.hpp"
#include "icm42688p.hpp"
#include "state_machine.hpp"

struct IControlTickState {
  virtual ~IControlTickState() = default;
  virtual void OnControlTick(AppContext &ctx) = 0;
};

struct IdleState : public IState<AppContext>, public IControlTickState {
  const char *Name() const override { return "Idle"; }
  void OnEnter(AppContext &ctx) override;
  void OnStep(AppContext &ctx) override;
  void OnControlTick(AppContext &ctx) override;
};

struct ArmedState : public IState<AppContext>, public IControlTickState {
  const char *Name() const override { return "Armed"; }
  void OnEnter(AppContext &ctx) override;
  void OnExit(AppContext &ctx) override;
  void OnStep(AppContext &ctx) override;
  void OnControlTick(AppContext &ctx) override;
};

struct EscConfigState : public IState<AppContext> {
  const char *Name() const override { return "EscConfig"; }
  void OnEnter(AppContext &ctx) override;
  void OnExit(AppContext &ctx) override;
  void OnStep(AppContext &ctx) override;

 private:
};

struct MscState : public IState<AppContext> {
  const char *Name() const override { return "Msc"; }
  void OnEnter(AppContext &ctx) override;
  void OnExit(AppContext &ctx) override;
  void OnStep(AppContext &ctx) override;
};
