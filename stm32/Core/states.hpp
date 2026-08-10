// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once
#include "ctx.hpp"
#include "icm42688p.hpp"
#include "state_machine.hpp"

struct IFastTickState {
  virtual ~IFastTickState() = default;
  virtual void OnFastTick(AppContext &ctx,
                          const Icm42688p::SampleBatch &batch) = 0;
};

struct IdleState : public IState<AppContext>, public IFastTickState {
  const char *Name() const override { return "Idle"; }
  void OnEnter(AppContext &ctx) override;
  void OnStep(AppContext &ctx, SmTick now) override;
  void OnFastTick(AppContext &ctx,
                  const Icm42688p::SampleBatch &batch) override;
};

struct ArmedState : public IState<AppContext>, public IFastTickState {
  const char *Name() const override { return "Armed"; }
  void OnEnter(AppContext &ctx) override;
  void OnStep(AppContext &ctx, SmTick now) override;
  void OnFastTick(AppContext &ctx,
                  const Icm42688p::SampleBatch &batch) override;
};

struct EscConfigState : public IState<AppContext> {
  const char *Name() const override { return "EscConfig"; }
  void OnEnter(AppContext &ctx) override;
  void OnStep(AppContext &ctx, SmTick now) override;

 private:
  uint32_t last_status_send_us_ = 0;
};
