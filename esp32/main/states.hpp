// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once
#include "state_machine.hpp"

struct AppContext;

class ServingState : public IState<AppContext> {
 public:
  const char *Name() const override { return "Serving"; }

  void OnEnter(AppContext &ctx) override;
  void OnStep(AppContext &ctx, SmTick now) override;
};

class MavlinkWifiState : public IState<AppContext> {
 public:
  const char *Name() const override { return "MavlinkWifi"; }

  void OnEnter(AppContext &ctx) override;
  void OnStep(AppContext &ctx, SmTick now) override;
};

class MavlinkUsbState : public IState<AppContext> {
 public:
  const char *Name() const override { return "MavlinkUsb"; }

  void OnEnter(AppContext &ctx) override;
  void OnStep(AppContext &ctx, SmTick now) override;
};

class DfuState : public IState<AppContext> {
 public:
  const char *Name() const override { return "Dfu"; }

  void OnEnter(AppContext &ctx) override;
  void OnStep(AppContext &ctx, SmTick now) override;
};

class ProgramState : public IState<AppContext> {
 public:
  const char *Name() const override { return "Program"; }

  void OnEnter(AppContext &ctx) override;
  void OnStep(AppContext &ctx, SmTick now) override;

 private:
  SmTick last_activity_ = 0;
  uint32_t last_written_ = 0;
};

class EscConfigState : public IState<AppContext> {
 public:
  const char *Name() const override { return "EscConfig"; }

  void OnEnter(AppContext &ctx) override;
  void OnStep(AppContext &ctx, SmTick now) override;

 private:
  void RequestEscConfig(AppContext &ctx, bool enabled) const;

  bool warned_armed_ = false;
  bool stream_seen_ = false;
  uint16_t last_usb_frames_ = 0;
  uint16_t request_attempts_ = 0;
  uint32_t last_request_ms_ = 0;
};
