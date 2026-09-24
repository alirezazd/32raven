// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include "host_link.hpp"
#include "message.hpp"
#include "state_machine_context.hpp"

class CommandHandler {
 public:
  struct Config {};

  static CommandHandler &GetInstance();

  void Dispatch(StateMachineContext &ctx, const message::Packet &pkt);

  void Dispatch(StateMachineContext &ctx, const HostLink::Event &ev);

 private:
  friend class System;

  void Init(const Config &cfg);
  CommandHandler() = default;
  ~CommandHandler() = default;
  CommandHandler(const CommandHandler &) = delete;
  CommandHandler &operator=(const CommandHandler &) = delete;
  Config cfg_;
};
