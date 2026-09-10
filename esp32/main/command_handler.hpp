// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include "ctx.hpp"
#include "host_link.hpp"
#include "message.hpp"

class CommandHandler {
 public:
  struct Config {};

  static CommandHandler &GetInstance();

  void Dispatch(const AppContext &ctx, const message::Packet &pkt);

  void Dispatch(AppContext &ctx, const HostLink::Event &ev);

 private:
  friend class System;

  void Init(const Config &cfg);
  CommandHandler() = default;
  ~CommandHandler() = default;
  CommandHandler(const CommandHandler &) = delete;
  CommandHandler &operator=(const CommandHandler &) = delete;
  Config cfg_;
};
