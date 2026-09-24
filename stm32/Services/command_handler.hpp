// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include "message.hpp"

struct StateMachineContext;

class CommandHandler {
 public:
  static CommandHandler &GetInstance();

  bool Dispatch(const StateMachineContext &ctx, const message::Packet &pkt);

 private:
  friend class System;
  void Init();

  CommandHandler() = default;
  ~CommandHandler() = default;
  CommandHandler(const CommandHandler &) = delete;
  CommandHandler &operator=(const CommandHandler &) = delete;

  bool initialized_ = false;
};
