// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include "dispatcher.hpp"

struct AppContext;

class CommandHandler {
 public:
  static CommandHandler &GetInstance();

  void Init();
  bool Dispatch(const AppContext &ctx, const message::Packet &pkt);

 private:
  CommandHandler() = default;
  ~CommandHandler() = default;
  CommandHandler(const CommandHandler &) = delete;
  CommandHandler &operator=(const CommandHandler &) = delete;
};
