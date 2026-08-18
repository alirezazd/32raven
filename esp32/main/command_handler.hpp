// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include "ctx.hpp"
#include "message.hpp"
#include "tcp_server.hpp"

class CommandHandler {
 public:
  struct Config {};
  enum class DfuTcpAction : uint8_t {
    kStayInDfu = 0,
    kEnterProgram,
    kEnterLogPull,
  };

  static CommandHandler &GetInstance();

  void Init(const Config &cfg);

  void Dispatch(const AppContext &ctx, const message::Packet &pkt);

  DfuTcpAction Dispatch(const AppContext &ctx, const TcpServer::Event &ev);

 private:
  CommandHandler() = default;
  ~CommandHandler() = default;
  CommandHandler(const CommandHandler &) = delete;
  CommandHandler &operator=(const CommandHandler &) = delete;
  Config cfg_;
};
