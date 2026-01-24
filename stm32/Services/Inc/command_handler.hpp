#pragma once

#include "dispatcher.hpp"

struct AppContext;

class CommandHandler {
public:
  static CommandHandler &GetInstance() {
    static CommandHandler instance;
    return instance;
  }

  void Init();
  bool Dispatch(AppContext &ctx, const message::Packet &pkt);

private:
  CommandHandler() = default;
  ~CommandHandler() = default;
  CommandHandler(const CommandHandler &) = delete;
  CommandHandler &operator=(const CommandHandler &) = delete;
};
