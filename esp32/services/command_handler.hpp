#pragma once

#include "message.hpp"

// Forward declaration
struct AppContext;

class CommandHandler {
public:
  struct Config {};

  static CommandHandler &GetInstance() {
    static CommandHandler instance;
    return instance;
  }

  void Init(const Config &cfg);

  // Dispatch uses a lookup table for O(1) access
  bool Dispatch(AppContext &ctx, const message::Packet &pkt);

private:
  CommandHandler() = default;
  ~CommandHandler() = default;
  CommandHandler(const CommandHandler &) = delete;
  CommandHandler &operator=(const CommandHandler &) = delete;

  bool initialized_ = false;
  Config cfg_;

  using HandlerFunc = void (*)(AppContext &ctx, const message::Packet &pkt);
  HandlerFunc handlers_[256];
};
