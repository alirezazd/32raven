#pragma once

#include "ctx.hpp"
#include "mavlink.hpp"
#include "message.hpp"
#include "tcp_server.hpp"

class CommandHandler {
 public:
  struct Config {};

  static CommandHandler &GetInstance() {
    static CommandHandler instance;
    return instance;
  }

  void Init(const Config &cfg);

  enum class TransitionResult {
    kNone,
    kUnknown,
    kTransitionToProgram,
    kTransitionToServing,
    kTransitionToBridge,
    kTransitionToDfu,
    kTransitionToHardError
  };

  // Dispatch uses a lookup table for O(1) access
  void Dispatch(AppContext &ctx, const message::Packet &pkt);

  // Dispatch TCP events
  TransitionResult Dispatch(AppContext &ctx, const TcpServer::Event &ev);

  // Dispatch MAVLink command events
  void Dispatch(AppContext &ctx, const Mavlink::CommandLongEvent &ev);

 private:
  CommandHandler() = default;
  ~CommandHandler() = default;
  CommandHandler(const CommandHandler &) = delete;
  CommandHandler &operator=(const CommandHandler &) = delete;
  Config cfg_;
  using HandlerFunc = void (*)(AppContext &ctx, const message::Packet &pkt);
  HandlerFunc handlers_[256];
};
