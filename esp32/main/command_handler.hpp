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
  };

  static CommandHandler &GetInstance() {
    static CommandHandler instance;
    return instance;
  }

  void Init(const Config &cfg);

  void Dispatch(AppContext &ctx, const message::Packet &pkt);

 DfuTcpAction Dispatch(AppContext &ctx, const TcpServer::Event &ev);

 private:
  CommandHandler() = default;
  ~CommandHandler() = default;
  CommandHandler(const CommandHandler &) = delete;
  CommandHandler &operator=(const CommandHandler &) = delete;
  Config cfg_;
};
