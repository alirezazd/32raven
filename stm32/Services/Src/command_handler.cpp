#include "command_handler.hpp"
#include "message.hpp"
#include "panic.hpp"
#include "states.hpp" // For AppContext definition
#include "system.hpp" // For LED/UART access

static void OnPing(AppContext &ctx, const message::Packet &pkt) {
  (void)pkt;
  // Send Pong
  message::Packet tx_pkt;
  tx_pkt.header.id = (uint8_t)message::MsgId::kPong;
  tx_pkt.header.len = 0;
  ctx.sys->GetFcLink().Send(tx_pkt);
}
static void OnConfig(AppContext &ctx, const message::Packet &pkt) {
  if (pkt.header.len != sizeof(message::ConfigMsg)) {
    return;
  }
  const auto *cfg = (const message::ConfigMsg *)pkt.payload;
  if (cfg->telemetry_rate_hz > 0) {
    ctx.telemetry_interval_ms = 1000 / cfg->telemetry_rate_hz;
  }
}

static void OnRcChannels(AppContext &ctx, const message::Packet &pkt) {
  (void)ctx;
  (void)pkt;
  // TODO: Forward to RC Service
}

static const Epistole::Dispatcher<AppContext>::Entry kHandlers[] = {
    {message::MsgId::kPing, OnPing},
    {message::MsgId::kConfig, OnConfig},
    {message::MsgId::kRcChannels, OnRcChannels},
};

static const Epistole::Dispatcher<AppContext>
    kDispatcher(kHandlers, sizeof(kHandlers) / sizeof(kHandlers[0]));

void CommandHandler::Init() {}

bool CommandHandler::Dispatch(AppContext &ctx, const message::Packet &pkt) {
  return kDispatcher.Dispatch(ctx, pkt);
}
