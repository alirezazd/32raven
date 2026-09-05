// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "command_handler.hpp"

#include <cstring>

#include "ctx.hpp"
#include "dispatcher.hpp"
#include "error_code.hpp"
#include "fc_link.hpp"
#include "mavlink.hpp"
#include "message.hpp"
#include "panic.hpp"
#include "state_machine.hpp"
#include "states.hpp"
#include "system.hpp"
#include "tcp_server.hpp"

extern "C" {
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"  // IWYU pragma: keep
#include "freertos/task.h"
}

static constexpr const char *kTag = "cmd";

[[noreturn]] static void PanicUnknownCommand(uint8_t id) {
  ESP_LOGE(kTag, "Unknown Command ID: 0x%02X (%u)", (unsigned)id, (unsigned)id);
  Panic(ErrorCode::Common::kUnknownCommand);
}

// Every arm that only refreshes a cache differs solely in the payload type,
// and the table below is where an id gets paired with one. PayloadAs cannot
// check that pairing, so keeping it to a single line per message is the whole
// defence against routing a packet into the wrong struct.
template <typename T>
static void OnTelemetry(const AppContext &ctx, const message::Packet &pkt) {
  ctx.sys->Mavlink().UpdateTelemetryCache(message::PayloadAs<T>(pkt),
                                          ctx.sys->Timebase().NowMs());
}

template <typename T>
static void OnConfig(const AppContext &ctx, const message::Packet &pkt) {
  ctx.sys->Mavlink().UpdateConfigCache(message::PayloadAs<T>(pkt),
                                       ctx.sys->Timebase().NowMs());
}

// Expected traffic this side has nothing to do with. Listed rather than
// omitted: an absent id is an unknown one, and unknown ids panic.
static void OnIgnored(const AppContext &, const message::Packet &) {}

static void OnLog(const AppContext &, const message::Packet &pkt) {
  if (pkt.header.len == 0) {
    return;
  }
  // Dispatch validated the length against kMaxLogTextPayload already.
  char buf[message::kMaxLogTextPayload + 1];
  memcpy(buf, pkt.payload, pkt.header.len);
  buf[pkt.header.len] = '\0';
  ESP_LOGI(FcLink::kPeerLogTag, "%s", buf);
}

static void OnTone(const AppContext &ctx, const message::Packet &pkt) {
  ctx.sys->TonePlayer().PlayBuiltin(static_cast<message::Tone>(
      message::PayloadAs<message::ToneMsg>(pkt).tone));
  ctx.sys->Ui().NotifyUserActivity();
}

static void OnAccelCalStatus(const AppContext &ctx,
                             const message::Packet &pkt) {
  ctx.sys->Mavlink().ReportAccelCalProgress(
      message::PayloadAs<message::AccelCalStatusMsg>(pkt));
}

static void OnPanic(const AppContext &ctx, const message::Packet &pkt) {
  const uint32_t error_code =
      message::PayloadAs<message::PanicMsg>(pkt).error_code;

  // Killing the bridge mid-write is worse than carrying on with a faulted
  // STM32, and a four-way session is writing ESC firmware.
  if (ctx.sm->CurrentState() == ctx.service_state ||
      ctx.sm->CurrentState() == ctx.esc_config_state) {
    static int64_t last_log_us = 0;
    int64_t now_us = esp_timer_get_time();
    if (now_us - last_log_us >= 5000000) {
      ctx.sys->Mavlink().ReportPanic(Mavlink::PanicSource::kStm32, error_code);
      ESP_LOGE(kTag, "STM32 Panic (Ignored in Service): Code 0x%08lX: %s",
               static_cast<unsigned long>(error_code), GetMessage(error_code));
      last_log_us = now_us;
    }
    return;
  }

  ctx.sys->Mavlink().ReportPanic(Mavlink::PanicSource::kStm32, error_code);
  // Enter panic state with error code - this will never return
  Panic(error_code);
}

static void OnUsbStatus(const AppContext &ctx, const message::Packet &pkt) {
  const auto &msg = message::PayloadAs<message::UsbStatusMsg>(pkt);
  const auto reported = static_cast<message::UsbMode>(msg.mode);
  const uint32_t now_ms = ctx.sys->Timebase().NowMs();
  ctx.sys->Ui().UpdatePeerUsb(
      {
          .attached = (msg.flags & message::kUsbStatusAttached) != 0u,
          .configured = (msg.flags & message::kUsbStatusConfigured) != 0u,
          .port_open = (msg.flags & message::kUsbStatusPortOpen) != 0u,
          .mode = reported,
          .rx_frames = msg.rx_frames,
          .tx_frames = msg.tx_frames,
      },
      now_ms);

  // The STM32 latches the mode, so correcting drift is this side's job.
  // Answering the report that carries it covers a command lost in either
  // direction, and a reboot that left the port open. Armed is excluded
  // because the STM32 refuses then, and asking anyway just loops. Unknown
  // arm reads as disarmed on purpose: this frame proves the peer is live.
  message::UsbMode want = message::UsbMode::kNone;
  if (!ctx.sys->Mavlink().PeerArmed(now_ms).value_or(false)) {
    if (ctx.sm->CurrentState() == ctx.esc_config_state) {
      want = message::UsbMode::kEscConfig;
    } else if (ctx.sm->CurrentState() == ctx.usb_log_state) {
      want = message::UsbMode::kMsc;
    }
  }
  if (reported != want) {
    ctx.sys->FcLink().SendPacket(
        message::MsgId::kSetUsbMode,
        message::SetUsbModeMsg{.mode = static_cast<uint8_t>(want)});
  }
}

// Stale replies to an abandoned pull are not a protocol error.
static void OnLogListReply(const AppContext &ctx, const message::Packet &pkt) {
  if (ctx.sm->CurrentState() == ctx.log_pull_state) {
    ctx.log_pull_state->OnListReply(
        message::PayloadAs<message::LogListReplyMsg>(pkt));
  }
}

static void OnLogData(const AppContext &ctx, const message::Packet &pkt) {
  if (ctx.sm->CurrentState() == ctx.log_pull_state) {
    ctx.log_pull_state->OnData(message::PayloadAs<message::LogDataMsg>(pkt));
  }
}

static const Dispatcher<const AppContext>::Entry kHandlers[] = {
    {message::MsgId::kHandshakeReply, OnIgnored},
    {message::MsgId::kLog, OnLog},
    {message::MsgId::kTone, OnTone},
    {message::MsgId::kAccelCalStatus, OnAccelCalStatus},
    {message::MsgId::kUsbStatus, OnUsbStatus},
    {message::MsgId::kPanic, OnPanic},
    {message::MsgId::kGpsData, OnTelemetry<message::GpsData>},
    {message::MsgId::kAttitude, OnTelemetry<message::AttitudeMsg>},
    {message::MsgId::kRcChannels, OnTelemetry<message::RcChannelsMsg>},
    {message::MsgId::kSystemStatus, OnTelemetry<message::SystemStatusMsg>},
    {message::MsgId::kVehicleStatus, OnTelemetry<message::VehicleStatusMsg>},
    {message::MsgId::kEscTelemetry, OnTelemetry<message::EscTelemetryMsg>},
    {message::MsgId::kRcMapConfig, OnConfig<message::RcMapConfigMsg>},
    {message::MsgId::kRcCalibrationConfig,
     OnConfig<message::RcCalibrationConfigMsg>},
    {message::MsgId::kGyroCalibrationIdConfig,
     OnConfig<message::GyroCalibrationIdConfigMsg>},
    {message::MsgId::kLogListReply, OnLogListReply},
    {message::MsgId::kLogData, OnLogData},
};

static const Dispatcher<const AppContext> kDispatcher(kHandlers);

CommandHandler &CommandHandler::GetInstance() {
  static CommandHandler instance;
  return instance;
}

void CommandHandler::Init(const Config &cfg) {
  cfg_ = cfg;
  ESP_LOGI(kTag, "Initialized");
}

void CommandHandler::Dispatch(const AppContext &ctx, const message::Packet &pkt) {
  if (!message::IsPacketValid(pkt.header.id, pkt.payload, pkt.header.len)) {
    ESP_LOGW(kTag, "Rejected invalid packet id=0x%02X len=%u",
             (unsigned)pkt.header.id, (unsigned)pkt.header.len);
    Panic(ErrorCode::Common::kCommandInvalidPacket);
  }

  if (!kDispatcher.Dispatch(ctx, pkt)) {
    PanicUnknownCommand(pkt.header.id);
  }
}

CommandHandler::ServiceTcpAction CommandHandler::Dispatch(
    const AppContext &ctx, const TcpServer::Event &ev) {
  switch (ev.id) {
    case TcpServer::EventId::kBegin: {
      ctx.sys->Tcp().BeginTransfer(ev.begin.size);
      ctx.sys->Programmer().SetTarget(ev.begin.target);

      ESP_LOGI(kTag, "TCP: BEGIN size=%u crc=%u target=%s",
               (unsigned)ev.begin.size, (unsigned)ev.begin.crc,
               ev.begin.target[0] != '\0' ? ev.begin.target : "stm32");
      return ServiceTcpAction::kEnterProgram;
    }
    case TcpServer::EventId::kLogList: {
      ctx.log_pull_state->PrepareList();
      return ServiceTcpAction::kEnterLogPull;
    }
    case TcpServer::EventId::kLogGet: {
      ctx.log_pull_state->PrepareGet(ev.log_name);
      return ServiceTcpAction::kEnterLogPull;
    }
    case TcpServer::EventId::kAbort: {
      ctx.sys->Tcp().EndTransfer();
      ESP_LOGI(kTag, "TCP: ABORT");
      return ServiceTcpAction::kStayInService;
    }
    case TcpServer::EventId::kReset: {
      ctx.sys->Tcp().CloseDataRx();
      ESP_LOGW(kTag, "TCP: RESET requested. Rebooting...");
      ctx.sys->Programmer().Boot();
      esp_restart();
      return ServiceTcpAction::kStayInService;
    }
    case TcpServer::EventId::kBridge: {
      ESP_LOGI(kTag, "TCP: BRIDGE requested");
      ctx.sys->Tcp().OpenDataRx();
      return ServiceTcpAction::kStayInService;
    }
    default:
      break;
  }
  return ServiceTcpAction::kStayInService;
}
