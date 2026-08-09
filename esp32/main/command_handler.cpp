// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "command_handler.hpp"

#include <cstring>

#include "ctx.hpp"
#include "dispatcher.hpp"
#include "error_code.hpp"
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
static void OnTelemetry(AppContext &ctx, const message::Packet &pkt) {
  ctx.sys->Mavlink().UpdateTelemetryCache(message::PayloadAs<T>(pkt),
                                          ctx.sys->Timebase().NowMs());
}

template <typename T>
static void OnConfig(AppContext &ctx, const message::Packet &pkt) {
  ctx.sys->Mavlink().UpdateConfigCache(message::PayloadAs<T>(pkt),
                                       ctx.sys->Timebase().NowMs());
}

// Expected traffic this side has nothing to do with. Listed rather than
// omitted: an absent id is an unknown one, and unknown ids panic.
static void OnIgnored(AppContext &, const message::Packet &) {}

static void OnLog(AppContext &, const message::Packet &pkt) {
  if (pkt.header.len == 0) {
    return;
  }
  // Dispatch validated the length against kMaxLogTextPayload already.
  char buf[message::kMaxLogTextPayload + 1];
  memcpy(buf, pkt.payload, pkt.header.len);
  buf[pkt.header.len] = '\0';
  ESP_LOGI("FC", "%s", buf);
}

static void OnTone(AppContext &ctx, const message::Packet &pkt) {
  ctx.sys->TonePlayer().PlayBuiltin(static_cast<message::Tone>(
      message::PayloadAs<message::ToneMsg>(pkt).tone));
  ctx.sys->Ui().NotifyUserActivity();
}

static void OnPanic(AppContext &ctx, const message::Packet &pkt) {
  const uint32_t error_code =
      message::PayloadAs<message::PanicMsg>(pkt).error_code;

  // Killing the bridge mid-write is worse than carrying on with a faulted
  // STM32, and a four-way session is writing ESC firmware.
  if (ctx.sm->CurrentState() == ctx.dfu_state ||
      ctx.sm->CurrentState() == ctx.esc_config_state) {
    static int64_t last_log_us = 0;
    int64_t now_us = esp_timer_get_time();
    if (now_us - last_log_us >= 5000000) {
      ctx.sys->Mavlink().ReportPanic(Mavlink::PanicSource::kStm32, error_code);
      ESP_LOGE(kTag, "STM32 Panic (Ignored in DFU): Code 0x%08lX: %s",
               static_cast<unsigned long>(error_code), GetMessage(error_code));
      last_log_us = now_us;
    }
    return;
  }

  ctx.sys->Mavlink().ReportPanic(Mavlink::PanicSource::kStm32, error_code);
  // Enter panic state with error code - this will never return
  Panic(error_code);
}

static void OnUsbStatus(AppContext &ctx, const message::Packet &pkt) {
  const auto &msg = message::PayloadAs<message::UsbStatusMsg>(pkt);
  const bool granted = (msg.flags & message::kUsbStatusEscConfigGranted) != 0u;
  ctx.sys->Ui().UpdatePeerUsb(
      {
          .attached = (msg.flags & message::kUsbStatusAttached) != 0u,
          .configured = (msg.flags & message::kUsbStatusConfigured) != 0u,
          .port_open = (msg.flags & message::kUsbStatusPortOpen) != 0u,
          .esc_config_granted = granted,
          .rx_frames = msg.rx_frames,
          .tx_frames = msg.tx_frames,
      },
      ctx.sys->Timebase().NowMs());

  // The STM32 latches the grant, so correcting drift is this side's job.
  // Answering the report that carries it covers a command lost in either
  // direction, and a reboot that left the port open. Armed is excluded
  // because the STM32 refuses then, and asking anyway just loops.
  const bool want = ctx.sm->CurrentState() == ctx.esc_config_state &&
                    !ctx.sys->Mavlink().PeerArmed().value_or(false);
  if (granted != want) {
    ctx.sys->FcLink().SendPacket(
        message::MsgId::kSetEscConfigMode,
        message::SetEscConfigModeMsg{.enabled = static_cast<uint8_t>(want)});
  }
}

static const Dispatcher<AppContext>::Entry kHandlers[] = {
    {message::MsgId::kPong, OnIgnored},
    {message::MsgId::kImuData, OnIgnored},
    {message::MsgId::kLog, OnLog},
    {message::MsgId::kTone, OnTone},
    {message::MsgId::kUsbStatus, OnUsbStatus},
    {message::MsgId::kPanic, OnPanic},
    {message::MsgId::kGpsData, OnTelemetry<message::GpsData>},
    {message::MsgId::kRcChannels, OnTelemetry<message::RcChannelsMsg>},
    {message::MsgId::kSystemStatus, OnTelemetry<message::SystemStatusMsg>},
    {message::MsgId::kVehicleStatus, OnTelemetry<message::VehicleStatusMsg>},
    {message::MsgId::kEscTelemetry, OnTelemetry<message::EscTelemetryMsg>},
    {message::MsgId::kRcMapConfig, OnConfig<message::RcMapConfigMsg>},
    {message::MsgId::kRcCalibrationConfig,
     OnConfig<message::RcCalibrationConfigMsg>},
    {message::MsgId::kGyroCalibrationIdConfig,
     OnConfig<message::GyroCalibrationIdConfigMsg>},
};

static const Dispatcher<AppContext> kDispatcher(kHandlers);

void CommandHandler::Init(const Config &cfg) {
  cfg_ = cfg;
  ESP_LOGI(kTag, "Initialized");
}

void CommandHandler::Dispatch(AppContext &ctx, const message::Packet &pkt) {
  if (!message::IsPacketValid(pkt.header.id, pkt.payload, pkt.header.len)) {
    ESP_LOGW(kTag, "Rejected invalid packet id=0x%02X len=%u",
             (unsigned)pkt.header.id, (unsigned)pkt.header.len);
    Panic(ErrorCode::Common::kCommandInvalidPacket);
  }

  if (!kDispatcher.Dispatch(ctx, pkt)) {
    PanicUnknownCommand(pkt.header.id);
  }
}

CommandHandler::DfuTcpAction CommandHandler::Dispatch(
    AppContext &ctx, const TcpServer::Event &ev) {
  switch (ev.id) {
    case TcpServer::EventId::kBegin: {
      ctx.sys->Tcp().StartDownload(ev.begin.size);
      ctx.sys->Programmer().SetTarget(ev.begin.target);

      ESP_LOGI(kTag, "TCP: BEGIN size=%u crc=%u target=%d",
               (unsigned)ev.begin.size, (unsigned)ev.begin.crc,
               (int)ev.begin.target);
      return DfuTcpAction::kEnterProgram;
    }
    case TcpServer::EventId::kAbort: {
      ctx.sys->Tcp().StopDownload();
      ESP_LOGI(kTag, "TCP: ABORT");
      return DfuTcpAction::kStayInDfu;
    }
    case TcpServer::EventId::kReset: {
      ctx.sys->Tcp().DisableBridge();
      ESP_LOGW(kTag, "TCP: RESET requested. Rebooting...");
      ctx.sys->Programmer().Boot();
      esp_restart();
      return DfuTcpAction::kStayInDfu;
    }
    case TcpServer::EventId::kBridge: {
      ESP_LOGI(kTag, "TCP: BRIDGE requested");
      ctx.sys->Tcp().EnableBridge();
      return DfuTcpAction::kStayInDfu;
    }
    default:
      break;
  }
  return DfuTcpAction::kStayInDfu;
}
