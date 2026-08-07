// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "command_handler.hpp"

#include <cstring>
#include <type_traits>

#include "ctx.hpp"
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
#include "freertos/FreeRTOS.h"  // IWYU pragma: keep
#include "freertos/task.h"
#include "panic.hpp"
}

static constexpr const char *kTag = "cmd";
#include "esp_timer.h"  // Added for esp_timer_get_time

template <typename T>
const T &PayloadAs(const message::Packet &pkt) {
  static_assert(std::is_trivially_copyable<T>::value,
                "packet payload type must be trivially copyable");
  static_assert(alignof(T) == 1,
                "packet payload type must be packed for zero-copy decode");
  static_assert(sizeof(T) <= message::kMaxPayload,
                "packet payload type exceeds wire payload limit");
  // Dispatch validates id/length first; protocol structs are packed so this
  // zero-copy view avoids another telemetry payload copy.
  return *reinterpret_cast<const T *>(pkt.payload);
}

[[noreturn]] static void PanicUnknownCommand(uint8_t id) {
  ESP_LOGE(kTag, "Unknown Command ID: 0x%02X (%u)", (unsigned)id, (unsigned)id);
  Panic(ErrorCode::Common::kUnknownCommand);
}

static ::TonePlayer::BuiltinTone ToneFor(uint8_t tone) {
  switch (tone) {
    case message::kToneConfirm:
      return ::TonePlayer::BuiltinTone::kConfirm;
    case message::kToneWarning:
      return ::TonePlayer::BuiltinTone::kWarning;
    case message::kToneError:
      return ::TonePlayer::BuiltinTone::kError;
    case message::kToneBeep:
    default:
      return ::TonePlayer::BuiltinTone::kBeep;
  }
}

static void OnLog(AppContext &, const message::Packet &pkt) {
  if (!message::IsPayloadValid(message::MsgId::kLog, pkt.payload,
                               pkt.header.len) ||
      pkt.header.len == 0) {
    return;
  }
  char buf[257];
  memcpy(buf, pkt.payload, pkt.header.len);
  buf[pkt.header.len] = '\0';
  ESP_LOGI("FC", "%s", buf);
}

static void OnUnknown(AppContext &, const message::Packet &pkt) {
  PanicUnknownCommand(pkt.header.id);
}

static void OnPanic(AppContext &ctx, const message::Packet &pkt) {
  const uint32_t error_code = PayloadAs<message::PanicMsg>(pkt).error_code;

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

  const uint32_t now_ms = ctx.sys->Timebase().NowMs();

  switch (static_cast<message::MsgId>(pkt.header.id)) {
    case message::MsgId::kPong:
      break;
    case message::MsgId::kGpsData:
      ctx.sys->Mavlink().UpdateTelemetryCache(PayloadAs<message::GpsData>(pkt),
                                              now_ms);
      break;
    case message::MsgId::kLog:
      OnLog(ctx, pkt);
      break;
    case message::MsgId::kRcMapConfig:
      ctx.sys->Mavlink().UpdateConfigCache(
          PayloadAs<message::RcMapConfigMsg>(pkt), now_ms);
      break;
    case message::MsgId::kRcCalibrationConfig:
      ctx.sys->Mavlink().UpdateConfigCache(
          PayloadAs<message::RcCalibrationConfigMsg>(pkt), now_ms);
      break;
    case message::MsgId::kRcChannels:
      ctx.sys->Mavlink().UpdateTelemetryCache(
          PayloadAs<message::RcChannelsMsg>(pkt), now_ms);
      break;
    case message::MsgId::kSystemStatus:
      ctx.sys->Mavlink().UpdateTelemetryCache(
          PayloadAs<message::SystemStatusMsg>(pkt), now_ms);
      break;
    case message::MsgId::kTone:
      ctx.sys->TonePlayer().PlayBuiltin(
          ToneFor(PayloadAs<message::ToneMsg>(pkt).tone));
      ctx.sys->Ui().NotifyUserActivity();
      break;

    case message::MsgId::kUsbStatus: {
      const auto &msg = PayloadAs<message::UsbStatusMsg>(pkt);
      const bool granted =
          (msg.flags & message::kUsbStatusEscConfigGranted) != 0u;
      ctx.sys->Ui().UpdatePeerUsb(
          {
              .attached = (msg.flags & message::kUsbStatusAttached) != 0u,
              .configured = (msg.flags & message::kUsbStatusConfigured) != 0u,
              .port_open = (msg.flags & message::kUsbStatusPortOpen) != 0u,
              .esc_config_granted = granted,
              .rx_frames = msg.rx_frames,
              .tx_frames = msg.tx_frames,
          },
          now_ms);

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
      break;
    }

    case message::MsgId::kVehicleStatus:
      ctx.sys->Mavlink().UpdateTelemetryCache(
          PayloadAs<message::VehicleStatusMsg>(pkt), now_ms);
      break;
    case message::MsgId::kEscTelemetry:
      ctx.sys->Mavlink().UpdateTelemetryCache(
          PayloadAs<message::EscTelemetryMsg>(pkt), now_ms);
      break;
    case message::MsgId::kGyroCalibrationIdConfig:
      ctx.sys->Mavlink().UpdateConfigCache(
          PayloadAs<message::GyroCalibrationIdConfigMsg>(pkt), now_ms);
      break;
    case message::MsgId::kImuData:
      break;
    case message::MsgId::kPanic:
      OnPanic(ctx, pkt);
      break;
    default:
      OnUnknown(ctx, pkt);
      break;
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
