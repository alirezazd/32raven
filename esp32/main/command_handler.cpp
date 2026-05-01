#include "command_handler.hpp"

#include <cstring>
#include <type_traits>

#include "ctx.hpp"
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
  Panic(ErrorCode::kUnknownCommand);
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
  const ErrorCode error_code = PayloadAs<message::PanicMsg>(pkt).error_code;

  // In DFU mode, we ignore the panic to allow for flashing/debugging
  if (ctx.sm->CurrentState() == ctx.dfu_state) {
    static int64_t last_log_us = 0;
    int64_t now_us = esp_timer_get_time();
    if (now_us - last_log_us >= 5000000) {
      ctx.sys->Mavlink().ReportPanic(Mavlink::PanicSource::kStm32,
                                     error_code);
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
    Panic(ErrorCode::kCommandInvalidPacket);
  }

  const uint32_t now_ms = ctx.sys->Timebase().NowMs();

  switch (static_cast<message::MsgId>(pkt.header.id)) {
    case message::MsgId::kPong:
      break;
    case message::MsgId::kGpsData:
      ctx.sys->Mavlink().UpdateTelemetryCache(
          PayloadAs<message::GpsData>(pkt), now_ms);
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
    case message::MsgId::kVehicleStatus:
      ctx.sys->Mavlink().UpdateTelemetryCache(
          PayloadAs<message::VehicleStatusMsg>(pkt), now_ms);
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
