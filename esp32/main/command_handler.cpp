#include "command_handler.hpp"

#include <cstdio>

#include "ctx.hpp"
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

[[noreturn]] static void PanicUnknownCommand(uint8_t id) {
  ESP_LOGE(kTag, "Unknown Command ID: 0x%02X (%u)", (unsigned)id, (unsigned)id);
  Panic(ErrorCode::kUnknownCommand);
}

static void OnLog(AppContext &ctx, const message::Packet &pkt) {
  (void)ctx;
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

static void OnGpsData(AppContext &ctx, const message::Packet &pkt) {
  const auto *t = (const message::GpsData *)pkt.payload;

  // Format for Monitor Mode (TCP) - Debugging
  char buf[128];
  int len = std::snprintf(
      buf, sizeof(buf),
      "GPS: fix=%d sats=%d lat=%ld lon=%ld alt=%ld "
      "vel=%u hdg=%u hac=%lu vac=%lu "
      "t=%02d:%02d:%02d\n",
      (int)t->fixType, (int)t->numSV, (long)t->lat, (long)t->lon, (long)t->hMSL,
      (unsigned)t->vel, (unsigned)t->hdg, (unsigned long)t->hAcc,
      (unsigned long)t->vAcc, (int)t->hour, (int)t->min, (int)t->sec);

  if (len > 0) {
    ctx.sys->Tcp().SendData((uint8_t *)buf, (size_t)len);
  }
}

static void OnUnknown(AppContext &ctx, const message::Packet &pkt) {
  (void)ctx;
  PanicUnknownCommand(pkt.header.id);
}

static void OnPong(AppContext &ctx, const message::Packet &pkt) {
  (void)ctx;
  (void)pkt;
}

static void OnRcMapConfig(AppContext &ctx, const message::Packet &pkt) {
  const auto *cfg = (const message::RcMapConfigMsg *)pkt.payload;
  (void)ctx;
  (void)cfg;
}

static void OnRcCalibrationConfig(AppContext &ctx, const message::Packet &pkt) {
  const auto *cfg = (const message::RcCalibrationConfigMsg *)pkt.payload;
  (void)ctx;
  (void)cfg;
}

static void OnRcChannels(AppContext &ctx, const message::Packet &pkt) {
  const auto *msg = (const message::RcChannelsMsg *)pkt.payload;
  (void)ctx;
  (void)msg;
}

static void OnGyroCalibrationIdConfig(AppContext &ctx,
                                      const message::Packet &pkt) {
  const auto *cfg = (const message::GyroCalibrationIdConfigMsg *)pkt.payload;
  (void)ctx;
  (void)cfg;
}

static void OnImuData(AppContext &ctx, const message::Packet &pkt) {
  const auto *m = (const message::ImuData *)pkt.payload;

  char buf[128];
  int len = std::snprintf(
      buf, sizeof(buf),
      "IMU: ax=%.3f ay=%.3f az=%.3f gx=%.3f gy=%.3f gz=%.3f\n", m->accel[0],
      m->accel[1], m->accel[2], m->gyro[0], m->gyro[1], m->gyro[2]);

  if (len > 0) {
    ctx.sys->Tcp().SendData((uint8_t *)buf, (size_t)len);
  }
}

static void OnPanic(AppContext &ctx, const message::Packet &pkt) {
  const auto *m = (const message::PanicMsg *)pkt.payload;

  // In DFU mode, we ignore the panic to allow for flashing/debugging
  if (ctx.sm->CurrentState() == (const IState<AppContext> *)ctx.dfu_state) {
    static int64_t last_log_us = 0;
    int64_t now_us = esp_timer_get_time();
    if (now_us - last_log_us >= 5000000) {
      ESP_LOGE(kTag, "STM32 Panic (Ignored in DFU): Code 0x%X: %s",
               m->error_code,
               GetMessage(static_cast<ErrorCode>(m->error_code)));
      last_log_us = now_us;
    }
    return;
  }

  // Enter panic state with error code - this will never return
  Panic(static_cast<ErrorCode>(m->error_code));
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
      OnPong(ctx, pkt);
      break;
    case message::MsgId::kGpsData:
      ctx.sys->Mavlink().UpdateGpsCache(
          *reinterpret_cast<const message::GpsData *>(pkt.payload), now_ms);
      OnGpsData(ctx, pkt);
      break;
    case message::MsgId::kLog:
      OnLog(ctx, pkt);
      break;
    case message::MsgId::kRcMapConfig:
      ctx.sys->Mavlink().UpdateRcMapConfigCache(
          *reinterpret_cast<const message::RcMapConfigMsg *>(pkt.payload),
          now_ms);
      OnRcMapConfig(ctx, pkt);
      break;
    case message::MsgId::kRcCalibrationConfig:
      ctx.sys->Mavlink().UpdateRcCalibrationConfigCache(
          *reinterpret_cast<const message::RcCalibrationConfigMsg *>(pkt.payload),
          now_ms);
      OnRcCalibrationConfig(ctx, pkt);
      break;
    case message::MsgId::kRcChannels:
      ctx.sys->Mavlink().UpdateRcChannelsCache(
          *reinterpret_cast<const message::RcChannelsMsg *>(pkt.payload),
          now_ms);
      OnRcChannels(ctx, pkt);
      break;
    case message::MsgId::kGyroCalibrationIdConfig:
      ctx.sys->Mavlink().UpdateGyroCalibrationIdConfigCache(
          *reinterpret_cast<const message::GyroCalibrationIdConfigMsg *>(
              pkt.payload),
          now_ms);
      OnGyroCalibrationIdConfig(ctx, pkt);
      break;
    case message::MsgId::kImuData:
      OnImuData(ctx, pkt);
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
