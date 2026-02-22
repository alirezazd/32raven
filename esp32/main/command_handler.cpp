#include "command_handler.hpp"
#include "ctx.hpp"
#include "mavlink.hpp"
#include "state_machine.hpp"
#include "states.hpp"
#include <cstdio>

#include "message.hpp"
#include "tcp_server.hpp"

#include "panic.hpp"
#include "system.hpp"

extern "C" {
#include "esp_log.h"
#include "freertos/FreeRTOS.h" // IWYU pragma: keep
#include "freertos/task.h"
}

static constexpr const char *kTag = "cmd";
#include "esp_timer.h" // Added for esp_timer_get_time

static void OnLog(AppContext &ctx, const message::Packet &pkt) {
  (void)ctx;
  if (pkt.header.len == 0) return;
  
  // Check if binary format (starts with fmt_id byte < 32)
  if (pkt.payload[0] < 32 && pkt.header.len >= 2) {
    const auto *log = (const message::LogBinary *)pkt.payload;
    
    // Format string table
    static const char *fmt_table[] = {
      nullptr,  // ID 0 unused
      "Prof: Fast=%lu Link=%lu GPS=%lu GpsPub=%lu TSync=%lu Step=%lu Slow=%lu Send=%lu",  // ID 1
      "Sched: GpsB=%lu SeqGap=%lu DtMax=%lu Drop=%lu/s Phase=%lu Loss=%lu",                // ID 2
    };
    
    if (log->fmt_id > 0 && log->fmt_id < sizeof(fmt_table)/sizeof(fmt_table[0])) {
      const char *fmt = fmt_table[log->fmt_id];
      if (fmt && log->argc <= 16) {
        // Do the formatting on ESP32 (fast CPU)
        char buf[256];
        snprintf(buf, sizeof(buf), fmt, 
                 log->args[0], log->args[1], log->args[2], log->args[3],
                 log->args[4], log->args[5], log->args[6], log->args[7],
                 log->args[8], log->args[9], log->args[10], log->args[11],
                 log->args[12], log->args[13], log->args[14], log->args[15]);
        ESP_LOGI("FC", "%s", buf);
        return;
      }
    }
  }
  
  // Fallback: text format
  char buf[257];
  memcpy(buf, pkt.payload, pkt.header.len);
  buf[pkt.header.len] = '\0';
  ESP_LOGI("FC", "%s", buf);
}

static void OnRcChannels(AppContext &ctx, const message::Packet &pkt) {
  (void)ctx;
  if (pkt.header.len < 2 + 18 * 2) {
    ESP_LOGW(kTag, "Invalid RC Channels Length");
    return;
  }
}

static void OnGpsData(AppContext &ctx, const message::Packet &pkt) {
  if (pkt.header.len != sizeof(message::GpsData)) {
    ESP_LOGW(kTag, "Invalid GPS Data Length");
    return;
  }

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

  // Update Cache (Scheduling handled by Mavlink::Poll RMS)
  ctx.sys->Mavlink().OfferTelemetry(*t,
                                    (uint32_t)(esp_timer_get_time() / 1000));
}

static void OnTimeSync(AppContext &ctx, const message::Packet &pkt) {
  if (pkt.header.len != sizeof(message::TimeSyncMsg)) {
    ESP_LOGW(kTag, "Invalid TimeSync Length");
    return;
  }
  const auto *m = (const message::TimeSyncMsg *)pkt.payload;

  // Format for Monitor Mode (TCP)
  char buf[128];
  int len = std::snprintf(
      buf, sizeof(buf), "[TimeSync] ts=%lu drift=%ld us synced=%d\n",
      (unsigned long)m->timestamp, (long)m->drift_micros, (int)m->synced);

  if (len > 0) {
    ctx.sys->Tcp().SendData((uint8_t *)buf, (size_t)len);
  }
}

static void OnUnknown(AppContext &ctx, const message::Packet &pkt) {
  (void)ctx;
  ESP_LOGW(kTag, "Unknown Command ID: %d", pkt.header.id);
}

static void OnPong(AppContext &ctx, const message::Packet &pkt) {
  (void)ctx;
  (void)pkt;
  // Ping response - ignore
}

static void OnImuData(AppContext &ctx, const message::Packet &pkt) {
  if (pkt.header.len != sizeof(message::ImuData)) {
    ESP_LOGW(kTag, "Invalid IMU Data Length");
    return;
  }

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
  if (pkt.header.len != sizeof(message::PanicMsg)) {
    ESP_LOGW(kTag, "Invalid Panic Message Length");
    return;
  }

  const auto *m = (const message::PanicMsg *)pkt.payload;

  // In DFU mode, we ignore the panic to allow for flashing/debugging
  if (ctx.sm->CurrentState() == (const IState<AppContext> *)ctx.dfu_state) {
    static int64_t last_log_us = 0;
    int64_t now_us = esp_timer_get_time();
    if (now_us - last_log_us >= 2000000) {
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
  if (initialized_)
    return;
  cfg_ = cfg;

  // Initialize table
  for (int i = 0; i < 256; ++i) {
    handlers_[i] = OnUnknown;
  }

  handlers_[(uint8_t)message::MsgId::kRcChannels] = OnRcChannels;
  handlers_[(uint8_t)message::MsgId::kGpsData] = OnGpsData;
  handlers_[(uint8_t)message::MsgId::kLog] = OnLog;
  handlers_[(uint8_t)message::MsgId::kTimeSync] = OnTimeSync;
  handlers_[(uint8_t)message::MsgId::kPong] = OnPong;
  handlers_[(uint8_t)message::MsgId::kImuData] = OnImuData;
  handlers_[(uint8_t)message::MsgId::kPanic] = OnPanic;

  initialized_ = true;
  ESP_LOGI(kTag, "Initialized");
}

void CommandHandler::Dispatch(AppContext &ctx, const message::Packet &pkt) {
  if (!initialized_)
    return;

  HandlerFunc handler = handlers_[pkt.header.id];
  if (handler) {
    handler(ctx, pkt);
  } else {
    ESP_LOGW(kTag, "No handler for ID %d", pkt.header.id);
  }
}

CommandHandler::TransitionResult
CommandHandler::Dispatch(AppContext &ctx, const TcpServer::Event &ev) {
  if (!initialized_)
    return TransitionResult::kNone;

  switch (ev.id) {
  case TcpServer::EventId::kBegin: {
    ctx.sys->Tcp().StartDownload(ev.begin.size);

    ESP_LOGI(kTag, "TCP: BEGIN size=%u crc=%u target=%d",
             (unsigned)ev.begin.size, (unsigned)ev.begin.crc,
             (int)ev.begin.target);
    // Send handshake ACK
    ctx.sys->Tcp().SendCtrlLine("OK");
    ctx.sys->Programmer().SetTarget(ev.begin.target);
    return TransitionResult::kTransitionToProgram;
  }
  case TcpServer::EventId::kAbort: {
    ctx.sys->Tcp().StopDownload();
    ESP_LOGI(kTag, "TCP: ABORT");
    return TransitionResult::kTransitionToServing;
  }
  case TcpServer::EventId::kReset: {
    ctx.sys->Tcp().DisableBridge();
    ESP_LOGW(kTag, "TCP: RESET requested. Rebooting...");
    ctx.sys->Programmer().Boot();
    esp_restart();
    return TransitionResult::kNone;
  }
  case TcpServer::EventId::kBridge: {
    ESP_LOGI(kTag, "TCP: BRIDGE requested");
    ctx.sys->Tcp().EnableBridge();
    ctx.sys->Tcp().SendCtrlLine("OK");
    return TransitionResult::kNone;
  }
  default:
    break;
  }
  return TransitionResult::kNone;
}
