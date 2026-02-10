#include "command_handler.hpp"
#include "ctx.hpp"
#include "mavlink.hpp"
#include "state_machine.hpp"
#include <cstdio>

#include "message.hpp"
#include "tcp_server.hpp"
#include "user_config.hpp" // Added for scheduler ratestdio>

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
  if (pkt.header.len > 0) {
    char buf[257];
    memcpy(buf, pkt.payload, pkt.header.len);
    buf[pkt.header.len] = '\0';
    ESP_LOGI("FC", "%s", buf);
  }
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
