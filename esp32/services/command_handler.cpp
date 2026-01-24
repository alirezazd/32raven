#include "command_handler.hpp"
#include "ctx.hpp"
#include "fc_link.hpp"
#include "mavlink.hpp"
#include "state_machine.hpp"
#include <cstdio>

#include "system.hpp"

extern "C" {
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
}

static constexpr const char *kTag = "cmd";

static void OnPong(AppContext &ctx, const message::Packet &pkt) {
  (void)ctx;
  (void)pkt;
  ESP_LOGI(kTag, "CMD: Pong");
}

static void OnLog(AppContext &ctx, const message::Packet &pkt) {
  (void)ctx;
  if (pkt.header.len > 0) {
    char buf[257];
    memcpy(buf, pkt.payload, pkt.header.len);
    buf[pkt.header.len] = '\0';
    ESP_LOGI("FC", "%s", buf);
  }
}

static void OnPing(AppContext &ctx, const message::Packet &pkt) {
  (void)pkt;
  ESP_LOGI(kTag, "CMD: Ping -> Sending Pong");
  message::Packet tx_pkt;
  tx_pkt.header.id = (uint8_t)message::MsgId::kPong;
  tx_pkt.header.len = 0;
  ctx.sys->FcLink().SendPacket(tx_pkt);
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
  auto &mav = ctx.sys->Mavlink();

  // Rate Limit (1Hz) logic
  static uint32_t last_sent = 0;
  uint32_t now =
      xTaskGetTickCount() * portTICK_PERIOD_MS; // Correct way to get ms

  if (now - last_sent >= 1000) {
    last_sent = now;

    if ((t->valid & 0x03) == 0x03) {
      mav.SendSystemTime(*t);
    }
    mav.SendGpsRawInt(*t);
    mav.SendGlobalPositionInt(*t);
  }
}

static void OnUnknown(AppContext &ctx, const message::Packet &pkt) {
  (void)ctx;
  ESP_LOGW(kTag, "Unknown Command ID: %d", pkt.header.id);
}

void CommandHandler::Init(const Config &cfg) {
  if (initialized_)
    return;
  cfg_ = cfg;

  // Initialize table
  for (int i = 0; i < 256; ++i) {
    handlers_[i] = OnUnknown;
  }
  handlers_[(uint8_t)message::MsgId::kPing] = OnPing;
  handlers_[(uint8_t)message::MsgId::kPong] = OnPong;
  handlers_[(uint8_t)message::MsgId::kRcChannels] = OnRcChannels;
  handlers_[(uint8_t)message::MsgId::kGpsData] = OnGpsData;
  handlers_[(uint8_t)message::MsgId::kLog] = OnLog;

  initialized_ = true;
  ESP_LOGI(kTag, "Initialized");
}

bool CommandHandler::Dispatch(AppContext &ctx, const message::Packet &pkt) {
  if (!initialized_)
    return false;

  HandlerFunc handler = handlers_[pkt.header.id];
  if (handler) {
    handler(ctx, pkt);
    return (handler != OnUnknown);
  }
  return false;
}

CommandHandler::Result CommandHandler::Dispatch(AppContext &ctx,
                                                const TcpServer::Event &ev) {
  if (!initialized_)
    return Result::kNone;

  switch (ev.id) {
  case TcpServer::EventId::kBegin: {
    ctx.sys->Tcp().StartDownload(ev.begin.size);

    ESP_LOGI(kTag, "TCP: BEGIN size=%u crc=%u target=%d",
             (unsigned)ev.begin.size, (unsigned)ev.begin.crc,
             (int)ev.begin.target);
    // Send handshake ACK
    ctx.sys->Tcp().SendCtrlLine("OK");
    ctx.sys->Programmer().SetTarget(ev.begin.target);
    return Result::kTransitionToProgram;
  }
  case TcpServer::EventId::kAbort: {
    ctx.sys->Tcp().StopDownload();
    ESP_LOGI(kTag, "TCP: ABORT");
    return Result::kTransitionToServing;
  }
  case TcpServer::EventId::kReset: {
    ctx.sys->Tcp().DisableBridge();
    ESP_LOGW(kTag, "TCP: RESET requested. Rebooting...");
    ctx.sys->Programmer().Boot();
    esp_restart();
    return Result::kNone;
  }
  default:
    break;
  }
  return Result::kNone;
}

void CommandHandler::Dispatch(AppContext &ctx, const mavlink_message_t &msg) {
  if (!initialized_ || !ctx.sm)
    return;

  const char *state_name = ctx.sm->CurrentName();
  if (!state_name)
    return;

  if (std::strcmp(state_name, "Dfu") == 0) {
    char buf[512];
    int len = 0;

    switch (msg.msgid) {
    case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE: {
      mavlink_rc_channels_override_t rc;
      mavlink_msg_rc_channels_override_decode(&msg, &rc);
      len = std::snprintf(
          buf, sizeof(buf),
          "[MAVLink] RC_OVERRIDE: Ch1=%d Ch2=%d Ch3=%d Ch4=%d Ch5=%d Ch6=%d\n",
          rc.chan1_raw, rc.chan2_raw, rc.chan3_raw, rc.chan4_raw, rc.chan5_raw,
          rc.chan6_raw);
      break;
    }
    // ... Add other messages if needed, or remove Bridge check above to unify
    default:
      break;
    }

    if (len > 0) {
      ctx.sys->Tcp().SendData((uint8_t *)buf, (size_t)len);
    }
  }
}
