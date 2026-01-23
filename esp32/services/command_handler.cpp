#include "command_handler.hpp"
#include "ctx.hpp"
#include "fc_link.hpp"
#include "mavlink.hpp"
#include "system.hpp"

extern "C" {
#include "esp_log.h"
#include "freertos/FreeRTOS.h" // IWYU pragma: keep
#include "freertos/task.h"
}

static constexpr const char *kTag = "cmd";

// --- Handlers ---

static void OnPong(AppContext &ctx, const message::Packet &pkt) {
  (void)ctx;
  (void)pkt;
  ESP_LOGI(kTag, "CMD: Pong");
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

  initialized_ = true;
  ESP_LOGI(kTag, "Initialized");
}

bool CommandHandler::Dispatch(AppContext &ctx, const message::Packet &pkt) {
  if (!initialized_)
    return false;

  // Direct O(1) Lookup
  HandlerFunc handler = handlers_[pkt.header.id];
  if (handler) {
    handler(ctx, pkt);
    return (handler != OnUnknown);
  }
  return false;
}
