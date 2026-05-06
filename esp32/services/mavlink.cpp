#include "mavlink.hpp"

#include "esp_log.h"
#include "panic.hpp"
#include "system.hpp"

static constexpr const char *kTag = "mavlink";

Mavlink::Mavlink() {}
Mavlink::~Mavlink() {}

Mavlink &Mavlink::GetInstance() {
  static Mavlink instance;
  return instance;
}

// Lifecycle and top-level poll entry points.
void Mavlink::Init(const Config &cfg, UdpServer *udp) {
  if (udp == nullptr) {
    Panic(ErrorCode::kMavlinkInitFailed);
  }
  if (cfg.identity.sysid == 0 || cfg.tx.periods.hb_ms == 0 ||
      cfg.tx.schedule.hb_deadline_ms == 0) {
    Panic(ErrorCode::kMavlinkInitFailed);
  }

  cfg_ = cfg;

  // `udp_` is a required dependency after initialization; service code assumes
  // this assignment succeeded and does not re-check for null on every tick.
  udp_ = udp;
  ResetParamState();
  SetTelemetryLink(false);
  ESP_LOGI(kTag, "Initialized (MAVLink UDP transport)");
}

void Mavlink::Poll(uint32_t now_ms) {
  ServiceUdpRx();
  ServicePendingParamApplies(now_ms);
  ServiceTx(now_ms);
}

void Mavlink::SetTelemetryLink(bool enabled) {
  tx_frame_.len = 0;
  tx_frame_.is_hb = false;
  tx_work_queue_.Clear();

  if (enabled) {
    const uint32_t now_ms = Sys().Timebase().NowMs();
    InitTxSchedule(cfg_.tx, now_ms, true);
    next_tx_poll_ms_ = now_ms;
    link_enabled_ = true;
    return;
  }

  link_enabled_ = false;
  udp_tx_.pending_param_queue_.Clear();
  udp_tx_.param_stream_ = TxState::ParamStreamIdle{};
  InitTxSchedule(cfg_.tx, 0);
  rc_map_apply_.Reset();
  rc_calibration_apply_.Reset();
  udp_->ClearPeer();
}

uint32_t Mavlink::GetUdpRxPacketCount() const {
  return udp_rx_packet_count_.load(std::memory_order_relaxed);
}

uint32_t Mavlink::GetUdpTxPacketCount() const {
  return udp_tx_packet_count_.load(std::memory_order_relaxed);
}

std::optional<Mavlink::LatestRcChannelsData>
Mavlink::GetLatestRcChannelsData() const {
  if (!rc_channels_.have_data) {
    return std::nullopt;
  }
  return LatestRcChannelsData{rc_channels_.value, rc_channels_.update_ms};
}
