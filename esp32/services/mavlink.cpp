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
  DisableTelemetryLink();
  ESP_LOGI(kTag, "Initialized (MAVLink UDP transport)");
}

void Mavlink::Poll(uint32_t now_ms) {
  ServiceUdpRx();
  ServicePendingParamApplies(now_ms);
  ServiceTx(now_ms);
}

void Mavlink::UpdateGpsCache(const message::GpsData &gps, uint32_t now_ms) {
  latest_gps_.value = gps;
  latest_gps_.have_data = true;
  latest_gps_.update_ms = now_ms;
  latest_gps_.generation = (latest_gps_.generation == UINT32_MAX)
                               ? 1u
                               : (latest_gps_.generation + 1u);
}

void Mavlink::UpdateRcChannelsCache(const message::RcChannelsMsg &channels,
                                    uint32_t now_ms) {
  latest_rc_channels_.value = channels;
  latest_rc_channels_.have_data = true;
  latest_rc_channels_.update_ms = now_ms;
  latest_rc_channels_.generation =
      (latest_rc_channels_.generation == UINT32_MAX)
          ? 1u
          : (latest_rc_channels_.generation + 1u);
}

void Mavlink::EnableTelemetryLink() {
  const uint32_t now_ms = Sys().Timebase().NowMs();
  tx_frame_.len = 0;
  tx_frame_.is_hb = false;
  tx_work_queue_.Clear();
  InitTxSchedule(cfg_.tx, now_ms, true);
  next_tx_poll_ms_ = now_ms;
  link_enabled_ = true;
}

void Mavlink::DisableTelemetryLink() {
  link_enabled_ = false;
  udp_tx_.pending_param_queue_.Clear();
  udp_tx_.param_stream_active = false;
  udp_tx_.next_param_index = 0;
  tx_frame_.len = 0;
  tx_frame_.is_hb = false;
  tx_work_queue_.Clear();
  InitTxSchedule(cfg_.tx, 0);
  rc_map_apply_ = {};
  rc_calibration_apply_ = {};
  udp_->ClearPeer();
}

std::optional<message::GpsData> Mavlink::GetLatestGpsData() const {
  std::optional<message::GpsData> gps;
  if (latest_gps_.have_data) {
    gps = latest_gps_.value;
  }
  return gps;
}

std::optional<Mavlink::RcChannelsSample> Mavlink::GetLatestRcChannelsData()
    const {
  if (!latest_rc_channels_.have_data) {
    return std::nullopt;
  }

  RcChannelsSample sample{};
  sample.msg = latest_rc_channels_.value;
  sample.update_ms = latest_rc_channels_.update_ms;

  return sample;
}

uint32_t Mavlink::UdpRxPacketCount() const {
  return udp_rx_packet_count_.load(std::memory_order_relaxed);
}

uint32_t Mavlink::UdpTxPacketCount() const {
  return udp_tx_packet_count_.load(std::memory_order_relaxed);
}
