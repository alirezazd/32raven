// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "mavlink.hpp"

#include "error_code.hpp"
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

void Mavlink::Init(const Config &cfg, IMavlinkTransport *transport,
                   FcLink &fc_link) {
  if (transport == nullptr) {
    Panic(ErrorCode::Esp32::kMavlinkInitFailed);
  }
  if (cfg.identity.sysid == 0 || cfg.tx.periods.hb_ms == 0 ||
      cfg.tx.schedule.hb_deadline_ms == 0) {
    Panic(ErrorCode::Esp32::kMavlinkInitFailed);
  }

  cfg_ = cfg;

  // Non-null from here on, which is why the service does not re-check it on
  // every tick.
  transport_ = transport;
  fc_link_ = &fc_link;
  ResetParamState();
  SetTelemetryLink(false);
  ESP_LOGI(kTag, "Initialized (MAVLink transport service)");
}

void Mavlink::SetTransport(IMavlinkTransport *transport) {
  if (transport == nullptr) {
    Panic(ErrorCode::Esp32::kMavlinkInitFailed);
  }
  if (transport_ != nullptr && transport_ != transport) {
    transport_->ClearPeer();
  }
  tx_frame_.Clear();
  transport_ = transport;
}

void Mavlink::Poll(uint32_t now_ms) {
  ServiceUdpRx();
  ServicePendingParamApplies(now_ms);
  ServiceTx(now_ms);
}

void Mavlink::SetTelemetryLink(bool enabled) {
  tx_frame_.Clear();
  tx_work_queue_.Clear();

  if (enabled) {
    const uint32_t now_ms = Sys().Timebase().NowMs();
    InitTxSchedule(now_ms, true);
    next_tx_poll_ms_ = now_ms;
    link_enabled_ = true;
    return;
  }

  link_enabled_ = false;
  udp_tx_.pending_param_queue_.Clear();
  udp_tx_.param_stream_ = TxState::ParamStreamIdle{};
  InitTxSchedule(0);
  rc_map_apply_.Reset();
  rc_calibration_apply_.Reset();
  transport_->ClearPeer();
}

uint32_t Mavlink::GetUdpRxPacketCount() const {
  return udp_rx_packet_count_.load(std::memory_order_relaxed);
}

uint32_t Mavlink::GetUdpTxPacketCount() const {
  return udp_tx_packet_count_.load(std::memory_order_relaxed);
}

uint32_t Mavlink::GetUdpRxHeartbeatCount() const {
  return udp_rx_heartbeat_count_.load(std::memory_order_relaxed);
}

uint32_t Mavlink::GetUdpTxHeartbeatCount() const {
  return udp_tx_heartbeat_count_.load(std::memory_order_relaxed);
}

std::optional<bool> Mavlink::PeerArmed(uint32_t now_ms) const {
  if (!vehicle_status_.have_data ||
      (now_ms - vehicle_status_.update_ms) > peer_timeout_ms_) {
    return std::nullopt;  // Peer is unresponsive or unknown
  }
  return static_cast<message::ArmedState>(vehicle_status_.value.armed_state) ==
         message::ArmedState::kArmed;
}

std::optional<Mavlink::LatestRcChannelsData> Mavlink::GetLatestRcChannelsData()
    const {
  if (!rc_channels_.have_data) {
    return std::nullopt;
  }
  return LatestRcChannelsData{rc_channels_.value, rc_channels_.update_ms};
}
