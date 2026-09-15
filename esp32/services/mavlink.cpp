// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "mavlink.hpp"

#include <cstdio>

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

void Mavlink::Init(const MavlinkConfig &cfg, IMavlinkTransport *transport,
                   FcLink &fc_link) {
  if (transport == nullptr) {
    Panic(ErrorCode::Esp32::kMavlinkInitFailed);
  }
  if (cfg.sysid == 0 || cfg.tx.periods.hb_ms == 0 ||
      cfg.tx.schedule.hb_deadline_ms == 0) {
    Panic(ErrorCode::Esp32::kMavlinkInitFailed);
  }

  cfg_ = cfg;

  // Non-null from here on, which is why the service does not re-check it on
  // every tick.
  transport_ = transport;
  fc_link_ = &fc_link;
  fc_config_.Init(fc_link);
  params_.Init(cfg_, fc_config_);
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
  ServiceRx();
  fc_config_.Poll(now_ms);
  if (const char *record = fc_config_.TakeStalled()) {
    char text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN + 1];
    std::snprintf(text, sizeof(text), "FC not answering: %s", record);
    NotifyGcsIssueOnce(text, MAV_SEVERITY_ERROR);
  }
  ServiceTx(now_ms);
}

void Mavlink::SetTelemetryLink(bool enabled) {
  tx_frame_.Clear();
  tx_work_queue_.Clear();

  if (enabled) {
    const uint32_t now_ms = Sys().Timebase().NowMs();
    InitTxSchedule(now_ms, true);
    next_tx_poll_ms_ = now_ms;
    link_up_ms_ = now_ms;
    link_enabled_ = true;
    return;
  }

  link_enabled_ = false;
  params_.Reset();
  InitTxSchedule(0);
  fc_config_.AbandonWrites();
  // Reached from panic recovery too, which may run before Init.
  if (transport_ != nullptr) {
    transport_->ClearPeer();
  }
}

uint32_t Mavlink::GetRxPacketCount() const {
  return rx_packet_count_.load(std::memory_order_relaxed);
}

uint32_t Mavlink::GetTxPacketCount() const {
  return tx_packet_count_.load(std::memory_order_relaxed);
}

uint32_t Mavlink::GetRxHeartbeatCount() const {
  return rx_heartbeat_count_.load(std::memory_order_relaxed);
}

uint32_t Mavlink::GetTxHeartbeatCount() const {
  return tx_heartbeat_count_.load(std::memory_order_relaxed);
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
