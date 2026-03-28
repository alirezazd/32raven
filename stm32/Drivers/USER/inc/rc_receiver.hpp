#pragma once

#include "ee.hpp"
#include "ee_schema.hpp"
#include "message.hpp"
#include "stm32_limits.hpp"
#include "vehicle_state.hpp"

#include <array>
#include <cstdint>

class FcLink;
class VehicleState;

class RcReceiver {
public:
  struct Config {
    bool enabled_channels[16]{};
  };

  struct RawData {
    std::array<uint16_t, stm32_limits::kRcEnabledChannelCount> channels{};
    uint32_t timestamp_us = 0;
  };

  static RcReceiver &GetInstance() {
    static RcReceiver instance;
    return instance;
  }

  void ProcessRawState(const message::RcChannelsMsg &msg, uint32_t now_us);
  void Poll(uint32_t now_us);

  const RawData &GetRawData() const { return raw_; }
  const ee_schema::RcCalibration &GetCalibration() const {
    return calibration_;
  }
  ee_schema::RcCalibration &GetCalibration() { return calibration_; }

  bool SaveCalibration();

private:
  friend class System;

  void Init(const Config &cfg, EE &ee, VehicleState &vehicle_state,
            FcLink &fc_link);
  uint16_t ApplyCalibration(uint16_t raw_us, uint16_t min_us,
                            uint16_t max_us) const;
  void RefreshLinkState(uint32_t now_us);
  bool PublishIfChanged(const RcData &previous);
  void LogState(uint32_t now_us);

  RcReceiver() = default;
  ~RcReceiver() = default;
  RcReceiver(const RcReceiver &) = delete;
  RcReceiver &operator=(const RcReceiver &) = delete;

  EE *ee_ = nullptr;
  VehicleState *vehicle_state_ = nullptr;
  FcLink *fc_link_ = nullptr;
  bool initialized_ = false;
  uint32_t last_log_us_ = 0;
  uint8_t rssi_ = 0;
  uint32_t radio_status_update_us_ = 0;
  uint32_t tx_candidate_since_us_ = 0;
  bool tx_candidate_online_ = false;
  RawData raw_{};
  RcData current_{};
  ee_schema::RcCalibration calibration_{};
};
