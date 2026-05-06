#pragma once

#include <array>
#include <cstdint>

#include "ee.hpp"
#include "ee_schema.hpp"
#include "message.hpp"
#include "stm32_limits.hpp"
#include "vehicle_state.hpp"

class VehicleState;

class RcReceiver {
 public:
  struct Config {
    bool enabled_channels[16]{};
    uint8_t roll_channel = 0;
    uint8_t pitch_channel = 0;
    uint8_t yaw_channel = 0;
    uint8_t throttle_channel = 0;
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
  const RcData &GetCurrentData() const { return current_; }
  const ee_schema::RcCalibration &GetCalibration() const {
    return calibration_;
  }
  ee_schema::RcCalibration &GetCalibration() { return calibration_; }
  message::RcMapConfigMsg GetRcMapConfig() const {
    return {
        .roll = cfg_.roll_channel,
        .pitch = cfg_.pitch_channel,
        .yaw = cfg_.yaw_channel,
        .throttle = cfg_.throttle_channel,
    };
  }
  bool SetRcMapConfig(const message::RcMapConfigMsg &cfg);
  bool SetCalibrationConfig(const message::RcCalibrationConfigMsg &cfg);

  bool SaveCalibration();

 private:
  friend class System;

  void Init(const Config &cfg, EE &ee, VehicleState &vehicle_state);
  uint16_t ApplyCalibration(uint16_t raw_us, uint16_t min_us, uint16_t trim_us,
                            uint16_t max_us, int8_t rev) const;
  bool IsConfigValid(const Config &cfg) const;
  void RecomputeCurrentFromRaw();
  void RefreshLinkState(uint32_t now_us);
  bool PublishIfChanged(const RcData &previous);
  bool SaveRcMap(const Config &cfg);
  void LogState(uint32_t now_us);

  RcReceiver() = default;
  ~RcReceiver() = default;
  RcReceiver(const RcReceiver &) = delete;
  RcReceiver &operator=(const RcReceiver &) = delete;

  EE *ee_ = nullptr;
  VehicleState *vehicle_state_ = nullptr;
  bool initialized_ = false;
  uint32_t last_log_us_ = 0;
  uint8_t rssi_ = 0;
  uint32_t radio_status_update_us_ = 0;
  uint32_t tx_candidate_since_us_ = 0;
  bool tx_candidate_online_ = false;
  RawData raw_{};
  RcData current_{};
  ee_schema::RcCalibration calibration_{};
  Config cfg_{};
};
