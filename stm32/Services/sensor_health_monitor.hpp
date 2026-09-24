// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include <array>
#include <cstdint>
#include <utility>

#include "shared_state.hpp"

// Judges each sensor for the ground station: present once it has reported,
// healthy while its reports are fresh and its path has not faulted lately.
class SensorHealthMonitor {
 public:
  static SensorHealthMonitor &GetInstance();

  // Reads SystemHealth, so it runs after that is published.
  void Poll(uint32_t now_us);

 private:
  friend class System;
  void Init(SharedState &blackboard);

  SensorHealthMonitor() = default;
  ~SensorHealthMonitor() = default;
  SensorHealthMonitor(const SensorHealthMonitor &) = delete;
  SensorHealthMonitor &operator=(const SensorHealthMonitor &) = delete;

  // Every fault counter is a since-boot total, so a window is what makes the
  // bit mean an error now rather than an error ever.
  struct FaultWindow {
    uint32_t last_total = 0;
    bool healthy = true;
  };

  enum class FaultSource : uint8_t {
    kImu,
    kGps,
    kRc,
    kBattery,
    kCount,
  };

  void UpdateFaultWindows(uint32_t now_us);
  bool IsHealthy(FaultSource source) const {
    return fault_windows_[std::to_underlying(source)].healthy;
  }

  SharedState *blackboard_ = nullptr;
  uint32_t last_window_us_ = 0;
  std::array<FaultWindow, std::to_underlying(FaultSource::kCount)>
      fault_windows_{};
};
