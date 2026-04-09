#pragma once

#include <cstdint>

#include "vehicle_state.hpp"

class Battery {
 public:
  struct Config {
    float voltage_v;
    float current_a;
    float mah_drawn;
    uint8_t percentage;
  };

  static Battery &GetInstance() {
    static Battery instance;
    return instance;
  }

  void Poll(uint32_t now_us);
  const BatteryData &GetData() const { return data_; }

 private:
  friend class System;
  void Init(const Config &cfg);

  Battery() = default;
  ~Battery() = default;
  Battery(const Battery &) = delete;
  Battery &operator=(const Battery &) = delete;

  void RefreshData();

  Config cfg_{};
  BatteryData data_{};
  bool initialized_ = false;
};
