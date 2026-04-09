#include "battery.hpp"

#include "board.h"

void Battery::Init(const Config &cfg) {
  if (initialized_) {
    ErrorHandler();
  }

  cfg_ = cfg;
  RefreshData();
  initialized_ = true;
}

void Battery::Poll(uint32_t now_us) {
  (void)now_us;

  if (!initialized_) {
    ErrorHandler();
  }

  RefreshData();
}

void Battery::RefreshData() {
  data_.voltage = cfg_.voltage_v;
  data_.current = cfg_.current_a;
  data_.mah_drawn = cfg_.mah_drawn;
  data_.percentage = cfg_.percentage;
}
