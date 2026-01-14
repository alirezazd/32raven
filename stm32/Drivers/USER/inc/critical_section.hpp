#pragma once

#include "stm32f4xx.h"
#include <cstdint>

static constexpr uint32_t kMaskPri =
    (5u << (8u - __NVIC_PRIO_BITS)) & 0xFFu; // Priority 5 masked

struct BasepriGuard {
  uint32_t old;
  explicit BasepriGuard(uint32_t new_basepri) : old(__get_BASEPRI()) {
    __set_BASEPRI(new_basepri);
    __DSB();
    __ISB();
  }
  ~BasepriGuard() {
    __set_BASEPRI(old);
    __DSB();
    __ISB();
  }
};
