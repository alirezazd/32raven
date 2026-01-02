#pragma once
#include <stdint.h>

using time_ms_t = uint32_t;

// wraparound-safe: true if now is at/after deadline
static inline bool time_reached(time_ms_t now, time_ms_t deadline) {
  return (int32_t)(now - deadline) >= 0;
}

static inline time_ms_t time_after(time_ms_t now, time_ms_t delta_ms) {
  return (time_ms_t)(now + delta_ms);
}
