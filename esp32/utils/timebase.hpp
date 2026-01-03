#pragma once
#include <stdint.h>

using TimeMs = uint32_t;

TimeMs NowMs();

// wraparound-safe: true if now is at/after deadline
static inline bool TimeReached(TimeMs now, TimeMs deadline) {
  return (int32_t)(now - deadline) >= 0;
}

static inline TimeMs TimeAfter(TimeMs now, TimeMs delta_ms) {
  return (TimeMs)(now + delta_ms);
}
