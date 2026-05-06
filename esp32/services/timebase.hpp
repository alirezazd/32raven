#pragma once
#include <cstdint>

// Timebase service is just a thin wrapper around esp_timer, but it allows us to
// keep the code uniform.

using TimeMs = uint32_t;
using TimeUs = uint64_t;

class Timebase {
 public:
  static Timebase &GetInstance() {
    static Timebase instance;
    return instance;
  }

  struct Config {};

  TimeMs NowMs() const;
  TimeUs NowUs() const;
  void SleepMs(TimeMs ms) const;

  static bool Reached(TimeMs now, TimeMs deadline) {
    return (int32_t)(now - deadline) >= 0;
  }

  static TimeMs After(TimeMs now, TimeMs delta_ms) {
    return (TimeMs)(now + delta_ms);
  }

 private:
  friend class System;
  Timebase() = default;
  ~Timebase() = default;
  Timebase(const Timebase &) = delete;
  Timebase &operator=(const Timebase &) = delete;
};

// wraparound-safe: true if now is at/after deadline
static inline bool TimeReached(TimeMs now, TimeMs deadline) {
  return Timebase::Reached(now, deadline);
}

static inline TimeMs TimeAfter(TimeMs now, TimeMs delta_ms) {
  return Timebase::After(now, delta_ms);
}
