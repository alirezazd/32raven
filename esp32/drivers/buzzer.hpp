// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once
#include <cstdint>

extern "C" {
#include "hal/gpio_types.h"  // IWYU pragma: keep
#include "hal/ledc_types.h"
}
class Buzzer {
 public:
  static Buzzer &GetInstance() {
    static Buzzer instance;
    return instance;
  }
  struct Config {
    struct Output {
      gpio_num_t pin = GPIO_NUM_NC;
      bool active_low = false;
    };

    struct Ledc {
      ledc_mode_t speed_mode;
      ledc_timer_t timer;
      ledc_channel_t channel;
      ledc_timer_bit_t duty_resolution;
      // freq * 2^duty_resolution has to stay within the LEDC clock, so this
      // is the highest note the resolution above leaves room for.
      uint32_t max_note_hz;
    };

    Output output{};
    Ledc ledc{};
  };

  void Start(uint32_t freq_hz);
  void SetFrequency(uint32_t freq_hz);
  void SetDutyCycle(float duty_cycle);
  void Stop();

  bool IsRunning() const { return running_; }
  uint32_t GetFrequencyHz() const { return freq_hz_; }

 private:
  friend class System;
  void Init(const Config &cfg);
  uint32_t ComputeDutyTicks(float duty_cycle) const;
  uint32_t NoteHz(uint32_t freq_hz) const;
  Buzzer() = default;
  ~Buzzer() = default;
  Buzzer(const Buzzer &) = delete;
  Buzzer &operator=(const Buzzer &) = delete;

  Config cfg_{};
  bool running_ = false;
  uint32_t freq_hz_ = 0;
  uint32_t duty_ticks_ = 0;
};
