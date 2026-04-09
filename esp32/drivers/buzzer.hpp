#pragma once
#include <cstdint>

#include "error_code.hpp"
extern "C" {
#include "driver/ledc.h"
#include "hal/gpio_types.h"  // IWYU pragma: keep
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

    struct Pwm {
      ledc_mode_t speed_mode = LEDC_LOW_SPEED_MODE;
      ledc_timer_t timer_num = LEDC_TIMER_1;
      ledc_channel_t channel = LEDC_CHANNEL_1;
      ledc_timer_bit_t duty_resolution = LEDC_TIMER_10_BIT;
    };

    struct Startup {
      uint32_t freq_hz = 2000;
      float duty_cycle = 0.5f;
    };

    Output output{};
    Pwm pwm{};
    Startup startup{};
  };

  ErrorCode Start(uint32_t freq_hz);
  ErrorCode SetFrequency(uint32_t freq_hz);
  ErrorCode SetDutyCycle(float duty_cycle);
  ErrorCode Stop();

  bool IsRunning() const { return running_; }
  uint32_t GetFrequencyHz() const { return freq_hz_; }

 private:
  friend class System;
  void Init(const Config &cfg);
  static uint32_t ComputeDutyTicks(ledc_timer_bit_t resolution,
                                   float duty_cycle);
  Buzzer() = default;
  ~Buzzer() = default;
  Buzzer(const Buzzer &) = delete;
  Buzzer &operator=(const Buzzer &) = delete;

  Config cfg_{};
  bool running_ = false;
  uint32_t freq_hz_ = 0;
  uint32_t duty_ticks_ = 0;
};
