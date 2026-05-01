#include "buzzer.hpp"

#include <algorithm>
#include <cmath>

#include "panic.hpp"

extern "C" {
#include "esp_log.h"
}

namespace {
constexpr float kMinDuty = 0.0f;
constexpr float kMaxDuty = 1.0f;
constexpr const char *kTag = "buzzer";
}  // namespace

uint32_t Buzzer::ComputeDutyTicks(ledc_timer_bit_t resolution,
                                  float duty_cycle) {
  duty_cycle = std::clamp(duty_cycle, kMinDuty, kMaxDuty);

  const uint32_t max_ticks = (1UL << static_cast<uint32_t>(resolution)) - 1UL;
  const float ticks_f = duty_cycle * static_cast<float>(max_ticks);
  return static_cast<uint32_t>(std::lround(ticks_f));
}

void Buzzer::Init(const Config &cfg) {
  if (cfg.output.pin == GPIO_NUM_NC || cfg.startup.freq_hz == 0) {
    Panic(ErrorCode::kBuzzerInvalidConfig);
  }

  cfg_ = cfg;
  freq_hz_ = cfg.startup.freq_hz;
  duty_ticks_ =
      ComputeDutyTicks(cfg.pwm.duty_resolution, cfg.startup.duty_cycle);

  ledc_timer_config_t timer_cfg = {};
  timer_cfg.speed_mode = cfg.pwm.speed_mode;
  timer_cfg.duty_resolution = cfg.pwm.duty_resolution;
  timer_cfg.timer_num = cfg.pwm.timer_num;
  timer_cfg.freq_hz = cfg.startup.freq_hz;
  timer_cfg.clk_cfg = LEDC_AUTO_CLK;
  timer_cfg.deconfigure = false;

  if (ledc_timer_config(&timer_cfg) != ESP_OK) {
    Panic(ErrorCode::kBuzzerTimerInitFailed);
  }

  ledc_channel_config_t channel_cfg = {};
  channel_cfg.gpio_num = cfg.output.pin;
  channel_cfg.speed_mode = cfg.pwm.speed_mode;
  channel_cfg.channel = cfg.pwm.channel;
  channel_cfg.intr_type = LEDC_INTR_DISABLE;
  channel_cfg.timer_sel = cfg.pwm.timer_num;
  channel_cfg.duty = 0;
  channel_cfg.hpoint = 0;
  channel_cfg.sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD;
  channel_cfg.flags.output_invert =
      static_cast<unsigned int>(cfg.output.active_low ? 1U : 0U);

  if (ledc_channel_config(&channel_cfg) != ESP_OK) {
    Panic(ErrorCode::kBuzzerChannelInitFailed);
  }
  running_ = false;

  ESP_LOGI(
      kTag,
      "init pin=%d active_low=%d timer=%d channel=%d freq=%lu duty_ticks=%lu",
      static_cast<int>(cfg.output.pin), static_cast<int>(cfg.output.active_low),
      static_cast<int>(cfg.pwm.timer_num), static_cast<int>(cfg.pwm.channel),
      static_cast<unsigned long>(freq_hz_),
      static_cast<unsigned long>(duty_ticks_));
}

ErrorCode Buzzer::Start(uint32_t freq_hz) {
  if (freq_hz == 0) {
    return ErrorCode::kBuzzerInvalidArg;
  }

  if (ledc_set_freq(cfg_.pwm.speed_mode, cfg_.pwm.timer_num, freq_hz) ==
      ESP_OK) {
    freq_hz_ = freq_hz;
  } else {
    return ErrorCode::kBuzzerSetFreqFailed;
  }

  if (ledc_set_duty_and_update(cfg_.pwm.speed_mode, cfg_.pwm.channel,
                               duty_ticks_, 0) != ESP_OK) {
    return ErrorCode::kBuzzerSetDutyFailed;
  }

  running_ = true;
  return ErrorCode::kOk;
}

ErrorCode Buzzer::SetFrequency(uint32_t freq_hz) {
  if (freq_hz == 0) {
    return ErrorCode::kBuzzerInvalidArg;
  }

  if (ledc_set_freq(cfg_.pwm.speed_mode, cfg_.pwm.timer_num, freq_hz) !=
      ESP_OK) {
    return ErrorCode::kBuzzerSetFreqFailed;
  }

  freq_hz_ = freq_hz;
  return ErrorCode::kOk;
}

ErrorCode Buzzer::SetDutyCycle(float duty_cycle) {
  duty_ticks_ = ComputeDutyTicks(cfg_.pwm.duty_resolution, duty_cycle);

  if (!running_) {
    return ErrorCode::kOk;
  }

  if (ledc_set_duty_and_update(cfg_.pwm.speed_mode, cfg_.pwm.channel,
                               duty_ticks_, 0) != ESP_OK) {
    return ErrorCode::kBuzzerSetDutyFailed;
  }

  return ErrorCode::kOk;
}

ErrorCode Buzzer::Stop() {
  if (ledc_set_duty_and_update(cfg_.pwm.speed_mode, cfg_.pwm.channel, 0, 0) !=
      ESP_OK) {
    return ErrorCode::kBuzzerSetDutyFailed;
  }

  running_ = false;
  return ErrorCode::kOk;
}
