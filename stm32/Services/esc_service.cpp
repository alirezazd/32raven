// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "esc_service.hpp"

#include <utility>

#include "error_code.hpp"
#include "panic.hpp"
#include "system.hpp"

// A configurator that vanishes mid-test must not leave a motor turning, so the
// commanded values carry their own expiry rather than trusting anyone to clear
// them.
static constexpr uint32_t kTestThrottleTimeoutUs = 500000u;

// AM32's inputType enum, of which these three end up driving DShot.
static constexpr uint8_t kInputTypeAuto = 0;
static constexpr uint8_t kInputTypeDshot = 1;
static constexpr uint8_t kInputTypeDshotEdtArm = 4;

uint16_t EscService::ThrustToDshot(float thrust) {
  if (thrust <= 0.0f) return DShotCodec::kMotorStop;
  if (thrust >= 1.0f) return DShotCodec::kThrottleMax;
  const float scaled = static_cast<float>(DShotCodec::kThrottleMin) +
                       thrust * static_cast<float>(DShotCodec::kThrottleMax -
                                                   DShotCodec::kThrottleMin);
  return static_cast<uint16_t>(scaled);
}

float EscService::DshotToThrust(uint16_t value) {
  if (value < DShotCodec::kThrottleMin) return 0.0f;
  if (value >= DShotCodec::kThrottleMax) return 1.0f;
  return static_cast<float>(value - DShotCodec::kThrottleMin) /
         static_cast<float>(DShotCodec::kThrottleMax -
                            DShotCodec::kThrottleMin);
}

bool EscService::WriteMotorsThrust(const std::array<float, 4> &thrust) {
  return WriteMotors({ThrustToDshot(thrust[0]), ThrustToDshot(thrust[1]),
                      ThrustToDshot(thrust[2]), ThrustToDshot(thrust[3])});
}

bool EscService::WriteMotorsThrust(const std::array<float, 4> &thrust,
                                   uint32_t now_us) {
  return WriteMotors({ThrustToDshot(thrust[0]), ThrustToDshot(thrust[1]),
                      ThrustToDshot(thrust[2]), ThrustToDshot(thrust[3])},
                     now_us);
}

void EscService::Init(const Config &cfg, EscTelemetry &telemetry,
                      VehicleState &vehicle_state) {
  if (initialized_) {
    Panic(ErrorCode::Stm32::kEscServiceInitFailed);
  }

  if (cfg.idle_period_us == 0u || cfg.command_period_us == 0u ||
      cfg.command_repeat_count == 0u || cfg.telemetry_request_period_us == 0u) {
    Panic(ErrorCode::Stm32::kEscServiceInitFailed);
  }

  cfg_ = cfg;
  telemetry_ = &telemetry;
  vehicle_state_ = &vehicle_state;
  DShotCodec::GetInstance().Init(cfg_.dshot);
  initialized_ = true;
}

void EscService::Poll(uint32_t now_us) {
  if (!initialized_) {
    Panic(ErrorCode::Stm32::kEscServiceInitFailed);
  }

  if (telemetry_ != nullptr) {
    telemetry_->Poll(now_us);
    PublishTelemetryState();
    CheckEscFirmware();
  }

  if (command_.active &&
      (command_.next_send_us == 0u ||
       static_cast<int32_t>(now_us - command_.next_send_us) >= 0)) {
    DShotCodec::MotorValues command_values{};
    for (uint8_t i = 0; i < DShotCodec::kMotorCount; ++i) {
      command_values[i] =
          (command_.motor == kAllMotors || command_.motor == i)
              ? command_.value
              : DShotCodec::kMotorStop;
    }

    if (!WriteRaw(command_values, now_us, command_.telemetry)) {
      return;
    }

    command_.repeats_remaining--;
    command_.next_send_us = now_us + cfg_.command_period_us;
    if (command_.repeats_remaining == 0u) {
      command_.active = false;
    }
    return;
  }

  PollEscInfo(now_us);

  // AM32 counts six identical frames before acting and resets that count on
  // any zero, so an idle frame landing between two command frames -- which the
  // shorter idle period guarantees -- makes every command unreachable.
  if (!armed_ && !command_.active &&
      (last_idle_send_us_ == 0u ||
       static_cast<int32_t>(now_us - last_idle_send_us_) >=
           static_cast<int32_t>(cfg_.idle_period_us))) {
    if (SendIdleFrame(now_us)) {
      last_idle_send_us_ = now_us;
    }
  }
}

// Checked once per motor, when that ESC's settings first arrive -- there is no
// earlier moment, since an ESC only answers once it is powered and has armed.
// An ESC that never answers is not a failure: a bench session on USB alone has
// no battery and reaches none of them.
void EscService::CheckEscFirmware() {
  if (!cfg_.firmware_checks) {
    return;
  }
  for (uint8_t i = 0; i < DShotCodec::kMotorCount; ++i) {
    const uint8_t bit = static_cast<uint8_t>(1u << i);
    if ((info_checked_mask_ & bit) != 0u) {
      continue;
    }
    const EscTelemetry::Info info = telemetry_->GetInfo(i);
    if (!info.valid) {
      continue;
    }
    info_checked_mask_ |= bit;

    // Auto detects DShot and both DShot modes drive it; servo, serial and
    // DroneCAN leave the ESC deaf to every frame this board sends.
    if (info.input_type != kInputTypeAuto &&
        info.input_type != kInputTypeDshot &&
        info.input_type != kInputTypeDshotEdtArm) {
      Panic(ErrorCode::Stm32::kEscInputTypeNotDshot);
    }

    // Direction belongs to the mixer here. An ESC reversing on top of it turns
    // one motor against the airframe, and nothing downstream can see it.
    if (info.reversed) {
      Panic(ErrorCode::Stm32::kEscDirectionReversed);
    }
  }
}

// Telemetry arriving from a motor is the proof the info request needs: only a
// powered, armed, stopped ESC answers, which is exactly what AM32 requires
// before it will act on a command. It also covers a battery arriving long after
// boot, with no timer to get wrong.
void EscService::PollEscInfo(uint32_t now_us) {
  if (armed_ || command_.active || telemetry_ == nullptr ||
      !telemetry_->IsInitialized()) {
    return;
  }
  if (info_last_attempt_us_ != 0u &&
      static_cast<uint32_t>(now_us - info_last_attempt_us_) <
          kInfoRetryPeriodUs) {
    return;
  }

  const uint8_t seen = telemetry_->ValidMask();
  for (uint8_t n = 0; n < DShotCodec::kMotorCount; ++n) {
    // Round robin rather than draining one motor first. AM32 needs about a
    // second of zero throttle to arm, and telemetry starts before that, so
    // whoever is asked first spends its whole budget too early.
    const uint8_t i = static_cast<uint8_t>((info_cursor_ + n) %
                                           DShotCodec::kMotorCount);
    if ((seen & (1u << i)) == 0u || telemetry_->GetInfo(i).valid ||
        info_attempts_[i] >= kInfoMaxAttempts) {
      continue;
    }
    info_cursor_ = static_cast<uint8_t>((i + 1u) % DShotCodec::kMotorCount);
    if (QueueCommand(DshotCommand::kEscInfo, i)) {
      // Opened with the first frame, not the last. AM32 acts on the sixth of
      // ten, so the reply lands mid-burst -- a window opened at the end has
      // already missed it.
      telemetry_->ExpectInfo(i, now_us);
      info_attempts_[i]++;
      info_last_attempt_us_ = (now_us == 0u) ? 1u : now_us;
    }
    return;
  }
}

// Test values ride the idle path rather than being written straight out: they
// inherit its rate, and going stale is what stops the motors.
bool EscService::SendIdleFrame(uint32_t now_us) {
  if (test_set_us_ != 0u &&
      static_cast<uint32_t>(now_us - test_set_us_) < kTestThrottleTimeoutUs) {
    return WriteRaw(test_values_, now_us, false);
  }
  test_set_us_ = 0;
  return StopAll(now_us);
}

void EscService::SetTestThrottle(const std::array<float, 4> &thrust) {
  if (!initialized_) {
    Panic(ErrorCode::Stm32::kEscServiceInitFailed);
  }
  if (armed_) {
    return;
  }

  for (uint8_t i = 0; i < DShotCodec::kMotorCount; ++i) {
    test_values_[i] = ThrustToDshot(thrust[i]);
  }
  const uint32_t now_us = System::GetInstance().Time().Micros();
  test_set_us_ = (now_us == 0u) ? 1u : now_us;
}

uint8_t EscService::MotorPoles() const {
  if (telemetry_ == nullptr) {
    return 0;
  }
  for (uint8_t i = 0; i < DShotCodec::kMotorCount; ++i) {
    const EscTelemetry::Info info = telemetry_->GetInfo(i);
    if (info.valid) {
      return info.motor_poles;
    }
  }
  return 0;
}

void EscService::SetArmed(bool armed) {
  if (!initialized_) {
    Panic(ErrorCode::Stm32::kEscServiceInitFailed);
  }

  command_ = PendingCommand{};
  test_set_us_ = 0;
  armed_ = armed;
  if (!armed_) {
    (void)StopAll();
  }
}

bool EscService::WriteMotors(const DShotCodec::MotorValues &motor) {
  return WriteMotors(motor, System::GetInstance().Time().Micros());
}

bool EscService::WriteMotors(const DShotCodec::MotorValues &motor,
                             uint32_t now_us) {
  if (!initialized_) {
    Panic(ErrorCode::Stm32::kEscServiceInitFailed);
  }

  if (!armed_ || command_.active) {
    return false;
  }

  DShotCodec::MotorValues normalized{};
  for (uint8_t i = 0; i < DShotCodec::kMotorCount; ++i) {
    normalized[i] = NormalizeMotorValue(motor[i]);
  }

  return WriteRaw(normalized, now_us, false);
}

bool EscService::StopAll() {
  return StopAll(System::GetInstance().Time().Micros());
}

bool EscService::StopAll(uint32_t now_us) {
  if (!initialized_) {
    Panic(ErrorCode::Stm32::kEscServiceInitFailed);
  }

  const DShotCodec::MotorValues stop = {
      DShotCodec::kMotorStop,
      DShotCodec::kMotorStop,
      DShotCodec::kMotorStop,
      DShotCodec::kMotorStop,
  };
  return WriteRaw(stop, now_us, false);
}

bool EscService::QueueCommand(DshotCommand command, uint8_t motor_index,
                              bool telemetry) {
  if (!initialized_) {
    Panic(ErrorCode::Stm32::kEscServiceInitFailed);
  }

  const uint16_t value = std::to_underlying(command);
  if (armed_ || command_.active || value > DShotCodec::kCommandMax) {
    return false;
  }
  if (motor_index != kAllMotors && motor_index >= DShotCodec::kMotorCount) {
    return false;
  }

  command_ = PendingCommand{
      .value = value,
      .motor = motor_index,
      .repeats_remaining = cfg_.command_repeat_count,
      .next_send_us = 0,
      .telemetry = telemetry,
      .active = true,
  };
  return true;
}

uint16_t EscService::NormalizeMotorValue(uint16_t value) {
  if (value == DShotCodec::kMotorStop) {
    return DShotCodec::kMotorStop;
  }

  if (value < DShotCodec::kThrottleMin) {
    return DShotCodec::kMotorStop;
  }

  if (value > DShotCodec::kThrottleMax) {
    return DShotCodec::kThrottleMax;
  }

  return value;
}

bool EscService::WriteRaw(const DShotCodec::MotorValues &motor, uint32_t now_us,
                          bool force_telemetry) {
  DShotCodec::TelemetryRequests requests{};
  requests.fill(false);

  uint8_t telemetry_motor = DShotCodec::kMotorCount;
  // AM32 answers a telemetry request in preference to the info packet, and a
  // request also re-aims the shared line and resizes the parser. The window is
  // open for the whole command burst, so testing it covers both.
  if (telemetry_ != nullptr && telemetry_->IsInitialized() && now_us != 0u &&
      !telemetry_->IsExpectingInfo() &&
      (force_telemetry || last_telemetry_request_us_ == 0u ||
       static_cast<uint32_t>(now_us - last_telemetry_request_us_) >=
           cfg_.telemetry_request_period_us)) {
    telemetry_motor = next_telemetry_motor_;
    requests[telemetry_motor] = true;
  }

  if (!DShotCodec::GetInstance().Write(motor, requests)) {
    dropped_write_count_++;
    return false;
  }
  outputs_ = motor;

  if (telemetry_motor < DShotCodec::kMotorCount) {
    telemetry_->ExpectMotor(telemetry_motor, now_us);
    next_telemetry_motor_ = static_cast<uint8_t>((next_telemetry_motor_ + 1u) %
                                                 DShotCodec::kMotorCount);
    last_telemetry_request_us_ = now_us;
  }

  return true;
}

void EscService::PublishTelemetryState() {
  if (telemetry_ == nullptr || vehicle_state_ == nullptr) {
    return;
  }

  const EscTelemetry::Snapshot snapshot = telemetry_->GetSnapshot();
  EscTelemetryData data{};
  data.valid_mask = snapshot.valid_mask;
  data.frame_count = snapshot.frame_count;
  data.crc_error_count = snapshot.crc_error_count;
  data.unassigned_frame_count = snapshot.unassigned_frame_count;
  data.rx_drop_bytes = snapshot.rx_drop_bytes;
  data.rx_dma_error_count = snapshot.rx_dma_error_count;
  data.uart_error_count = snapshot.uart_error_count;

  for (uint8_t i = 0; i < DShotCodec::kMotorCount; ++i) {
    const EscTelemetry::Sample &src = snapshot.motors[i];
    EscTelemetryMotorData &dst = data.motors[i];
    dst.timestamp_us = src.timestamp_us;
    dst.voltage = static_cast<float>(src.voltage_centivolts) * 0.01f;
    dst.current = static_cast<float>(src.current_centiamps) * 0.01f;
    dst.consumption_mah = src.consumption_mah;
    dst.electrical_rpm = src.electrical_rpm;
    dst.rpm = src.rpm;
    dst.temperature_c = src.temperature_c;
    dst.valid = src.valid;
  }

  vehicle_state_->UpdateEscTelemetry(data);
}
