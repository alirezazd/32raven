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
// AM32's inputType enum, of which these three end up driving DShot.
static constexpr uint8_t kInputTypeAuto = 0;
static constexpr uint8_t kInputTypeDshot = 1;
static constexpr uint8_t kInputTypeDshotEdtArm = 4;

uint16_t EscService::ThrustToDshot(float thrust) {
  if (thrust <= 0.0f) return DShotCodec::kMotorStop;
  if (thrust >= 1.0f) return DShotCodec::kThrottleMax;
  const float scaled = static_cast<float>(DShotCodec::kThrottleMin) +
                       (thrust * static_cast<float>(DShotCodec::kThrottleMax -
                                                   DShotCodec::kThrottleMin));
  return static_cast<uint16_t>(scaled);
}

float EscService::DshotToThrust(uint16_t value) {
  if (value < DShotCodec::kThrottleMin) return 0.0f;
  if (value >= DShotCodec::kThrottleMax) return 1.0f;
  return static_cast<float>(value - DShotCodec::kThrottleMin) /
         static_cast<float>(DShotCodec::kThrottleMax -
                            DShotCodec::kThrottleMin);
}

namespace {

// A frame with fewer motors than TIM1 has channels leaves the spare ones
// configured and idle, which is what DShotCodec::kMotorStop encodes.
DShotCodec::MotorValues ThrustToDshotValues(
    const multirotor_mixer::MotorThrust &thrust) {
  DShotCodec::MotorValues values{};
  values.fill(DShotCodec::kMotorStop);
  for (uint8_t i = 0; i < common_config::kAirframeMotorCount; ++i) {
    values[i] = EscService::ThrustToDshot(thrust[i]);
  }
  return values;
}

}  // namespace

bool EscService::WriteMotorsThrust(const multirotor_mixer::MotorThrust &thrust) {
  return WriteMotors(ThrustToDshotValues(thrust));
}

bool EscService::WriteMotorsThrust(const multirotor_mixer::MotorThrust &thrust,
                                   uint32_t now_us) {
  return WriteMotors(ThrustToDshotValues(thrust), now_us);
}

void EscService::Init(const Config &cfg, DShotCodec &codec,
                      EscTelemetry &telemetry, SharedState &blackboard) {
  if (initialized_) {
    Panic(ErrorCode::Stm32::kEscServiceInitFailed);
  }

  if (cfg.idle_period_us == 0u || cfg.command_period_us == 0u ||
      cfg.command_repeat_count == 0u || cfg.telemetry_request_period_us == 0u) {
    Panic(ErrorCode::Stm32::kEscServiceInitFailed);
  }

  cfg_ = cfg;
  codec_ = &codec;
  telemetry_ = &telemetry;
  blackboard_ = &blackboard;
  codec_->Init(cfg_.dshot);
  initialized_ = true;
}

void EscService::Poll(uint32_t now_us) {
  if (telemetry_ != nullptr) {
    telemetry_->Poll(now_us);
    CheckEscFirmware();
  }

  // Armed, the control loop owns the wire, so a burst still standing from
  // before the arm is abandoned rather than paused. Paused is not an option in
  // either direction: these frames are written from the main tick while the
  // loop writes thrust from the control tick, and WriteMotors refuses outright
  // while a command stands -- so leaving one active would cut the motors for
  // the rest of the flight.
  if (command_.active && blackboard_->IsArmed()) {
    command_ = PendingCommand{};
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
  if (!blackboard_->IsArmed() && !command_.active &&
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

    // 3D reads the lower half of the throttle range as reverse, and
    // ThrustToDshot spans the whole range forward-only -- everything below
    // half thrust would drive that motor backwards. Roadmap #13 is the signed
    // thrust chain that would make the mode meaningful.
    if (info.bidirectional) {
      Panic(ErrorCode::Stm32::kEsc3dModeEnabled);
    }
  }
}

// Telemetry arriving from a motor is the proof the info request needs: only a
// powered, armed, stopped ESC answers, which is exactly what AM32 requires
// before it will act on a command. It also covers a battery arriving long after
// boot, with no timer to get wrong.
void EscService::PollEscInfo(uint32_t now_us) {
  // A host holding the ESC config grant owns the motors for as long as it holds
  // it, throttle standing or not: the command burst below writes kMotorStop to
  // every motor it is not addressed to, and a burst already under way when the
  // host's slider moves stops that motor just the same.
  const bool host_owns_motors = blackboard_->GetUsbStatus().esc_config_granted;
  if (blackboard_->IsArmed() || command_.active || host_owns_motors ||
      telemetry_ == nullptr || !telemetry_->IsInitialized()) {
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
  if (TestThrottleActive()) {
    return WriteRaw(test_values_, now_us, false);
  }
  return StopAll(now_us);
}

void EscService::SetTestThrottle(const std::array<float, 4> &thrust) {
  if (blackboard_->IsArmed()) {
    return;
  }

  for (uint8_t i = 0; i < DShotCodec::kMotorCount; ++i) {
    test_values_[i] = ThrustToDshot(thrust[i]);
  }
}

bool EscService::TestThrottleActive() const {
  for (const uint16_t value : test_values_) {
    if (value != DShotCodec::kMotorStop) {
      return true;
    }
  }
  return false;
}

void EscService::ClearTestThrottle() {
  test_values_.fill(DShotCodec::kMotorStop);
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

bool EscService::WriteMotors(const DShotCodec::MotorValues &motor) {
  return WriteMotors(motor, System::GetInstance().Time().Micros());
}

bool EscService::WriteMotors(const DShotCodec::MotorValues &motor,
                             uint32_t now_us) {
  // Re-read rather than trusted: this is the last stage before the wire, so a
  // missed transition upstream still cannot reach a motor.
  if (!blackboard_->IsArmed() || command_.active) {
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
  const uint16_t value = std::to_underlying(command);
  if (blackboard_->IsArmed() || command_.active ||
      value > DShotCodec::kCommandMax) {
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

  if (!codec_->Write(motor, requests)) {
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
