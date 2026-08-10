// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include <cstdint>

#include "dshot_codec.hpp"
#include "esc_telemetry.hpp"
#include "vehicle_state.hpp"

class EscService {
 public:
  struct Config {
    DShotCodec::Config dshot;
    uint32_t idle_period_us;
    uint32_t command_period_us;
    uint32_t telemetry_request_period_us;
    uint8_t command_repeat_count;
    bool firmware_checks;
  };

  enum class DshotCommand : uint16_t {
    kMotorStop = 0,
    kBeacon1 = 1,
    kBeacon2 = 2,
    kBeacon3 = 3,
    kBeacon4 = 4,
    kBeacon5 = 5,
    kEscInfo = 6,
    kSpinDirection1 = 7,
    kSpinDirection2 = 8,
    k3dModeOff = 9,
    k3dModeOn = 10,
    kSettingsRequest = 11,
    kSaveSettings = 12,
  };

  void Init(const Config &cfg, EscTelemetry &telemetry,
            VehicleState &vehicle_state);
  void Poll(uint32_t now_us);

  void SetArmed(bool armed);
  [[nodiscard]] bool WriteMotors(const DShotCodec::MotorValues &motor);
  [[nodiscard]] bool WriteMotors(const DShotCodec::MotorValues &motor,
                                 uint32_t now_us);

  // Overloads taking normalized thrust [0, 1] per motor — the units mixers /
  // controllers produce. Per-motor ThrustToDshot, then WriteMotors.
  [[nodiscard]] bool WriteMotorsThrust(const std::array<float, 4> &thrust);
  [[nodiscard]] bool WriteMotorsThrust(const std::array<float, 4> &thrust,
                                       uint32_t now_us);

  // Map normalized thrust [0, 1] → DShot wire units. 0 → kMotorStop (motor
  // off). >0 → linearly into [kThrottleMin, kThrottleMax]. Saturates.
  static uint16_t ThrustToDshot(float thrust);
  static float DshotToThrust(uint16_t value);
  static constexpr uint8_t kAllMotors = 0xFF;

  [[nodiscard]] bool StopAll();
  // A command aimed at one motor leaves the others at kMotorStop, which is what
  // they are already being sent while disarmed. Anything read back arrives on
  // the telemetry line all four share, so a broadcast would collide.
  [[nodiscard]] bool QueueCommand(DshotCommand command,
                                  uint8_t motor_index = kAllMotors,
                                  bool telemetry = false);
  void SetTestThrottle(const std::array<float, 4> &thrust);

  // What actually went out on the wire, not what was asked for: a dropped
  // write leaves this at the previous frame, and the test deadman shows up
  // here as a return to stop without anyone clearing it.
  const DShotCodec::MotorValues &Outputs() const { return outputs_; }

  // Zero until an ESC has answered. The motors are identical in practice, so
  // the first one to report speaks for all of them.
  uint8_t MotorPoles() const;

  bool IsArmed() const { return armed_; }
  bool HasPendingCommand() const { return command_.active; }
  uint32_t DroppedWriteCount() const { return dropped_write_count_; }

 private:
  struct PendingCommand {
    uint16_t value = 0;
    uint8_t motor = kAllMotors;
    uint8_t repeats_remaining = 0;
    uint32_t next_send_us = 0;
    bool telemetry = false;
    bool active = false;
  };

  // Telemetry proves an ESC is powered, not that it has armed -- AM32 needs
  // about a second of zero throttle for that, and refuses commands until then.
  // The budget spans that gap rather than trying to detect it.
  static constexpr uint8_t kInfoMaxAttempts = 6;
  static constexpr uint32_t kInfoRetryPeriodUs = 250000u;

  static uint16_t NormalizeMotorValue(uint16_t value);
  void CheckEscFirmware();
  void PollEscInfo(uint32_t now_us);
  [[nodiscard]] bool StopAll(uint32_t now_us);
  [[nodiscard]] bool SendIdleFrame(uint32_t now_us);
  [[nodiscard]] bool WriteRaw(const DShotCodec::MotorValues &motor,
                              uint32_t now_us, bool force_telemetry);
  void PublishTelemetryState();

  Config cfg_{};
  PendingCommand command_{};
  EscTelemetry *telemetry_ = nullptr;
  VehicleState *vehicle_state_ = nullptr;
  bool initialized_ = false;
  bool armed_ = false;
  DShotCodec::MotorValues test_values_{};
  DShotCodec::MotorValues outputs_{};
  uint32_t test_set_us_ = 0;
  uint32_t last_idle_send_us_ = 0;
  uint32_t last_telemetry_request_us_ = 0;
  uint32_t dropped_write_count_ = 0;
  uint32_t info_last_attempt_us_ = 0;
  uint8_t info_attempts_[DShotCodec::kMotorCount] = {};
  uint8_t info_checked_mask_ = 0;
  uint8_t info_cursor_ = 0;
  uint8_t next_telemetry_motor_ = 0;
};
