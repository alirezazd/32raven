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
  };

  static constexpr uint16_t kDshotCommandMotorStop = 0;
  static constexpr uint16_t kDshotCommandBeacon1 = 1;
  static constexpr uint16_t kDshotCommandBeacon2 = 2;
  static constexpr uint16_t kDshotCommandBeacon3 = 3;
  static constexpr uint16_t kDshotCommandBeacon4 = 4;
  static constexpr uint16_t kDshotCommandBeacon5 = 5;
  static constexpr uint16_t kDshotCommandEscInfo = 6;
  static constexpr uint16_t kDshotCommandSpinDirection1 = 7;
  static constexpr uint16_t kDshotCommandSpinDirection2 = 8;
  static constexpr uint16_t kDshotCommand3dModeOff = 9;
  static constexpr uint16_t kDshotCommand3dModeOn = 10;
  static constexpr uint16_t kDshotCommandSettingsRequest = 11;
  static constexpr uint16_t kDshotCommandSaveSettings = 12;

  void Init(const Config &cfg, EscTelemetry &telemetry,
            VehicleState &vehicle_state);
  void Poll(uint32_t now_us);

  void SetArmed(bool armed);
  bool WriteMotors(const DShotCodec::MotorValues &motor);
  bool WriteMotors(const DShotCodec::MotorValues &motor, uint32_t now_us);
  bool StopAll();
  bool QueueCommand(uint16_t command, bool telemetry = false);

  bool IsArmed() const { return armed_; }
  bool HasPendingCommand() const { return command_.active; }
  uint32_t DroppedWriteCount() const { return dropped_write_count_; }

 private:
  struct PendingCommand {
    uint16_t value = 0;
    uint8_t repeats_remaining = 0;
    uint32_t next_send_us = 0;
    bool telemetry = false;
    bool active = false;
  };

  static uint16_t NormalizeMotorValue(uint16_t value);
  bool StopAll(uint32_t now_us);
  bool WriteRaw(const DShotCodec::MotorValues &motor, uint32_t now_us,
                bool force_telemetry);
  void PublishTelemetryState();

  Config cfg_{};
  PendingCommand command_{};
  EscTelemetry *telemetry_ = nullptr;
  VehicleState *vehicle_state_ = nullptr;
  bool initialized_ = false;
  bool armed_ = false;
  uint32_t last_idle_send_us_ = 0;
  uint32_t last_telemetry_request_us_ = 0;
  uint32_t dropped_write_count_ = 0;
  uint8_t next_telemetry_motor_ = 0;
};
