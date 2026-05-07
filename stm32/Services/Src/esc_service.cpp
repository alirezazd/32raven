#include "esc_service.hpp"

#include "error_code.hpp"
#include "panic.hpp"
#include "system.hpp"

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
  }

  if (command_.active &&
      (command_.next_send_us == 0u ||
       static_cast<int32_t>(now_us - command_.next_send_us) >= 0)) {
    const DShotCodec::MotorValues command_values = {
        command_.value,
        command_.value,
        command_.value,
        command_.value,
    };

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

  if (!armed_ && (last_idle_send_us_ == 0u ||
                  static_cast<int32_t>(now_us - last_idle_send_us_) >=
                      static_cast<int32_t>(cfg_.idle_period_us))) {
    if (StopAll(now_us)) {
      last_idle_send_us_ = now_us;
    }
  }
}

void EscService::SetArmed(bool armed) {
  if (!initialized_) {
    Panic(ErrorCode::Stm32::kEscServiceInitFailed);
  }

  command_ = PendingCommand{};
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

bool EscService::QueueCommand(uint16_t command, bool telemetry) {
  if (!initialized_) {
    Panic(ErrorCode::Stm32::kEscServiceInitFailed);
  }

  if (armed_ || command > DShotCodec::kCommandMax) {
    return false;
  }

  command_ = PendingCommand{
      .value = command,
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
  if (telemetry_ != nullptr && telemetry_->IsInitialized() && now_us != 0u &&
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
