#include "dshot_codec.hpp"

#include "dshot_tim1.hpp"
#include "error_code.hpp"
#include "panic.hpp"

void DShotCodec::Init(const Config &cfg) {
  if (initialized_) {
    Panic(ErrorCode::Stm32::kDshotCodecInvalidArg);
  }

  if (!DShotTim1::GetInstance().IsInitialized()) {
    Panic(ErrorCode::Stm32::kDshotCodecInvalidArg);
  }

  if (cfg.gap_bits > kMaxGapBits) {
    Panic(ErrorCode::Stm32::kDshotCodecInvalidArg);
  }

  cfg_ = cfg;
  initialized_ = true;
}

bool DShotCodec::Write(const MotorValues &motor, bool telemetry) {
  TelemetryRequests telemetry_requests{};
  telemetry_requests.fill(telemetry);
  return Write(motor, telemetry_requests);
}

bool DShotCodec::Write(const MotorValues &motor,
                       const TelemetryRequests &telemetry) {
  if (!initialized_ || !DShotTim1::GetInstance().IsInitialized() ||
      DShotTim1::IsBusy()) {
    return false;
  }

  const DShotTim1Timings &timings = DShotTim1::Timings();
  const uint16_t total_bits = kFrameBits + cfg_.gap_bits;

  const uint16_t packets[kMotorCount] = {
      MakePacket(motor[0], telemetry[0]),
      MakePacket(motor[1], telemetry[1]),
      MakePacket(motor[2], telemetry[2]),
      MakePacket(motor[3], telemetry[3]),
  };

  uint32_t offset = 0;
  for (uint16_t bit = 0; bit < kFrameBits; ++bit) {
    const uint16_t mask = static_cast<uint16_t>(0x8000u >> bit);
    for (uint8_t motor_index = 0; motor_index < kMotorCount; ++motor_index) {
      buf_[offset++] =
          (packets[motor_index] & mask) != 0u ? timings.t1h : timings.t0h;
    }
  }

  for (uint16_t bit = kFrameBits; bit < total_bits; ++bit) {
    for (uint8_t motor_index = 0; motor_index < kMotorCount; ++motor_index) {
      buf_[offset++] = 0;
    }
  }

  return DShotTim1::SendBits(buf_, total_bits);
}

uint16_t DShotCodec::MakePacket(uint16_t value, bool telemetry) {
  value &= 0x07FFu;
  const uint16_t data =
      static_cast<uint16_t>((value << 1u) | (telemetry ? 1u : 0u));

  uint16_t csum = 0;
  uint16_t csum_data = data;
  for (uint8_t i = 0; i < 3u; ++i) {
    csum ^= csum_data & 0x0Fu;
    csum_data >>= 4u;
  }
  csum &= 0x0Fu;

  return static_cast<uint16_t>((data << 4u) | csum);
}
