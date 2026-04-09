#include "DShotCodec.hpp"

#include "DShotTim1.hpp"
#include "system.hpp"

void DShotCodec::InitImpl(const Config &cfg) {
  if (cfg.gap_bits > MAX_GAP_BITS) {
    ErrorHandler();
  }

  cfg_ = cfg;
}

void DShotCodec::WriteImpl(const uint16_t motor[4], bool telemetry) {
  // If hardware is busy, just skip.
  // This is mathematically safe at sane rates above DShot150
  if (DShotTim1::isBusy()) {
    return;
  }

  const auto timings = DShotTim1::timings();
  const uint16_t total_bits = FRAME_BITS + cfg_.gap_bits;

  const uint16_t p1 = MakePacket(motor[0], telemetry);
  const uint16_t p2 = MakePacket(motor[1], telemetry);
  const uint16_t p3 = MakePacket(motor[2], telemetry);
  const uint16_t p4 = MakePacket(motor[3], telemetry);

  uint32_t o = 0;

  // MSB first
  for (uint16_t b = 0; b < FRAME_BITS; ++b) {
    const uint16_t mask = static_cast<uint16_t>(0x8000u >> b);

    buf_[o++] = (p1 & mask) ? timings.t1h : timings.t0h;
    buf_[o++] = (p2 & mask) ? timings.t1h : timings.t0h;
    buf_[o++] = (p3 & mask) ? timings.t1h : timings.t0h;
    buf_[o++] = (p4 & mask) ? timings.t1h : timings.t0h;
  }

  // gap bits = idle low
  for (uint16_t b = FRAME_BITS; b < total_bits; ++b) {
    buf_[o++] = 0;
    buf_[o++] = 0;
    buf_[o++] = 0;
    buf_[o++] = 0;
  }

  DShotTim1::sendBits(buf_, total_bits);
}

uint16_t DShotCodec::MakePacket(uint16_t value11, bool telemetry) {
  value11 &= 0x07FFu;

  // 12-bit data: [11-bit value][telemetry]
  const uint16_t data =
      static_cast<uint16_t>((value11 << 1) | (telemetry ? 1u : 0u));

  // 4-bit checksum: XOR of 3 nibbles
  uint16_t csum = 0;
  uint16_t x = data;
  for (int i = 0; i < 3; ++i) {
    csum ^= (x & 0xFu);
    x >>= 4;
  }
  csum &= 0xFu;

  return static_cast<uint16_t>((data << 4) | csum);
}
