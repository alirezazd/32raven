#include "DShotCodec.hpp"
#include "DShotTim1.hpp"
#include "system.hpp"

void DShotCodec::_init(const Config &cfg) {
  if (cfg.gap_bits > MAX_GAP_BITS) {
    ErrorHandler();
  }

  cfg_ = cfg;
}

void DShotCodec::_write(const uint16_t motor[4], bool telemetry) {

  // If hardware is busy, just skip.
  // This is mathematically safe at sane rates above DShot150
  if (DShotTim1::isBusy()) {
    return;
  }

  const auto kTimings = DShotTim1::timings();
  const uint16_t kTotalBits = FRAME_BITS + cfg_.gap_bits;

  const uint16_t kP1 = makePacket(motor[0], telemetry);
  const uint16_t kP2 = makePacket(motor[1], telemetry);
  const uint16_t kP3 = makePacket(motor[2], telemetry);
  const uint16_t kP4 = makePacket(motor[3], telemetry);

  uint32_t o = 0;

  // MSB first
  for (uint16_t b = 0; b < FRAME_BITS; ++b) {
    const uint16_t kMask = static_cast<uint16_t>(0x8000u >> b);

    buf_[o++] = (kP1 & kMask) ? kTimings.t1h : kTimings.t0h;
    buf_[o++] = (kP2 & kMask) ? kTimings.t1h : kTimings.t0h;
    buf_[o++] = (kP3 & kMask) ? kTimings.t1h : kTimings.t0h;
    buf_[o++] = (kP4 & kMask) ? kTimings.t1h : kTimings.t0h;
  }

  // gap bits = idle low
  for (uint16_t b = FRAME_BITS; b < kTotalBits; ++b) {
    buf_[o++] = 0;
    buf_[o++] = 0;
    buf_[o++] = 0;
    buf_[o++] = 0;
  }

  DShotTim1::sendBits(buf_, kTotalBits);
}

uint16_t DShotCodec::makePacket(uint16_t value11, bool telemetry) {
  value11 &= 0x07FFu;

  // 12-bit data: [11-bit value][telemetry]
  const uint16_t kData =
      static_cast<uint16_t>((value11 << 1) | (telemetry ? 1u : 0u));

  // 4-bit checksum: XOR of 3 nibbles
  uint16_t csum = 0;
  uint16_t x = kData;
  for (int i = 0; i < 3; ++i) {
    csum ^= (x & 0xFu);
    x >>= 4;
  }
  csum &= 0xFu;

  return static_cast<uint16_t>((kData << 4) | csum);
}
