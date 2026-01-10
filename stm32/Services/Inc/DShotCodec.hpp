#pragma once
#include <cstdint>

// Forward declaration of the HW driver
class DShotTim1;

class DShotCodec {
public:
  struct Config {
    uint8_t gap_bits; // extra idle bits after 16-bit frame (0..8)
  };

  static DShotCodec &getInstance() {
    static DShotCodec instance;
    return instance;
  }

  static void init(const Config &cfg) { getInstance()._init(cfg); }

  // Called from PID loop
  // motor values must be 0..2047 (11-bit)
  static void write(const uint16_t motor[4], bool telemetry = false) {
    // TODO: Come up with a better way to do this
    getInstance()._write(motor, telemetry);
  }

private:
  DShotCodec() = default;
  DShotCodec(const DShotCodec &) = delete;
  DShotCodec &operator=(const DShotCodec &) = delete;

  void _init(const Config &cfg);
  void _write(const uint16_t motor[4], bool telemetry);

  static uint16_t makePacket(uint16_t value11, bool telemetry);

  static constexpr uint16_t FRAME_BITS = 16;
  static constexpr uint16_t MAX_GAP_BITS = 8;
  static constexpr uint16_t MAX_BITS = FRAME_BITS + MAX_GAP_BITS;
  static constexpr uint16_t CHANNELS = 4;

  uint16_t buf_[MAX_BITS * CHANNELS] = {};

  Config cfg_{};
};
