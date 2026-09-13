// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include <cstdint>
#include <span>

#include "i2c.hpp"
#include "shared_state.hpp"

// QMC5883P three-axis magnetometer (QST, QST-PD-B002-22 Rev E) -- the compass
// on the HGLRC M100-5883, whose other half is the M10 on USART2.
//
// Not the QMC5883L, which the same module has also shipped with and which
// shares nothing but a name: different address, different chip ID, data one
// register higher, status somewhere else entirely, and four field ranges
// instead of two. BringUp probes for the L so an aircraft carrying one is told
// which part it has rather than left with a dead bus.
//
// Bring-up blocks, which is legal only because it runs before the control loop
// and the bus enforces its own deadline. The per-sample read does not, because
// it runs on the main tick beside everything else.
class Qmc5883p {
 public:
  // Control register field values, datasheet tables 17 and 18.
  enum class Odr : uint8_t { k10Hz = 0, k50Hz = 1, k100Hz = 2, k200Hz = 3 };
  // Bandwidth of the internal filter: a larger ratio is quieter and draws
  // more. Spelled for the ratio itself, which the datasheet numbers backwards
  // from the register value.
  enum class Osr1 : uint8_t { k8 = 0, k4 = 1, k2 = 2, k1 = 3 };
  // A second filter behind the first, as a downsampling depth.
  enum class Osr2 : uint8_t { k1 = 0, k2 = 1, k4 = 2, k8 = 3 };
  enum class Range : uint8_t { k30G = 0, k12G = 1, k8G = 2, k2G = 3 };

  struct Config {
    Odr odr;
    Osr1 osr1;
    Osr2 osr2;
    Range range;
    // Independent of the chip's own rate on purpose. Reading slower than the
    // ODR just means DRDY is clear on some passes, which costs nothing;
    // reading faster re-reads a sample the chip has not replaced yet.
    uint32_t sample_period_us;

    // Chip frame to body frame, applied here and nowhere else -- the same
    // rule and the same shape as the IMU's, so no downstream consumer re-flips
    // an axis. Each component names the chip axis feeding it (0=X, 1=Y, 2=Z).
    struct AxisMap {
      uint8_t x_from;
      bool x_neg;
      uint8_t y_from;
      bool y_neg;
      uint8_t z_from;
      bool z_neg;
    } axes;
  };

  static Qmc5883p &GetInstance();

  // The same shape the IMU reports: bus type, bus, address, part. Panics
  // rather than returning zero, because a zero identifier reads as a valid
  // device to whatever stores a calibration against it.
  uint32_t GetDeviceId() const;

  void Poll(uint32_t now_us);

 private:
  friend class System;

  Qmc5883p() = default;
  ~Qmc5883p() = default;
  Qmc5883p(const Qmc5883p &) = delete;
  Qmc5883p &operator=(const Qmc5883p &) = delete;

  void Init(const Config &cfg, I2c1 &bus, SharedState &blackboard);

  // Status and data sit either side of two registers the map does not list, so
  // one burst cannot cover both. Read in this order because DRDY clears when
  // the status register is read while the output registers hold their sample
  // until the chip replaces it -- so the flags describe the data that follows.
  enum class Phase : uint8_t { kIdle, kReadingStatus, kReadingData };

  // Spins on the bus until it settles, which its own deadline bounds. Init
  // only: after the loop starts nothing may hold the main tick this long.
  I2cTransferStatus AwaitBus();
  bool WriteRegister(uint8_t reg, uint8_t value);
  bool ReadRegister(uint8_t reg, uint8_t &value);
  bool AddressAnswers(uint8_t addr7);
  // Probe, reset, configure. Anything but kOk means there is no usable
  // compass, and which value says why -- each maps to its own panic code,
  // which at boot is the only thing a driver can say: FcLink is brought up
  // after System, so nothing is listening yet.
  enum class BringUpResult : uint8_t {
    kOk,
    kAbsent,          // nothing answered at either part's address
    kUnexpectedPart,  // a QMC5883L answered instead
    kRefused,         // answered, then would not take its configuration
    kConfigNotKept,   // took every write, then read back something else
  };

  BringUpResult BringUp();
  bool StartRead(uint8_t reg, size_t len);
  void DecodeSample(std::span<const uint8_t> rx, uint32_t now_us);

  bool initialized_ = false;
  uint32_t device_id_ = 0;
  Config cfg_{};
  float microtesla_per_count_ = 0.0f;
  I2c1 *bus_ = nullptr;
  SharedState *blackboard_ = nullptr;
  Phase phase_ = Phase::kIdle;
  uint32_t last_start_us_ = 0;
  // Carried between the two transfers of one sample.
  uint8_t status_ = 0;
  MagnetometerData data_{};
};
