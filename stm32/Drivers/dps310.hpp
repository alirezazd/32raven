// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include <cstdint>
#include <optional>
#include <span>

#include "i2c.hpp"
#include "shared_state.hpp"

// DPS310 barometric pressure sensor (Infineon, datasheet v1.0), on the sensor
// bus beside the compass. Background mode: the part measures on its own clock,
// and this driver collects each finished result, compensates it with the
// part's own calibration coefficients and publishes it unfiltered.
//
// Bring-up blocks, which is legal only because it runs before the control loop
// and the bus enforces its own deadline. The per-sample read does not.
class Dps310 {
 public:
  // PM_RATE and TMP_RATE: results per second in background mode.
  enum class Rate : uint8_t {
    k1Hz = 0,
    k2Hz = 1,
    k4Hz = 2,
    k8Hz = 3,
    k16Hz = 4,
    k32Hz = 5,
    k64Hz = 6,
    k128Hz = 7,
  };
  // PM_PRC: internal measurements averaged into one result.
  enum class Oversampling : uint8_t {
    k1 = 0,
    k2 = 1,
    k4 = 2,
    k8 = 3,
    k16 = 4,
    k32 = 5,
    k64 = 6,
    k128 = 7,
  };

  // The SDO pin doubles as the address strap.
  enum class Address : uint8_t {
    kSdoLow = 0x76,
    kSdoHigh = 0x77,  // also SDO floating
  };

  struct Config {
    Address address;
    Oversampling pressure_oversampling;
    Rate pressure_rate;
    Rate temperature_rate;
    // Shorter than the pressure period, so a finished result waits at most
    // this long to be collected.
    uint32_t sample_period_us;
  };

  static Dps310 &GetInstance();

  void Poll(uint32_t now_us);

 private:
  friend class System;

  Dps310() = default;
  ~Dps310() = default;
  Dps310(const Dps310 &) = delete;
  Dps310 &operator=(const Dps310 &) = delete;

  using Bus = I2c1::Port<I2cTenant::kDps310>;
  enum class Reg : uint8_t {
    kPsrB2 = 0x00,  // the first of three pressure bytes, then three temperature
    kPrsCfg = 0x06,
    kTmpCfg = 0x07,
    kMeasCfg = 0x08,
    kIntFifoCfg = 0x09,  // CFG_REG
    kReset = 0x0C,
    kProductId = 0x0D,
    kCoef = 0x10,  // the first of 18
    kCoefSrce = 0x28,
  };

  static constexpr size_t kCoefLen = 18;
  static constexpr size_t kDataLen = 6;  // pressure then temperature, 24 bits

  void Init(const Config &cfg, Bus bus, SharedState &blackboard);

  // The ready flags sit after the results, so one burst cannot cover both.
  // Flags first: PRS_RDY clears when the pressure is read.
  enum class Phase : uint8_t { kIdle, kReadingStatus, kReadingData };

  // Datasheet table 18, sign-extended.
  struct Coefficients {
    float c0;
    float c1;
    float c00;
    float c10;
    float c01;
    float c11;
    float c20;
    float c21;
    float c30;
  };

  enum class BringUpResult : uint8_t {
    kOk,
    kAbsent,          // nothing answered at the configured address
    kUnexpectedPart,  // something answered with another product ID
    kNotReady,        // never reported its coefficients and sensor ready
    kRefused,         // answered, then failed a transfer
    kConfigNotKept,   // took every write, then read back something else
  };

  static Coefficients ParseCoefficients(std::span<const uint8_t, kCoefLen> raw);

  // Spins on the bus until it settles, which its own deadline bounds. Init
  // only: after the loop starts nothing may hold the main tick this long.
  I2cTransferStatus AwaitBus();
  bool WriteRegister(Reg target, uint8_t value);
  std::optional<uint8_t> ReadRegister(Reg source);
  std::optional<Coefficients> ReadCoefficients();
  BringUpResult BringUp();
  bool StartRead(Reg first, size_t len);
  void DecodeSample(std::span<const uint8_t, kDataLen> rx, uint32_t now_us);

  bool initialized_ = false;
  Config cfg_{};
  Bus bus_{};
  SharedState *blackboard_ = nullptr;
  Coefficients coef_{};
  float pressure_per_count_ = 0.0f;
  Phase phase_ = Phase::kIdle;
  uint32_t last_start_us_ = 0;
  // The temperature register reads zero until the first temperature lands,
  // and a pressure compensated against it is plausible nonsense.
  bool temperature_seen_ = false;
  BarometerData data_{};
};
