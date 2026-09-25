// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "dps310.hpp"

#include <array>
#include <utility>

#include "error_code.hpp"
#include "panic.hpp"
#include "system.hpp"
#include "time_base.hpp"

namespace {

// The low nibble of the ID register; the high one is the silicon revision.
constexpr uint8_t kProductIdMask = 0x0F;
constexpr uint8_t kProductIdDps310 = 0x00;

constexpr uint8_t kMeasCfgCoefRdy = 1u << 7;
constexpr uint8_t kMeasCfgSensorRdy = 1u << 6;
constexpr uint8_t kMeasCfgTmpRdy = 1u << 5;
constexpr uint8_t kMeasCfgPrsRdy = 1u << 4;
constexpr uint8_t kMeasCtrlMask = 0x07;
constexpr uint8_t kMeasCtrlContinuous = 0x07;  // pressure and temperature

constexpr uint8_t kPrsCfgMask = 0x7F;  // bit 7 is reserved
constexpr uint8_t kTmpCfgExternal = 1u << 7;
constexpr uint8_t kIntFifoCfgPressureShift = 1u << 2;
constexpr uint8_t kCoefSrceExternal = 1u << 7;
constexpr uint8_t kSoftReset = 0x09;

// Past 8x the result no longer fits the register without a shift, and the
// scale factors below assume it is applied.
constexpr Dps310::Oversampling kLongestUnshifted = Dps310::Oversampling::k8;

// PX4's device-id layout, as the compass builds it.
constexpr uint32_t kDeviceBusTypeI2c = 1u;

// Five tries over a tenth of a second, the same regulator allowance the
// compass makes.
constexpr uint8_t kProbeAttempts = 5;
constexpr uint32_t kProbeRetryUs = 20000;

// The coefficients land at most 40 ms after a reset; the ready flags are
// then polled rather than trusted to that figure.
constexpr uint32_t kResetSettleUs = 40000;
constexpr uint8_t kReadyAttempts = 10;
constexpr uint32_t kReadyRetryUs = 10000;

constexpr uint8_t kConfigAttempts = 4;
constexpr uint32_t kConfigRetryUs = 5000;

// Datasheet table 9, indexed by Oversampling.
constexpr std::array<float, 8> kScaleFactor = {
    524288.0f,  1572864.0f, 3670016.0f, 7864320.0f,
    253952.0f,  516096.0f,  1040384.0f, 2088960.0f,
};
// Temperature runs at single oversampling, as every configuration in the
// datasheet's table 10 has it: it only corrects the pressure.
constexpr float kTemperaturePerCount = 1.0f / kScaleFactor[0];

float FromTwosComplement(uint32_t value, uint8_t bits) {
  const uint32_t sign = 1u << (bits - 1u);
  return static_cast<float>(static_cast<int32_t>((value ^ sign) - sign));
}

float LoadBe24(uint8_t msb, uint8_t mid, uint8_t lsb) {
  return FromTwosComplement((static_cast<uint32_t>(msb) << 16u) |
                                (static_cast<uint32_t>(mid) << 8u) | lsb,
                            24u);
}

}  // namespace

Dps310 &Dps310::GetInstance() {
  static Dps310 instance;
  return instance;
}

Dps310::Coefficients Dps310::ParseCoefficients(
    std::span<const uint8_t, kCoefLen> raw) {
  const auto byte = [&](size_t i) { return static_cast<uint32_t>(raw[i]); };
  return Coefficients{
      .c0 = FromTwosComplement((byte(0) << 4u) | (byte(1) >> 4u), 12u),
      .c1 = FromTwosComplement(((byte(1) & 0x0Fu) << 8u) | byte(2), 12u),
      .c00 = FromTwosComplement(
          (byte(3) << 12u) | (byte(4) << 4u) | (byte(5) >> 4u), 20u),
      .c10 = FromTwosComplement(
          ((byte(5) & 0x0Fu) << 16u) | (byte(6) << 8u) | byte(7), 20u),
      .c01 = FromTwosComplement((byte(8) << 8u) | byte(9), 16u),
      .c11 = FromTwosComplement((byte(10) << 8u) | byte(11), 16u),
      .c20 = FromTwosComplement((byte(12) << 8u) | byte(13), 16u),
      .c21 = FromTwosComplement((byte(14) << 8u) | byte(15), 16u),
      .c30 = FromTwosComplement((byte(16) << 8u) | byte(17), 16u),
  };
}

I2cTransferStatus Dps310::AwaitBus() {
  auto &time = System::GetInstance().Time();
  for (;;) {
    const I2cTransferStatus status = bus_.Poll(time.Micros());
    if (status != I2cTransferStatus::kBusy) {
      return status;
    }
  }
}

bool Dps310::WriteRegister(Reg target, uint8_t value) {
  const std::array<uint8_t, 2> tx = {std::to_underlying(target), value};
  if (bus_.StartWrite(std::to_underlying(cfg_.address), tx) != Outcome::kOk) {
    return false;
  }
  return AwaitBus() == I2cTransferStatus::kComplete;
}

std::optional<uint8_t> Dps310::ReadRegister(Reg source) {
  if (!StartRead(source, 1) || AwaitBus() != I2cTransferStatus::kComplete) {
    return std::nullopt;
  }
  const std::span<const uint8_t> rx = bus_.Received();
  if (rx.empty()) {
    return std::nullopt;
  }
  return rx[0];
}

std::optional<Dps310::Coefficients> Dps310::ReadCoefficients() {
  if (!StartRead(Reg::kCoef, kCoefLen) ||
      AwaitBus() != I2cTransferStatus::kComplete) {
    return std::nullopt;
  }
  const std::span<const uint8_t> rx = bus_.Received();
  if (rx.size() < kCoefLen) {
    return std::nullopt;
  }
  return ParseCoefficients(rx.first<kCoefLen>());
}

bool Dps310::StartRead(Reg first, size_t len) {
  const std::array<uint8_t, 1> tx = {std::to_underlying(first)};
  return bus_.StartWriteRead(std::to_underlying(cfg_.address), tx, len) ==
         Outcome::kOk;
}

Dps310::BringUpResult Dps310::BringUp() {
  std::optional<uint8_t> id;
  for (uint8_t attempt = 0; attempt < kProbeAttempts && !id; ++attempt) {
    if (attempt != 0u) {
      System::GetInstance().Time().DelayMicros(kProbeRetryUs);
    }
    id = ReadRegister(Reg::kProductId);
  }
  if (!id) {
    return BringUpResult::kAbsent;
  }
  if ((*id & kProductIdMask) != kProductIdDps310) {
    return BringUpResult::kUnexpectedPart;
  }
  data_.device_id =
      kDeviceBusTypeI2c | (static_cast<uint32_t>(I2cInstance::kI2c1) << 3) |
      (static_cast<uint32_t>(std::to_underlying(cfg_.address)) << 8) |
      (static_cast<uint32_t>(*id) << 16);

  // A firmware flash restarts this code without power-cycling the part, which
  // may still be measuring on the previous image's configuration.
  if (!WriteRegister(Reg::kReset, kSoftReset)) {
    return BringUpResult::kRefused;
  }
  System::GetInstance().Time().DelayMicros(kResetSettleUs);

  bool ready = false;
  for (uint8_t attempt = 0; attempt < kReadyAttempts && !ready; ++attempt) {
    if (attempt != 0u) {
      System::GetInstance().Time().DelayMicros(kReadyRetryUs);
    }
    const std::optional<uint8_t> meas_cfg = ReadRegister(Reg::kMeasCfg);
    if (!meas_cfg) {
      return BringUpResult::kRefused;
    }
    constexpr uint8_t kReady = kMeasCfgCoefRdy | kMeasCfgSensorRdy;
    ready = (*meas_cfg & kReady) == kReady;
  }
  if (!ready) {
    return BringUpResult::kNotReady;
  }

  const std::optional<Coefficients> coef = ReadCoefficients();
  const std::optional<uint8_t> coef_srce = ReadRegister(Reg::kCoefSrce);
  if (!coef || !coef_srce) {
    return BringUpResult::kRefused;
  }
  coef_ = *coef;

  // The temperature coefficients were fitted against one of the part's two
  // temperature sensors, and are only valid read against that one.
  const uint8_t tmp_ext =
      (*coef_srce & kCoefSrceExternal) != 0u ? kTmpCfgExternal : 0u;
  const uint8_t prs_cfg = static_cast<uint8_t>(
      (static_cast<uint8_t>(cfg_.pressure_rate) << 4) |
      static_cast<uint8_t>(cfg_.pressure_oversampling));
  const uint8_t tmp_cfg = static_cast<uint8_t>(
      tmp_ext | (static_cast<uint8_t>(cfg_.temperature_rate) << 4));
  const uint8_t int_fifo_cfg =
      cfg_.pressure_oversampling > kLongestUnshifted ? kIntFifoCfgPressureShift
                                                     : 0u;

  for (uint8_t attempt = 0; attempt < kConfigAttempts; ++attempt) {
    if (attempt != 0u) {
      System::GetInstance().Time().DelayMicros(kConfigRetryUs);
    }
    if (!WriteRegister(Reg::kPrsCfg, prs_cfg) ||
        !WriteRegister(Reg::kTmpCfg, tmp_cfg) ||
        !WriteRegister(Reg::kIntFifoCfg, int_fifo_cfg)) {
      return BringUpResult::kRefused;
    }
    const std::optional<uint8_t> seen_prs = ReadRegister(Reg::kPrsCfg);
    const std::optional<uint8_t> seen_tmp = ReadRegister(Reg::kTmpCfg);
    const std::optional<uint8_t> seen_cfg = ReadRegister(Reg::kIntFifoCfg);
    if (!seen_prs || !seen_tmp || !seen_cfg) {
      return BringUpResult::kRefused;
    }
    if (((*seen_prs ^ prs_cfg) & kPrsCfgMask) != 0u || *seen_tmp != tmp_cfg ||
        *seen_cfg != int_fifo_cfg) {
      continue;
    }

    // Measuring starts only once the configuration is known to have taken.
    if (!WriteRegister(Reg::kMeasCfg, kMeasCtrlContinuous)) {
      return BringUpResult::kRefused;
    }
    const std::optional<uint8_t> seen_meas = ReadRegister(Reg::kMeasCfg);
    if (!seen_meas) {
      return BringUpResult::kRefused;
    }
    return (*seen_meas & kMeasCtrlMask) == kMeasCtrlContinuous
               ? BringUpResult::kOk
               : BringUpResult::kConfigNotKept;
  }
  return BringUpResult::kConfigNotKept;
}

void Dps310::Init(const Config &cfg, Bus bus, SharedState &blackboard) {
  if (initialized_) {
    Panic(ErrorCode::Stm32::kBaroReinit);
  }
  if (cfg.sample_period_us == 0u) {
    Panic(ErrorCode::Stm32::kBaroInitFailed);
  }

  cfg_ = cfg;
  bus_ = bus;
  blackboard_ = &blackboard;
  pressure_per_count_ =
      1.0f /
      kScaleFactor[static_cast<size_t>(cfg.pressure_oversampling)];

  // Held to the compass's rule: a sensor the build expects and the bus cannot
  // find is a wiring fault, and the panic code is all a driver can say at boot.
  switch (BringUp()) {
    case BringUpResult::kOk:
      break;
    case BringUpResult::kAbsent:
      Panic(ErrorCode::Stm32::kBaroNotResponding);
    case BringUpResult::kUnexpectedPart:
      Panic(ErrorCode::Stm32::kBaroUnexpectedPart);
    case BringUpResult::kNotReady:
      Panic(ErrorCode::Stm32::kBaroNotReady);
    case BringUpResult::kRefused:
      Panic(ErrorCode::Stm32::kBaroConfigRefused);
    case BringUpResult::kConfigNotKept:
      Panic(ErrorCode::Stm32::kBaroConfigNotKept);
  }

  initialized_ = true;
}

void Dps310::Poll(uint32_t now_us) {
  if (!initialized_) {
    return;
  }

  if (phase_ != Phase::kIdle) {
    const I2cTransferStatus status = bus_.Poll(now_us);
    if (status == I2cTransferStatus::kBusy) {
      return;
    }
    if (status != I2cTransferStatus::kComplete) {
      // Counted by the bus in SystemHealth::sensor_i2c; a stamp that stops
      // moving is what tells a reader the barometer has stopped answering.
      phase_ = Phase::kIdle;
      return;
    }

    if (phase_ == Phase::kReadingStatus) {
      const std::span<const uint8_t> rx = bus_.Received();
      const uint8_t meas_cfg = rx.empty() ? 0u : rx[0];
      if ((meas_cfg & kMeasCfgTmpRdy) != 0u) {
        temperature_seen_ = true;
      }
      if ((meas_cfg & kMeasCfgPrsRdy) == 0u || !temperature_seen_ ||
          !StartRead(Reg::kPsrB2, kDataLen)) {
        phase_ = Phase::kIdle;
        return;
      }
      phase_ = Phase::kReadingData;
      return;
    }

    phase_ = Phase::kIdle;
    // Unreachable by the bus's contract -- a kComplete read hands back what
    // was asked for -- and checked so the span below is sized by type.
    const std::span<const uint8_t> rx = bus_.Received();
    if (rx.size() >= kDataLen) {
      DecodeSample(rx.first<kDataLen>(), now_us);
    }
    return;
  }

  if (last_start_us_ != 0u &&
      static_cast<uint32_t>(now_us - last_start_us_) < cfg_.sample_period_us) {
    return;
  }

  // Rejected means another tenant's transfer is on the wire. last_start_us_
  // stays put, so the read retries on the next tick.
  if (!StartRead(Reg::kMeasCfg, 1)) {
    return;
  }
  last_start_us_ = now_us;
  phase_ = Phase::kReadingStatus;
}

void Dps310::DecodeSample(std::span<const uint8_t, kDataLen> rx,
                          uint32_t now_us) {
  const float ps = LoadBe24(rx[0], rx[1], rx[2]) * pressure_per_count_;
  const float ts = LoadBe24(rx[3], rx[4], rx[5]) * kTemperaturePerCount;

  // Datasheet sections 4.9.1 and 4.9.2.
  data_.pressure_pa =
      coef_.c00 +
      (ps * (coef_.c10 + (ps * (coef_.c20 + (ps * coef_.c30))))) +
      (ts * coef_.c01) + (ts * ps * (coef_.c11 + (ps * coef_.c21)));
  data_.temperature_c = (coef_.c0 * 0.5f) + (coef_.c1 * ts);
  data_.timestamp_us = now_us;
  blackboard_->UpdateBarometer(data_);
}
