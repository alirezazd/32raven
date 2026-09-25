// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "qmc5883p.hpp"

#include <array>
#include <utility>

#include "error_code.hpp"
#include "panic.hpp"
#include "system.hpp"
#include "time_base.hpp"

namespace {

// Fixed in silicon: no address strap, so two of them cannot share a bus.
constexpr uint8_t kAddr = 0x2C;

// The other part the same module has shipped. A different register map
// entirely, so this is not a fallback -- it is probed only so a board carrying
// one is told which part it has, which is all a driver can say at boot: FcLink
// comes up after System, so there is no link to log over yet.
constexpr uint8_t kQmc5883lAddr = 0x0D;

constexpr uint8_t kChipId = 0x80;

constexpr uint8_t kStatusDrdy = 1u << 0;
constexpr uint8_t kStatusOvfl = 1u << 1;

// Control register 1, MODE<1:0>. Suspend is the state after any reset.
//
// Normal rather than Continuous, because the ODR field paces the part only
// here. Section 9.2.3 claims otherwise, but 6.2.1 scopes ODR and RNG setup to
// "the normal mode" and 6.2.3 has Continuous running "all the time without
// sleep time, so the maximum ODR can be got" -- and that is what measured out:
// at ODR 10 Hz, Continuous still answered every 20 ms read. Continuous also
// draws the datasheet's 2200 uA row rather than the 310 uA one.
constexpr uint8_t kModeNormal = 0x01;

// Which bits of each control register carry meaning, and so which ones a
// read-back may be compared on. Control register 2 leaves bits 5:4 unlabeled,
// and a part is free to return anything in them.
constexpr uint8_t kCtrl1Mask = 0xFF;
constexpr uint8_t kCtrl2Mask = 0xCF;

// The part does not reliably keep control register 2 on the first write after
// a soft reset: it reads back as the reset default, while control register 1 --
// written a moment later -- holds. Rather than time a window the datasheet
// never specifies, write the pair and check it, and try again if it did not
// take.
constexpr uint8_t kConfigAttempts = 4;
constexpr uint32_t kConfigRetryUs = 5000;

// Control register 2: soft reset restores every default, and the offset is
// only renewed during measurement with both phases on.
constexpr uint8_t kCtrl2SoftReset = 1u << 7;
constexpr uint8_t kSetResetOn = 0x00;

// Bus type as PX4's device-id layout numbers them: bus_type<2:0>, bus<7:3>,
// address<15:8>, part<23:16>. The IMU builds its identifier the same way and
// leaves the address byte zero, having none on SPI; this one has a real
// address, so the field it was defined for gets used. The part byte carries
// the chip's own ID rather than PX4's registry number, which is what the IMU
// does with its WHO_AM_I.
constexpr uint32_t kDeviceBusTypeI2c = 1u;

// Five tries over a tenth of a second, which is the module's regulator
// settling rather than anything the part specifies.
constexpr uint8_t kProbeAttempts = 5;
constexpr uint32_t kProbeRetryUs = 20000;

// The datasheet documents a power-on reset but never times it, so this is
// generous rather than derived -- it is paid once, at boot.
constexpr uint32_t kResetSettleUs = 10000;

constexpr size_t kDataLen = 6;

// Datasheet section 3, in LSB per gauss, indexed by Range. One gauss is
// 100 uT.
constexpr std::array<float, 4> kMicroteslaPerCount = {
    100.0f / 1000.0f,   // k30G
    100.0f / 2500.0f,   // k12G
    100.0f / 3750.0f,   // k8G
    100.0f / 15000.0f,  // k2G
};

int16_t LoadLe16(uint8_t lsb, uint8_t msb) {
  return static_cast<int16_t>(static_cast<uint16_t>(lsb) |
                              (static_cast<uint16_t>(msb) << 8u));
}

}  // namespace

Qmc5883p &Qmc5883p::GetInstance() {
  static Qmc5883p instance;
  return instance;
}

uint32_t Qmc5883p::GetDeviceId() const {
  if (device_id_ == 0u) {
    Panic(ErrorCode::Stm32::kMagNotInitialized);
  }

  return device_id_;
}

I2cTransferStatus Qmc5883p::AwaitBus() {
  auto &time = System::GetInstance().Time();
  // Cannot spin forever: the bus derives a deadline from the transfer's own
  // length and reports kTimeout once it passes, tearing the engine down
  // itself.
  for (;;) {
    const I2cTransferStatus status = bus_.Poll(time.Micros());
    if (status != I2cTransferStatus::kBusy) {
      return status;
    }
  }
}

bool Qmc5883p::WriteRegister(Reg target, uint8_t value) {
  const std::array<uint8_t, 2> tx = {std::to_underlying(target), value};
  if (bus_.StartWrite(kAddr, tx) != Outcome::kOk) {
    return false;
  }
  return AwaitBus() == I2cTransferStatus::kComplete;
}

// Paired with WriteRegister so a configuration can be checked rather than
// assumed. Without it a register the part declines is indistinguishable from
// one it took, and the only symptom is a scale factor quietly describing a
// range the part is not measuring on.
std::optional<uint8_t> Qmc5883p::ReadRegister(Reg source) {
  if (!StartRead(source, 1) || AwaitBus() != I2cTransferStatus::kComplete) {
    return std::nullopt;
  }
  const std::span<const uint8_t> rx = bus_.Received();
  if (rx.empty()) {
    return std::nullopt;
  }
  return rx[0];
}

// A zero-length write: START, address, STOP. The bus answers kComplete on an
// ACK and kNackAddr when nothing is there.
bool Qmc5883p::AddressAnswers(uint8_t addr7) {
  if (bus_.StartWrite(addr7, {}) != Outcome::kOk) {
    return false;
  }
  return AwaitBus() == I2cTransferStatus::kComplete;
}

bool Qmc5883p::StartRead(Reg first, size_t len) {
  const std::array<uint8_t, 1> tx = {std::to_underlying(first)};
  return bus_.StartWriteRead(kAddr, tx, len) == Outcome::kOk;
}

Qmc5883p::BringUpResult Qmc5883p::BringUp() {
  // Retried rather than probed once: the module carries its own regulator and
  // need not be answering the instant this runs.
  bool answered = false;
  for (uint8_t attempt = 0; attempt < kProbeAttempts; ++attempt) {
    if (attempt != 0u) {
      System::GetInstance().Time().DelayMicros(kProbeRetryUs);
    }
    if (!StartRead(Reg::kChipId, 1) ||
        AwaitBus() != I2cTransferStatus::kComplete) {
      continue;
    }
    const std::span<const uint8_t> id = bus_.Received();
    if (!id.empty() && id[0] == kChipId) {
      device_id_ = kDeviceBusTypeI2c |
                   (static_cast<uint32_t>(I2cInstance::kI2c1) << 3) |
                   (static_cast<uint32_t>(kAddr) << 8) |
                   (static_cast<uint32_t>(kChipId) << 16);
      answered = true;
      break;
    }
  }
  if (!answered) {
    return AddressAnswers(kQmc5883lAddr) ? BringUpResult::kUnexpectedPart
                                         : BringUpResult::kAbsent;
  }

  // A firmware flash restarts this code without power-cycling the module, so
  // the part may still be measuring from the previous image. Reset puts it
  // back in Suspend, which is the mode the datasheet wants a configuration
  // change to pass through.
  if (!WriteRegister(Reg::kCtrl2, kCtrl2SoftReset)) {
    return BringUpResult::kRefused;
  }
  System::GetInstance().Time().DelayMicros(kResetSettleUs);

  // Range before mode: control register 2 carries the field range, and the
  // scale this driver applies has to be the one the part is measuring on
  // before the first sample lands.
  const uint8_t ctrl2 = static_cast<uint8_t>(
      (static_cast<uint8_t>(cfg_.range) << 2) | kSetResetOn);
  const uint8_t ctrl1 = static_cast<uint8_t>(
      (static_cast<uint8_t>(cfg_.osr2) << 6) |
      (static_cast<uint8_t>(cfg_.osr1) << 4) |
      (static_cast<uint8_t>(cfg_.odr) << 2) | kModeNormal);

  // An ACK says the byte arrived, not that the register kept it -- and this
  // part will ACK a range it then discards, leaving the driver scaling counts
  // against a range the chip is not measuring on. Everything downstream is
  // derived from these two: the scale from the range, the sample age from the
  // rate.
  for (uint8_t attempt = 0; attempt < kConfigAttempts; ++attempt) {
    if (attempt != 0u) {
      System::GetInstance().Time().DelayMicros(kConfigRetryUs);
    }
    if (!WriteRegister(Reg::kCtrl2, ctrl2) ||
        !WriteRegister(Reg::kCtrl1, ctrl1)) {
      return BringUpResult::kRefused;
    }
    const std::optional<uint8_t> seen_ctrl1 = ReadRegister(Reg::kCtrl1);
    const std::optional<uint8_t> seen_ctrl2 = ReadRegister(Reg::kCtrl2);
    if (!seen_ctrl1 || !seen_ctrl2) {
      return BringUpResult::kRefused;
    }
    if (((*seen_ctrl1 ^ ctrl1) & kCtrl1Mask) == 0u &&
        ((*seen_ctrl2 ^ ctrl2) & kCtrl2Mask) == 0u) {
      // The output registers still hold whatever was measured before this
      // configuration landed, and DRDY still marks it new. Reading the status
      // clears that flag, so the first sample the loop publishes is one taken
      // on the range this driver is about to scale against.
      (void)ReadRegister(Reg::kStatus);
      return BringUpResult::kOk;
    }
  }
  return BringUpResult::kConfigNotKept;
}

void Qmc5883p::Init(const Config &cfg, Bus bus, SharedState &blackboard) {
  if (initialized_) {
    Panic(ErrorCode::Stm32::kMagReinit);
  }
  if (cfg.sample_period_us == 0u) {
    Panic(ErrorCode::Stm32::kMagInitFailed);
  }

  // The remap is a signed permutation. A repeated source axis would drop one
  // component and duplicate another, which reads as a plausible heading that
  // no calibration can correct.
  const Config::AxisMap &axes = cfg.axes;
  if (axes.x_from > 2u || axes.y_from > 2u || axes.z_from > 2u ||
      axes.x_from == axes.y_from || axes.y_from == axes.z_from ||
      axes.x_from == axes.z_from) {
    Panic(ErrorCode::Stm32::kMagInitFailed);
  }

  cfg_ = cfg;
  bus_ = bus;
  blackboard_ = &blackboard;
  microtesla_per_count_ =
      kMicroteslaPerCount[static_cast<size_t>(cfg.range)];

  // A sensor the build expects and the bus cannot find is a wiring fault, not
  // a condition to fly with, and the M10 on the same module is held to the
  // same rule. Which code says what was found, because the panic is all a
  // driver can say at boot.
  switch (BringUp()) {
    case BringUpResult::kOk:
      break;
    case BringUpResult::kAbsent:
      Panic(ErrorCode::Stm32::kMagNotResponding);
    case BringUpResult::kUnexpectedPart:
      Panic(ErrorCode::Stm32::kMagUnexpectedPart);
    case BringUpResult::kRefused:
      Panic(ErrorCode::Stm32::kMagConfigRefused);
    case BringUpResult::kConfigNotKept:
      Panic(ErrorCode::Stm32::kMagConfigNotKept);
  }

  data_.device_id = device_id_;
  initialized_ = true;
}

void Qmc5883p::Poll(uint32_t now_us) {
  if (!initialized_) {
    return;
  }

  if (phase_ != Phase::kIdle) {
    const I2cTransferStatus status = bus_.Poll(now_us);
    if (status == I2cTransferStatus::kBusy) {
      return;
    }
    if (status != I2cTransferStatus::kComplete) {
      // Nothing published and nothing counted here: the bus already recorded
      // why in SystemHealth::sensor_i2c, and leaving the stamp where it was is
      // what tells a reader the compass has stopped answering.
      phase_ = Phase::kIdle;
      return;
    }

    if (phase_ == Phase::kReadingStatus) {
      const std::span<const uint8_t> rx = bus_.Received();
      status_ = rx.empty() ? 0u : rx[0];
      if (!StartRead(Reg::kDataX, kDataLen)) {
        phase_ = Phase::kIdle;
        return;
      }
      phase_ = Phase::kReadingData;
      return;
    }

    phase_ = Phase::kIdle;
    DecodeSample(bus_.Received(), now_us);
    return;
  }

  if (last_start_us_ != 0u &&
      static_cast<uint32_t>(now_us - last_start_us_) < cfg_.sample_period_us) {
    return;
  }

  // Rejected means another tenant's transfer is on the wire. last_start_us_
  // stays put, so the read retries on the next tick.
  if (!StartRead(Reg::kStatus, 1)) {
    return;
  }
  last_start_us_ = now_us;
  phase_ = Phase::kReadingStatus;
}

void Qmc5883p::DecodeSample(std::span<const uint8_t> rx, uint32_t now_us) {
  // Unreachable by the bus's contract -- a kComplete read hands back exactly
  // what was asked for -- and kept because the alternative is indexing a span
  // on the strength of that.
  if (rx.size() < kDataLen) {
    return;
  }

  if ((status_ & kStatusOvfl) != 0u) {
    ++data_.overflow_count;
  }

  // Not a fault: reading below the ODR is the configured behaviour, so landing
  // between conversions is ordinary. The last sample stands with the stamp it
  // already had, which is what keeps its age honest.
  if ((status_ & kStatusDrdy) == 0u) {
    blackboard_->UpdateMagnetometer(data_);
    return;
  }

  const std::array<int16_t, 3> chip = {
      LoadLe16(rx[0], rx[1]),
      LoadLe16(rx[2], rx[3]),
      LoadLe16(rx[4], rx[5]),
  };
  const auto body = [&](uint8_t from, bool negate) {
    const float value = static_cast<float>(chip[from]) * microtesla_per_count_;
    return negate ? -value : value;
  };

  data_.x = body(cfg_.axes.x_from, cfg_.axes.x_neg);
  data_.y = body(cfg_.axes.y_from, cfg_.axes.y_neg);
  data_.z = body(cfg_.axes.z_from, cfg_.axes.z_neg);
  data_.timestamp_us = now_us;
  data_.valid = true;
  ++data_.sample_count;
  blackboard_->UpdateMagnetometer(data_);
}
