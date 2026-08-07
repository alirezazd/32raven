// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "esc_bootloader.hpp"

#include "dshot_codec.hpp"
#include "error_code.hpp"
#include "gpio.hpp"
#include "panic.hpp"
#include "stm32_config.hpp"

namespace {

// Eight zero bytes to let the bootloader's auto-baud settle, then the literal
// greeting it matches on. Sent without a CRC: there is no connection yet, and
// the bootloader is not listening for one until there is.
constexpr uint8_t kWake[] = {0,   0,   0,   0,   0,   0,   0,    0,   0x0D,
                             'B', 'L', 'H', 'e', 'l', 'i', 0xF4, 0x7D};

// "471x" then signature hi, signature lo, boot version, boot pages.
constexpr size_t kBootInfoBytes = 8;
constexpr uint8_t kBootMsg[] = {'4', '7', '1'};

// The bootloader's own ACK vocabulary, distinct from the four-way acks the
// host sees.
constexpr uint8_t kBlbSuccess = 0x30;

// AM32 and BLHeli_32 are ARM parts, and imARM_BLB is the only mode this
// firmware offers; the host is told so in the fourth DeviceInfo byte.
constexpr uint8_t kInterfaceModeArmBlb = 4;

const board::BoardPin *MotorPin(uint8_t index) {
  switch (index) {
    case 0:
      return &board::kDshotMotor1;
    case 1:
      return &board::kDshotMotor2;
    case 2:
      return &board::kDshotMotor3;
    case 3:
      return &board::kDshotMotor4;
    default:
      return nullptr;
  }
}

}  // namespace

void EscBootloader::Init(UartSoft &uart) {
  if (initialized_) {
    Panic(ErrorCode::Stm32::kEscBootloaderReinit);
  }
  initialized_ = true;
  uart_ = &uart;
}

bool EscBootloader::SendWake() {
  uart_->Send(kWake, sizeof(kWake));
  return true;
}

bool EscBootloader::ReadBootInfo(DeviceInfo &out) {
  uint8_t info[kBootInfoBytes] = {};
  if (uart_->ReadBlock(info, kBootInfoBytes) != kBootInfoBytes) {
    return false;
  }

  uint8_t ack = 0;
  if (!uart_->Read(ack) || ack != kBlbSuccess) {
    return false;
  }

  // Only the first three characters are checked. The fourth varies by
  // bootloader build and is reported to the host rather than validated.
  for (size_t i = 0; i < sizeof(kBootMsg); ++i) {
    if (info[i] != kBootMsg[i]) {
      return false;
    }
  }

  out.signature_lo = info[5];
  out.signature_hi = info[4];
  out.boot_version = info[3];
  out.interface_mode = kInterfaceModeArmBlb;
  return true;
}

bool EscBootloader::Connect(uint8_t motor_index, DeviceInfo &out) {
  const board::BoardPin *pin = MotorPin(motor_index);
  if (pin == nullptr) {
    return false;
  }

  // A previous session on another motor leaves that pin borrowed.
  Disconnect();

  uart_->Open(pin->port, pin->pin, pin->af);
  motor_index_ = motor_index;

  const bool ok = SendWake() && ReadBootInfo(out);
  if (!ok) {
    ++connect_fail_count_;
    Disconnect();
    return false;
  }

  connected_ = true;
  return true;
}

void EscBootloader::Disconnect() {
  if (!uart_->IsOpen()) {
    connected_ = false;
    return;
  }

  // Close restores the alternate function it was opened with, so the pin goes
  // back to TIM1 without this having to know how DShot programmed it.
  uart_->Close();
  connected_ = false;
}
