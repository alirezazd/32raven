// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include <cstdint>
#include <span>

#include "uart_soft.hpp"

// BLHeli's bootloader protocol ("BLB"), spoken down a motor signal wire at
// 19200 8N1. FourWayService is the other half, parsing the host's USB frames;
// the command names line up between the two but nothing else does, so a
// four-way cmd_DeviceRead becomes a BLB set-address plus a BLB read.
//
// Framing once connected: command bytes, CRC16, then the reply, its CRC16 and
// an ACK. Before that the bootloader neither expects nor sends a CRC, which is
// why Connect is not just another command.
class EscBootloader {
 public:
  // Four bytes, in the order the four-way host expects them back from
  // cmd_DeviceInitFlash.
  struct __attribute__((packed)) DeviceInfo {
    uint8_t signature_lo;
    uint8_t signature_hi;
    uint8_t boot_version;
    uint8_t interface_mode;
  };

  void Init(UartSoft &uart);

  // False means no ESC answered: unpowered, absent, or running firmware rather
  // than sitting in its bootloader. The caller must have stopped DShot and the
  // flight cascade first -- the pin belongs to TIM1 until this claims it, and a
  // byte costs 520 us with interrupts masked.
  bool Connect(uint8_t motor_index, DeviceInfo &out);

  void HoldAll();
  void ReleaseAll();

  enum class VerifyResult : uint8_t { kOk, kMismatch, kFailed };

  // Every AM32 target links its firmware at 0x08001000, reserving the four 1 KB
  // pages below it for the bootloader. Erasing one of those is the only failure
  // on this interface that a second attempt cannot undo -- it destroys the
  // thing the second attempt would arrive through.
  static constexpr uint8_t kFirstErasablePage = 4;
  static constexpr uint16_t kFirstWritableAddress = kFirstErasablePage * 1024u;

  bool ReadFlash(uint16_t address, uint8_t *out, uint16_t len);
  bool PageErase(uint8_t page);
  bool WriteFlash(uint16_t address, std::span<const uint8_t> bytes);
  VerifyResult VerifyFlash(uint16_t address, std::span<const uint8_t> bytes);

  bool KeepAlive();
  bool Reset(uint8_t motor_index, bool reboot);

  void Disconnect();

  bool IsConnected() const { return connected_; }
  uint8_t SelectedMotor() const { return motor_index_; }

  uint32_t ConnectFailCount() const { return connect_fail_count_; }

  static constexpr uint16_t kMaxTransferBytes = 256;

 private:
  bool SendWake();
  bool ReadBootInfo(DeviceInfo &out);
  bool SendCommand(std::span<const uint8_t> cmd);
  bool SendPayload(std::span<const uint8_t> bytes);
  bool ReadFramed(uint8_t *out, uint16_t len);
  uint8_t ReadAck(uint16_t attempts);
  bool SetAddress(uint16_t address);
  bool SetBuffer(std::span<const uint8_t> bytes);

  UartSoft *uart_ = nullptr;
  bool initialized_ = false;
  bool connected_ = false;
  bool held_ = false;
  uint8_t motor_index_ = 0;

  uint32_t connect_fail_count_ = 0;
};
