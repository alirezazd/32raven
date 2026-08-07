// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include <cstddef>
#include <cstdint>

#include "uart_soft.hpp"

// The ESC half of ESC configuration: BLHeli's bootloader protocol ("BLB"),
// spoken down a motor signal wire at 19200 8N1.
//
// Separate from FourWayService on purpose. That one parses the host's frames
// over USB; this one talks to an ESC over a borrowed DShot pin. Two wires, two
// grammars, and only the command names line up -- a four-way cmd_DeviceRead
// becomes a BLB set-address followed by a BLB read.
//
// Framing after a connection exists: command bytes out, CRC16 out, then the
// reply, its CRC16, and an ACK. Before a connection exists the bootloader
// neither expects nor sends a CRC, which is why Connect is not just another
// command.
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

  // Claims the motor pin, wakes the bootloader, and reads back its signature.
  // False means no ESC answered: unpowered, absent, or already running
  // firmware rather than sitting in the bootloader.
  //
  // The caller must have stopped DShot and the flight cascade first. A byte
  // takes 520 us with interrupts masked and the pin belongs to TIM1 until this
  // takes it.
  bool Connect(uint8_t motor_index, DeviceInfo &out);

  // Returns the pin to TIM1. Safe to call when not connected.
  void Disconnect();

  bool IsConnected() const { return connected_; }
  uint8_t SelectedMotor() const { return motor_index_; }

  uint32_t ConnectFailCount() const { return connect_fail_count_; }

 private:
  bool SendWake();
  bool ReadBootInfo(DeviceInfo &out);

  UartSoft *uart_ = nullptr;
  bool initialized_ = false;
  bool connected_ = false;
  uint8_t motor_index_ = 0;

  uint32_t connect_fail_count_ = 0;
};
