// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include <cstddef>
#include <cstdint>

#include "esc_bootloader.hpp"
#include "usb_cdc.hpp"

// BLHeli four-way interface, spoken once MSP hands the port over. The flight
// controller becomes a relay: the host addresses one ESC at a time.
//
// Framing is fixed-shape rather than delimited:
//   request   0x2F cmd addr_hi addr_lo len params[len] crc_hi crc_lo
//   response  0x2E cmd addr_hi addr_lo len params[len] ack crc_hi crc_lo
// A len byte of 0 means 256 parameters, not zero. CRC is CRC16-XMODEM over
// every preceding byte, transmitted high byte first.
//
// Interface commands (0x30-0x34) are answered here. Device commands get a
// failure ack until the ESC link exists, so a configurator sees a refusal
// rather than a hang.

class FourWayService {
 public:
  void Init(UsbCdc &usb, EscBootloader &bootloader);

  // While active the CDC byte stream belongs to this parser, not MSP's.
  void Enter();

  // Forced exit when the grant is withdrawn. Unlike cmd_InterfaceExit,
  // nothing is acknowledged -- the host did not ask.
  void Exit();
  bool IsActive() const { return active_; }

  void Feed(uint8_t byte);

  // A host that stops mid-frame leaves the parser waiting for bytes that never
  // arrive, and nothing announces it: the next session's frame start is then
  // swallowed as this one's payload. MspService ticks this.
  void Poll(uint32_t now_us);

  uint32_t RequestCount() const { return request_count_; }
  uint32_t ReplyCount() const { return reply_count_; }
  uint32_t CrcErrorCount() const { return crc_error_count_; }
  uint8_t LastCommand() const { return last_command_; }
  uint32_t TxDropCount() const { return tx_drop_count_; }
  uint32_t StallCount() const { return stall_count_; }
  uint8_t SelectedEsc() const { return selected_esc_; }

 private:
  static constexpr size_t kMaxParams = 256;

  enum class Parse : uint8_t {
    kEscape,
    kCommand,
    kAddressHi,
    kAddressLo,
    kLength,
    kParams,
    kCrcHi,
    kCrcLo,
  };

  enum class Ack : uint8_t {
    kOk = 0x00,
    kInvalidCommand = 0x02,
    kInvalidCrc = 0x03,
    kVerifyError = 0x04,
    kInvalidChannel = 0x08,
    kInvalidParam = 0x09,
    kDeviceGeneralError = 0x0F,
  };

  // Escape, command, address x2, length -- everything Respond writes ahead of
  // the payload. Replies are staged past it so the payload is never copied.
  static constexpr size_t kHeaderBytes = 5;

  // ...and the ack plus the two CRC bytes that follow it.
  static constexpr size_t kFrameOverhead = kHeaderBytes + 3;

  void Dispatch();
  void Respond(Ack ack);
  void Reset();

  uint8_t *ReplyBuf() { return &frame_[kHeaderBytes]; }

  UsbCdc *usb_ = nullptr;
  EscBootloader *bootloader_ = nullptr;
  bool initialized_ = false;
  bool active_ = false;

  Parse parse_ = Parse::kEscape;
  uint8_t command_ = 0;
  uint16_t address_ = 0;
  uint16_t param_count_ = 0;
  uint16_t param_index_ = 0;
  uint16_t crc_ = 0;
  uint16_t crc_received_ = 0;
  uint8_t params_[kMaxParams]{};
  uint16_t reply_len_ = 0;

  uint8_t frame_[UsbCdc::kMaxFrameBytes]{};
  static_assert(sizeof(frame_) >= kMaxParams + kFrameOverhead);
  // A reply the ring cannot take whole is dropped, not truncated, so the ring
  // is sized from this buffer rather than from the largest frame seen so far.
  static_assert(sizeof(frame_) <= UsbCdc::kTxRingSize - 1);

  uint8_t selected_esc_ = 0;

  // Sampled by Poll rather than stamped in Feed: Feed runs once per received
  // byte, and reading the timer that often costs more than the counter does.
  uint32_t feed_count_ = 0;
  uint32_t last_feed_count_ = 0;
  uint32_t last_progress_us_ = 0;

  uint32_t request_count_ = 0;
  uint32_t reply_count_ = 0;
  uint32_t crc_error_count_ = 0;
  uint32_t tx_drop_count_ = 0;
  uint32_t stall_count_ = 0;
  uint8_t last_command_ = 0;
};
