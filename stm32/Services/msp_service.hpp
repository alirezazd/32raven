// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include <cstddef>
#include <cstdint>

#include "esc_service.hpp"
#include "four_way_service.hpp"
#include "shared_state.hpp"
#include "usb_cdc.hpp"

// MSP responder on the USB CDC port. Configurators probe with MSP before they
// admit a board exists.
//
// Both framings are accepted because tools disagree on which they send:
//   v1  '$' 'M' '<' len cmd payload... xor
//   v2  '$' 'X' '<' flag cmd16 len16 payload... crc8_dvb_s2
// Each request is answered in its own dialect; most parsers silently drop the
// other.

class MspService {
 public:
  static MspService &GetInstance();

  struct Config {
    const char *board_identifier;  // exactly 4 chars, e.g. "RAVN"
    const char *board_name;
    const char *manufacturer_id;
    const char *craft_name;
    // Two views of one tick, so both are folded from kControlLoopHz.
    uint16_t loop_rate_hz;
    uint16_t loop_period_us;
  };

  void Poll(uint32_t now_us);

  // The one writer of SharedState::usb_: this service already holds the CDC
  // driver and the four-way service, so it is where all three sources meet
  // without something reaching sideways. Stamps only on a real change.
  //
  // Public for the state that owned the session to call on its way out: Poll
  // is the only other caller and it stops running when that state does, so
  // the last record published would otherwise be the pre-teardown one -- a
  // port still claimed by a host that has gone.
  void PublishUsbStatus(uint32_t now_us);

  // Latched: the ESP32 reconciles it against its own state, having been told
  // what this side believes via kUsbStatus.
  void SetEscConfigMode(bool enabled);
  bool EscConfigGranted() const { return esc_config_granted_; }

  uint32_t RequestCount() const { return request_count_; }
  uint32_t ReplyCount() const { return reply_count_; }
  uint32_t CrcErrorCount() const { return crc_error_count_; }
  uint32_t UnknownCommandCount() const { return unknown_command_count_; }
  uint32_t TxDropCount() const { return tx_drop_count_; }
  uint16_t LastCommand() const { return last_command_; }

 private:
  friend class System;
  void Init(const Config &cfg, UsbCdc &usb, SharedState &blackboard,
            FourWayService &four_way, EscService &esc);

  MspService() = default;
  ~MspService() = default;
  MspService(const MspService &) = delete;
  MspService &operator=(const MspService &) = delete;

  static constexpr size_t kMaxPayload = 256;

  // '$', 'M', direction, size, command -- and v2's longer '$', 'X', direction,
  // flags, command x2, length x2. Replies are staged past the longer of the
  // two so the payload is never copied; a v1 frame simply starts three bytes
  // in and is sent from there.
  static constexpr size_t kV1HeaderBytes = 5;
  static constexpr size_t kV2HeaderBytes = 8;

  enum class Parse : uint8_t {
    kIdle,
    kVersion,
    kDirection,
    kV1Size,
    kV1Command,
    kV1Payload,
    kV1Checksum,
    kV2Flag,
    kV2CommandLo,
    kV2CommandHi,
    kV2SizeLo,
    kV2SizeHi,
    kV2Payload,
    kV2Checksum,
  };

  void Feed(uint8_t byte);
  void Dispatch();

  // Fills reply_ from the current request. False answers with the error
  // direction character, so an unsupported command is not a timeout.
  bool BuildReply(uint16_t command);

  bool SendReply(bool ok);
  void RevokeEscConfigMode();
  void Reset() { parse_ = Parse::kIdle; }

  // MSP is little-endian on the wire regardless of host byte order.
  uint8_t *ReplyBuf() { return &frame_[kV2HeaderBytes]; }
  void Push8(uint8_t v);
  void Push16(uint16_t v);
  void Push32(uint32_t v);
  void PushString(const char *s);
  void PushLengthPrefixedString(const char *s);

  Config cfg_{};
  UsbCdc *usb_ = nullptr;
  FourWayService *four_way_ = nullptr;
  EscService *esc_ = nullptr;
  SharedState *blackboard_ = nullptr;
  bool initialized_ = false;

  // MSP_SET_PASSTHROUGH is answered in MSP framing before the port changes
  // hands, so the handover is deferred until the reply is sent.
  bool passthrough_pending_ = false;

  uint32_t last_usb_reset_ = 0;
  bool last_usb_connected_ = false;
  bool esc_config_granted_ = false;

  Parse parse_ = Parse::kIdle;
  bool is_v2_ = false;
  uint16_t command_ = 0;
  uint16_t payload_size_ = 0;
  uint16_t payload_index_ = 0;
  uint8_t v2_flag_ = 0;
  uint8_t checksum_ = 0;
  uint8_t payload_[kMaxPayload]{};

  uint16_t reply_len_ = 0;
  bool reply_overflow_ = false;

  uint8_t frame_[UsbCdc::kMaxFrameBytes]{};
  static_assert(sizeof(frame_) >= kV2HeaderBytes + kMaxPayload + 1u);
  // A reply the ring cannot take whole is dropped, not truncated, so the ring
  // is sized from this buffer rather than from the largest frame seen so far.
  static_assert(sizeof(frame_) <= UsbCdc::kTxRingSize - 1);

  uint32_t request_count_ = 0;
  uint32_t reply_count_ = 0;
  uint32_t crc_error_count_ = 0;
  uint32_t unknown_command_count_ = 0;
  uint32_t tx_drop_count_ = 0;
  uint16_t last_command_ = 0;
};
