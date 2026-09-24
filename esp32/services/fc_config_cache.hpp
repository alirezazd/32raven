// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include <cstdint>
#include <optional>

#include "error_code.hpp"
#include "fc_link.hpp"
#include "message.hpp"

// The flight computer's RC map, board trim and calibration ids as last
// reported, plus the writes a ground station asked for that the flight
// computer has not yet echoed back. Nothing MAVLink here: the parameter
// server reads and writes records, and this gets them across the link.
class FcConfigCache {
 public:
  enum class Record : uint8_t {
    kRcMap,
    kGyroCalibrationId,
    kMagCalibrationId,
    kBoardTrim,
  };

  void Init(FcLink &fc_link);
  // Writes in flight are dropped: the link that asked for them is gone.
  void AbandonWrites();
  // Retries due writes. One out of attempts panics -- a map the flight
  // computer will not take is not a state to keep flying in.
  void Poll(uint32_t now_ms);

  // A record arriving from the flight computer settles any write waiting on
  // it. An invalid record panics: the peer is the authority on these.
  void Adopt(const message::RcMapConfigMsg &cfg);
  void Adopt(const message::CalibrationIdConfigMsg &cfg);
  void Adopt(const message::BoardTrimConfigMsg &cfg);

  const std::optional<message::RcMapConfigMsg> &RcMap() const {
    return rc_map_.value;
  }
  const std::optional<message::CalibrationIdConfigMsg> &GyroCalibrationId()
      const {
    return gyro_calibration_id_.value;
  }
  const std::optional<message::CalibrationIdConfigMsg> &MagCalibrationId()
      const {
    return mag_calibration_id_.value;
  }
  const std::optional<message::BoardTrimConfigMsg> &BoardTrim() const {
    return board_trim_.value;
  }

  // Held and not mid-write. A record not held is asked for, rate-limited;
  // the caller tries again next poll.
  bool Available(Record record, uint32_t now_ms);
  // The record a request has gone unanswered for long enough to call a
  // fault, once, by name; null otherwise. That request is not sent again:
  // the record stays unavailable until an answer arrives on its own.
  const char *TakeStalled();

  std::optional<message::RcMapConfigMsg> RcMapWriteBase() const {
    return rc_map_.WriteBase();
  }
  std::optional<message::BoardTrimConfigMsg> BoardTrimWriteBase() const {
    return board_trim_.WriteBase();
  }
  // Sent at the next Poll and resent until the flight computer echoes it.
  void WriteRcMap(const message::RcMapConfigMsg &value);
  void WriteBoardTrim(const message::BoardTrimConfigMsg &value);

 private:
  struct RequestState {
    uint32_t next_request_ms = 0;
    bool waiting = false;
    uint16_t sent = 0;
  };

  // One record. Nothing writes a calibration id, so those slots' `write`
  // stays empty for the life of the board and every path through it no-ops.
  template <typename T>
  struct Slot {
    struct PendingWrite {
      T desired{};
      // Absent until the first send, which is due at once.
      std::optional<uint32_t> next_retry_ms{};
      uint16_t attempts_remaining = 0;
    };

    // Take the reported value and drop a write it matches.
    void Settle(const T &cfg);

    void StartWrite(const T &desired, uint16_t attempts) {
      write = PendingWrite{desired, std::nullopt, attempts};
    }

    // What a read-modify-write edits: the write in flight if there is one,
    // otherwise the record as last reported.
    std::optional<T> WriteBase() const {
      return write.has_value() ? std::optional<T>{write->desired} : value;
    }

    // The value to send when a retry is due, nothing otherwise. Out of
    // attempts panics with `failed`.
    std::optional<T> Retry(uint32_t now_ms, ErrorCode::Esp32 failed);

    std::optional<T> value{};
    RequestState request{};
    std::optional<PendingWrite> write{};
  };

  bool Held(Record record) const;
  bool WritePending(Record record) const;
  void Request(Record record, uint32_t now_ms);
  void SendRequest(RequestState &state, const message::Packet &request,
                   const char *description, uint32_t now_ms);

  FcLink *fc_link_ = nullptr;
  const char *stalled_ = nullptr;
  Slot<message::RcMapConfigMsg> rc_map_{};
  Slot<message::CalibrationIdConfigMsg> gyro_calibration_id_{};
  Slot<message::CalibrationIdConfigMsg> mag_calibration_id_{};
  Slot<message::BoardTrimConfigMsg> board_trim_{};
};
