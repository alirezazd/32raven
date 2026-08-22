// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once
#include "mbedtls/sha256.h"  // IWYU pragma: keep
#include "message.hpp"
#include "state_machine.hpp"
#include "timebase.hpp"

struct AppContext;

// Peer frame counters wrap, so only equality is ever asked of them.
class ActivityWatch {
 public:
  void Reset(uint32_t counters) { seen_ = counters; }
  bool Advanced(uint32_t counters) {
    if (counters == seen_) {
      return false;
    }
    seen_ = counters;
    return true;
  }

 private:
  uint32_t seen_ = 0;
};

// Paces a request the peer may never answer.
class RetryCadence {
 public:
  // Counts the request the caller is about to send as the first attempt.
  void Begin(uint32_t now_ms) {
    last_ms_ = now_ms;
    attempts_ = 1;
  }
  void Clear() { attempts_ = 0; }
  bool Due(uint32_t now_ms, uint32_t period_ms) const {
    return (now_ms - last_ms_) >= period_ms;
  }
  bool Exhausted(uint16_t limit) const { return attempts_ >= limit; }
  void Sent(uint32_t now_ms) {
    last_ms_ = now_ms;
    ++attempts_;
  }

 private:
  uint32_t last_ms_ = 0;
  uint16_t attempts_ = 0;
};

class ServingState : public IState<AppContext> {
 public:
  const char *Name() const override { return "Serving"; }

  void OnEnter(AppContext &ctx) override;
  void OnStep(AppContext &ctx) override;
};

class MavlinkWifiState : public IState<AppContext> {
 public:
  const char *Name() const override { return "MavlinkWifi"; }

  void OnEnter(AppContext &ctx) override;
  void OnStep(AppContext &ctx) override;
};

class MavlinkUsbState : public IState<AppContext> {
 public:
  const char *Name() const override { return "MavlinkUsb"; }

  void OnEnter(AppContext &ctx) override;
  void OnStep(AppContext &ctx) override;
};

class ServiceState : public IState<AppContext> {
 public:
  const char *Name() const override { return "Service"; }

  void OnEnter(AppContext &ctx) override;
  void OnStep(AppContext &ctx) override;
};

class ProgramState : public IState<AppContext> {
 public:
  const char *Name() const override { return "Program"; }

  void OnEnter(AppContext &ctx) override;
  void OnStep(AppContext &ctx) override;

};

class EscConfigState : public IState<AppContext> {
 public:
  const char *Name() const override { return "EscConfig"; }

  void OnEnter(AppContext &ctx) override;
  void OnStep(AppContext &ctx) override;

 private:

  bool warned_armed_ = false;
  bool stream_seen_ = false;
  ActivityWatch activity_{};
  RetryCadence grant_{};
};

// AP up, TCP server serving the LOG verbs. A LOG LIST or LOG GET hands off
// to LogPullState, which returns here when the transfer ends either way.
class WifiLogState : public IState<AppContext> {
 public:
  const char *Name() const override { return "WifiLog"; }

  void OnEnter(AppContext &ctx) override;
  void OnStep(AppContext &ctx) override;
};

// Streams one log (or the listing) from the STM32 to the TCP client. One
// request in flight, host-paced: the STM32 serves a frame per request,
// forwarded before the next is asked.
class LogPullState : public IState<AppContext> {
 public:
  const char *Name() const override { return "LogPull"; }

  void PrepareList();
  void PrepareGet(const char *name);
  void OnListReply(const message::LogListReplyMsg &reply);
  void OnData(const message::LogDataMsg &data);

  void OnEnter(AppContext &ctx) override;
  void OnStep(AppContext &ctx) override;

 private:
  enum class Op : uint8_t { kList, kGet };

  void SendRequest(AppContext &ctx);
  void Finish(AppContext &ctx, const char *ctrl_line);

  Op op_ = Op::kList;
  char name_[13] = {};
  uint32_t offset_ = 0;
  uint8_t list_first_ = 0;

  bool reply_pending_ = false;
  bool done_ = false;
  ActivityWatch activity_{};
  RetryCadence reply_{};
  uint32_t last_progress_ms_ = 0;

  // The forwarded-but-unsent tail of the last kLogData, kept until the data
  // socket accepts it; TCP backpressure is the flow control.
  message::LogDataMsg chunk_{};
  uint16_t chunk_sent_ = 0;
  bool chunk_valid_ = false;

  mbedtls_sha256_context sha_{};
  uint32_t total_bytes_ = 0;

  uint16_t rx_frames_ = 0;
  uint16_t tx_frames_ = 0;

  AppContext *ctx_ = nullptr;
};

// The MSC session: the STM32 hands the card to the host as a disk and
// reports block traffic through kUsbStatus.
class UsbLogState : public IState<AppContext> {
 public:
  const char *Name() const override { return "UsbLog"; }

  void OnEnter(AppContext &ctx) override;
  void OnStep(AppContext &ctx) override;

 private:

  bool stream_seen_ = false;
  ActivityWatch activity_{};
  RetryCadence grant_{};
};
