// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "ring_buffer.hpp"

// This end of the link to the host tool, as FcLink is this end of the link to
// the flight computer: ASCII verbs in, a sized image in, status lines out.
// The transport is the subclass -- TCP hands out two sockets, USB one byte
// stream -- and this owns everything that does not depend on it: the line
// assembler and verb parser, the command queue and the transfer status. Where
// the image waits on its way to the programmer is the transport's too.
class HostLink {
 public:
  // Longest control line accepted. The longest legitimate one is BEGIN with
  // every key at maximum -- ten-digit size and crc, a 31-character target --
  // which comes to 75 bytes. The rest is headroom, not a tight bound.
  static constexpr size_t kMaxLineBytes = 160;

  enum class EventId : uint8_t {
    kNone = 0,
    kBegin,
    kAbort,
    kReset,
    kLogList,
    kLogGet,
  };

  struct BeginArgs {
    uint32_t size = 0;
    uint32_t crc = 0;
    char target[8] = {};  // wire token verbatim ("esp32", "stm32", or empty)
  };

  struct Event {
    EventId id = EventId::kNone;
    BeginArgs begin{};
    char log_name[13] = {};  // kLogGet only, 8.3 + terminator
    // The link that queued it, stamped by PushEvent: the answer goes back
    // the way the command came, and a mode draining two links cannot mix
    // them up.
    HostLink *origin = nullptr;
  };

  // Status snapshot (App updates, client may query via STATUS?)
  struct Status {
    // Values for `state`. On the wire, so tools/esp32_client.py mirrors them:
    // rx counts written bytes while kWriting and verified bytes while
    // kVerifying, and the host restarts its progress at the switch.
    static constexpr uint32_t kWriting = 0;
    static constexpr uint32_t kDone = 1;
    static constexpr uint32_t kVerifying = 2;
    uint32_t rx = 0;
    uint32_t total = 0;
    uint32_t state = kWriting;
    uint32_t err = 0;
  };

  virtual ~HostLink() = default;
  HostLink(const HostLink &) = delete;
  HostLink &operator=(const HostLink &) = delete;

  // Tick entry point (non-blocking)
  virtual void Poll() = 0;
  virtual void SendCtrlLine(const char *line) = 0;

  std::optional<Event> PopEvent();
  // A link drop since the last call, latched so one between polls is never
  // missed. Take once per tick, after draining PopEvent, so a command and a
  // drop landing in the same tick resolve to the drop.
  bool TakeLinkDrop();
  // For a mode with no transfer to abort: a drop that happened while the
  // host sat idle must not surface as one the moment a transfer starts.
  void ClearLinkDrop();

  // CloseDataRx shuts the sink and leaves Status alone -- a finished
  // transfer's Status must survive the return to the service mode --
  // Begin/EndTransfer wrap a sized download and own its Status.
  void CloseDataRx();
  void BeginTransfer(const BeginArgs &begin);
  void EndTransfer();
  bool DataRxOpen() const { return data_rx_open_; }
  // The BEGIN that armed the transfer: size and crc are what the image is
  // checked against.
  const BeginArgs &Begin() const { return begin_; }

  // Image bytes the transport holds, drained a chunk per app tick.
  [[nodiscard]] virtual size_t ReadDataRx(std::span<uint8_t> dst) = 0;

  void SetStatus(const Status &s);
  Status GetStatus() const;

 protected:
  HostLink() = default;

  // One byte of the control channel. A completed line is parsed and either
  // queued as an event or answered.
  void FeedCtrl(uint8_t byte);
  // Whatever image bytes the transport holds are stale: the sink closed, or
  // a transfer began, ended or was reset.
  virtual void DiscardDataRx() = 0;
  void MarkDrop();
  // A new control peer starts mid-nothing, not mid-line.
  void ResetLine();
  void ResetSession();

 private:
  void HandleLine(const char *line);
  [[nodiscard]] bool PushEvent(const Event &e);
  // Push-and-OK for the payload-free verbs; BEGIN and LOG answer from the
  // mode that pops them.
  void QueueSimpleCommand(EventId id);
  enum class LineFeed : uint8_t { kIncomplete, kComplete, kTruncated };
  LineFeed LinebufAdd(char c);

  // One past the longest line, for the NUL written before HandleLine sees it.
  char line_buf_[kMaxLineBytes + 1]{};
  size_t line_len_ = 0;
  // Saturated line, reported once at its newline so resync is free.
  bool line_overflow_ = false;
  // Host commands queued while the loop services the previous one. One
  // control connection issues one verb per line, so the depth is rarely past
  // one; overflow answers ERR evt_queue_full rather than dropping, so this
  // trades a little RAM against a spurious error.
  static constexpr size_t kEvtCap = 8;
  RingBuffer<Event, kEvtCap + 1> evt_q_;
  bool link_dropped_ = false;
  bool data_rx_open_ = false;
  BeginArgs begin_{};
  Status status_{};
};
