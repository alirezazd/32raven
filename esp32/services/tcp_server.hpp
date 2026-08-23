// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

// tcp_server.hpp
#pragma once
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

class TcpServer {
 public:
  // Longest control line accepted. The longest legitimate one is BEGIN with
  // every key at maximum -- ten-digit size and crc, a 31-character target --
  // which comes to 75 bytes. The rest is headroom, not a tight bound.
  static constexpr size_t kMaxLineBytes = 160;

  static TcpServer &GetInstance();

  struct Config {
    uint16_t ctrl_port = 9000;
    uint16_t data_port = 9001;

    // Keepalive (0 disables)
    int keepalive_idle_s = 10;
    int keepalive_intvl_s = 5;
    int keepalive_cnt = 3;
  };

  // Failure is tolerated by design; Running() is the queryable fact.
  void Start();
  void Stop();

  bool Running() const { return running_; }

  // Tick entry point (non-blocking)
  void Poll();

  enum class EventId : uint8_t {
    kNone = 0,
    kBegin,
    kAbort,
    kReset,
    kBridge,  // Explicit command to enable transparent bridge
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
  };

  std::optional<Event> PopEvent();

  // Socket drops since the last call, latched so a drop between polls is
  // never missed. Take once per tick, after draining PopEvent, so a command
  // and a drop landing in the same tick resolve to the drop.
  struct LinkDrops {
    bool ctrl = false;
    bool data = false;
  };
  LinkDrops TakeLinkDrops();

  // The inbound ring has one gate and two lifecycles on it: Open/CloseDataRx
  // switch the raw sink (bridge) and leave Status alone -- a finished
  // transfer's Status must survive the return to the service page --
  // Begin/EndTransfer wrap a sized download and own its Status.
  void OpenDataRx();
  void CloseDataRx();
  void BeginTransfer(size_t total_size);
  void EndTransfer();
  bool DataRxOpen() const { return data_rx_open_; }

  // Binary bytes buffered from DATA socket
  [[nodiscard]] size_t ReadDataRx(std::span<uint8_t> dst);

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

  void SetStatus(const Status &s);
  Status GetStatus() const;

  void SendCtrlLine(const char *line);

  // Send raw data to DATA client
  int SendData(const uint8_t *data, size_t len);

 private:
  friend class System;
  // Out of line so the instance in GetInstance cannot constant-initialize:
  // a few non-zero members would drag the whole object into .data.
  TcpServer();
  ~TcpServer() = default;
  TcpServer(const TcpServer &) = delete;
  TcpServer &operator=(const TcpServer &) = delete;

  void Init(const Config &cfg);

  void CloseCtrl();
  void CloseData();
  void CloseAll();

  void AcceptCtrl();
  void AcceptData();

  void PumpCtrlRx();
  void PumpDataRx();

  // Control command handling (ASCII lines)
  void HandleLine(const char *line);

  // Queues/buffers
  [[nodiscard]] bool PushEvent(const Event &e);
  // Push-and-OK for the payload-free verbs; BEGIN and LOG answer from the
  // page that pops them.
  void QueueSimpleCommand(EventId id);
  void StageDataRx(const uint8_t *data, size_t len);

  enum class LineFeed : uint8_t { kIncomplete, kComplete, kTruncated };
  LineFeed LinebufAdd(char c);

  // Socket helpers
  static bool SetNonblock(int fd);
  static void SetKeepalive(int fd, const Config &cfg);
  static std::optional<int> MakeListenSocket(uint16_t port);

  Config cfg_{};
  bool running_ = false;

  // ctrl is the session anchor; data only lives inside a ctrl session.
  int ctrl_listen_fd_ = -1;
  int data_listen_fd_ = -1;
  int ctrl_fd_ = -1;
  int data_fd_ = -1;

  uint32_t ctrl_peer_ipv4_ = 0;

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
  Event evt_q_[kEvtCap]{};
  size_t evt_head_ = 0;
  size_t evt_tail_ = 0;

  LinkDrops link_drops_{};

  // Inbound staging for the firmware download, drained a chunk per app tick and
  // sized to swallow one whole PumpDataRx cycle. Below that, flow control
  // engages on every pump instead of only under real backpressure; the pump
  // static_asserts against this.
  static constexpr size_t kDataRxCap = 4096;
  uint8_t data_rx_[kDataRxCap]{};
  size_t data_rx_head_ = 0;
  size_t data_rx_tail_ = 0;
  bool data_rx_open_ = false;

  Status status_{};
};
