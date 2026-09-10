// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "host_link.hpp"
#include "ring_buffer.hpp"

// HostLink over WiFi: a line-oriented ctrl socket and a binary data
// socket, one client each, the data one only inside a ctrl session.
class TcpServer final : public HostLink {
 public:
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

  void Poll() override;
  void SendCtrlLine(const char *line) override;
  [[nodiscard]] size_t ReadDataRx(std::span<uint8_t> dst) override;
  // Send raw data to DATA client
  int SendData(const uint8_t *data, size_t len);

 private:
  friend class System;

  // Out of line so the instance in GetInstance cannot constant-initialize:
  // a few non-zero members would drag the whole object into .data.
  TcpServer();
  ~TcpServer() override = default;
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

  void DiscardDataRx() override;
  // len always fits: the pump stages at most DataRxFree() while the sink is
  // open, and a closed sink drains-and-drops here.
  void StageDataRx(const uint8_t *data, size_t len);
  size_t DataRxFree() const;

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

  // Where the image waits between the data socket and the programmer, sized
  // to swallow one whole pump cycle. Below that, flow control engages on
  // every pump instead of only under real backpressure; PumpDataRx
  // static_asserts against it.
  static constexpr size_t kDataRxCap = 4096;
  RingBuffer<uint8_t, kDataRxCap + 1> data_rx_;
};
