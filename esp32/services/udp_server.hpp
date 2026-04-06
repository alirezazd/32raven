#pragma once

#include <cstddef>
#include <cstdint>

extern "C" {
#include "esp_err.h"
}

class UdpServer {
 public:
  struct Config {
    uint16_t port = 14550;
    bool nonblocking = true;
  };

  static UdpServer &GetInstance() {
    static UdpServer instance;
    return instance;
  }

  esp_err_t Start();
  void Stop();
  void ClearPeer();

  bool Running() const { return running_; }
  bool HasPeer() const { return peer_port_ != 0; }

  int Receive(uint8_t *dst, size_t max_len);
  int Send(const uint8_t *data, size_t len);

 private:
  friend class System;
  void Init(const Config &cfg);
  static bool SetNonblock(int fd);

  Config cfg_{};
  int fd_ = -1;
  bool running_ = false;
  uint32_t peer_ipv4_ = 0;
  uint16_t peer_port_ = 0;

  UdpServer() = default;
  ~UdpServer() = default;
  UdpServer(const UdpServer &) = delete;
  UdpServer &operator=(const UdpServer &) = delete;
};
