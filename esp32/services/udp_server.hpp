#pragma once

#include <cstddef>
#include <cstdint>

#include "esp32_limits.hpp"
#include "ring_buffer.hpp"

extern "C" {
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
}

class UdpServer {
 public:
  struct Config {
    uint16_t port = 14550;
    uint32_t upload_cap_kbits = 0;
    uint32_t download_cap_kbits = 0;
    uint32_t overflow_threshold = 16;
  };

  static UdpServer &GetInstance() {
    static UdpServer instance;
    return instance;
  }

  esp_err_t Start();
  void Stop();
  void ClearPeer();

  int Receive(uint8_t *dst, size_t max_len);
  int Send(const uint8_t *data, size_t len);

 private:
  class LockGuard {
   public:
    explicit LockGuard(const UdpServer &server);
    ~LockGuard();
    LockGuard(const LockGuard &) = delete;
    LockGuard &operator=(const LockGuard &) = delete;

   private:
    const UdpServer &server_;
  };

  friend class System;
  void Init(const Config &cfg);
  static bool SetNonblock(int fd);
  static void RefillTokens(uint32_t bytes_per_s, uint32_t burst_bytes,
                           uint32_t &tokens_bytes, int64_t &last_refill_us);
  void Lock() const;
  void Unlock() const;
  void ClearPeerLocked();
  void ResetShaperStateLocked();

  Config cfg_{};
  int fd_ = -1;
  bool running_ = false;
  uint32_t peer_ipv4_ = 0;
  uint16_t peer_port_ = 0;
  mutable StaticSemaphore_t mutex_buffer_{};
  mutable SemaphoreHandle_t mutex_ = nullptr;
  static constexpr uint32_t kUploadBufferBytes =
      static_cast<uint32_t>(esp32_limits::kUdpServerUploadBufferBytes);
  static constexpr uint32_t kDownloadBufferBytes =
      static_cast<uint32_t>(esp32_limits::kUdpServerDownloadBufferBytes);
  bool upload_cap_enabled_ = false;
  uint32_t upload_cap_bytes_per_s_ = 0;
  uint32_t upload_tokens_bytes_ = 0;
  int64_t upload_last_refill_us_ = 0;
  uint32_t upload_overflow_count_ = 0;
  RingBuffer<uint8_t, esp32_limits::kUdpServerUploadBufferBytes + 1>
      upload_shaper_buffer_;
  bool download_cap_enabled_ = false;
  uint32_t download_cap_bytes_per_s_ = 0;
  uint32_t download_tokens_bytes_ = 0;
  int64_t download_last_refill_us_ = 0;
  uint32_t download_overflow_count_ = 0;
  RingBuffer<uint8_t, esp32_limits::kUdpServerDownloadBufferBytes + 1>
      download_shaper_buffer_;

  UdpServer() = default;
  ~UdpServer() = default;
  UdpServer(const UdpServer &) = delete;
  UdpServer &operator=(const UdpServer &) = delete;
};
