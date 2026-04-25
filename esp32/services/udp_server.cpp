#include "udp_server.hpp"

#include <fcntl.h>

#include <algorithm>
#include <cerrno>
#include <cstring>

#include "error_code.hpp"
#include "panic.hpp"
#include "system.hpp"

extern "C" {
#include "esp_log.h"
#include "esp_timer.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
}

static constexpr const char *kTag = "udp_server";

static bool IsWouldBlock(int error) {
  return error == EWOULDBLOCK || error == EAGAIN;
}

void UdpServer::ClearPeerState() {
  peer_ipv4_ = 0;
  peer_port_ = 0;
}

void UdpServer::ResetShaperState() {
  upload_tokens_bytes_ = 0;
  upload_last_refill_us_ = 0;
  upload_overflow_count_ = 0;
  upload_shaper_buffer_.Clear();
  download_tokens_bytes_ = 0;
  download_last_refill_us_ = 0;
  download_overflow_count_ = 0;
  download_shaper_buffer_.Clear();
}

void UdpServer::Init(const Config &cfg) {
  cfg_ = cfg;
  if (cfg_.overflow_threshold == 0) {
    Panic(ErrorCode::kUdpServerInvalidOverflowThreshold);
  }
  upload_cap_enabled_ = cfg_.upload_cap_kbits > 0;
  upload_cap_bytes_per_s_ =
      upload_cap_enabled_ ? ((cfg_.upload_cap_kbits * 1000u) / 8u) : 0;
  if (upload_cap_enabled_ && upload_cap_bytes_per_s_ == 0) {
    upload_cap_bytes_per_s_ = 1;
  }
  download_cap_enabled_ = cfg_.download_cap_kbits > 0;
  download_cap_bytes_per_s_ =
      download_cap_enabled_ ? ((cfg_.download_cap_kbits * 1000u) / 8u) : 0;
  if (download_cap_enabled_ && download_cap_bytes_per_s_ == 0) {
    download_cap_bytes_per_s_ = 1;
  }
  ResetShaperState();
  ClearPeerState();
  running_ = false;
  fd_ = -1;
  ESP_LOGI(kTag, "initialized on port %u", static_cast<unsigned>(cfg_.port));
}

bool UdpServer::SetNonblock(int fd) {
  const int flags = fcntl(fd, F_GETFL, 0);
  if (flags < 0) {
    return false;
  }
  return fcntl(fd, F_SETFL, flags | O_NONBLOCK) == 0;
}

void UdpServer::RefillTokens(uint32_t bytes_per_s, uint32_t burst_bytes,
                             uint32_t &tokens_bytes, int64_t &last_refill_us) {
  const int64_t now_us = esp_timer_get_time();
  if (last_refill_us == 0) {
    last_refill_us = now_us;
  }
  if (now_us <= last_refill_us) {
    return;
  }

  const uint64_t elapsed_us = static_cast<uint64_t>(now_us - last_refill_us);
  const uint64_t refill =
      (elapsed_us * static_cast<uint64_t>(bytes_per_s)) / 1000000ull;
  const uint64_t topped = static_cast<uint64_t>(tokens_bytes) + refill;
  tokens_bytes = static_cast<uint32_t>(
      std::min<uint64_t>(topped, static_cast<uint64_t>(burst_bytes)));
  last_refill_us = now_us;
}

esp_err_t UdpServer::Start() {
  if (running_) {
    return ESP_OK;
  }

  fd_ = static_cast<int>(socket(AF_INET, SOCK_DGRAM, IPPROTO_IP));
  if (fd_ < 0) {
    ESP_LOGE(kTag, "socket() failed: errno=%d", errno);
    return ESP_FAIL;
  }

  int reuse_addr = 1;
  (void)setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &reuse_addr,
                   sizeof(reuse_addr));
  int broadcast = 1;
  (void)setsockopt(fd_, SOL_SOCKET, SO_BROADCAST, &broadcast,
                   sizeof(broadcast));

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(cfg_.port);
  addr.sin_addr.s_addr = htonl(INADDR_ANY);

  if (bind(fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
    ESP_LOGE(kTag, "bind(%u) failed: errno=%d",
             static_cast<unsigned>(cfg_.port), errno);
    close(fd_);
    fd_ = -1;
    return ESP_FAIL;
  }

  if (!SetNonblock(fd_)) {
    ESP_LOGE(kTag, "failed to enable non-blocking mode");
    close(fd_);
    fd_ = -1;
    return ESP_FAIL;
  }

  ClearPeerState();
  ResetShaperState();
  running_ = true;
  ESP_LOGI(kTag, "listening on UDP port %u", static_cast<unsigned>(cfg_.port));
  return ESP_OK;
}

void UdpServer::Stop() {
  if (fd_ >= 0) {
    close(fd_);
    fd_ = -1;
  }

  ClearPeerState();
  running_ = false;
}

void UdpServer::ClearPeer() {
  ClearPeerState();
}

int UdpServer::Receive(uint8_t *dst, size_t max_len) {
  if (!running_ || fd_ < 0 || dst == nullptr || max_len == 0) {
    return 0;
  }

  if (!upload_cap_enabled_) {
    sockaddr_in peer{};
    socklen_t peer_len = sizeof(peer);
    const int received = recvfrom(
        fd_, dst, max_len, 0, reinterpret_cast<sockaddr *>(&peer), &peer_len);
    if (received > 0) {
      peer_ipv4_ = peer.sin_addr.s_addr;
      peer_port_ = peer.sin_port;
      return received;
    }

    if (received < 0 && !IsWouldBlock(errno)) {
      ESP_LOGW(kTag, "recvfrom failed: errno=%d", errno);
    }
    return 0;
  }

  const auto on_upload_overflow = [this](unsigned dropped_bytes) {
    ++upload_overflow_count_;
    ESP_LOGW(kTag, "upload shaper overflow: dropped=%u threshold=%u count=%u",
             dropped_bytes, static_cast<unsigned>(cfg_.overflow_threshold),
             static_cast<unsigned>(upload_overflow_count_));
    Sys().TonePlayer().PlayBuiltin(::TonePlayer::BuiltinTone::kError);
    if (upload_overflow_count_ >= cfg_.overflow_threshold) {
      Panic(ErrorCode::kUdpServerUploadOverflow);
    }
  };

  RefillTokens(upload_cap_bytes_per_s_, kUploadBufferBytes,
               upload_tokens_bytes_, upload_last_refill_us_);

  uint8_t rx_buf[512];
  while (upload_shaper_buffer_.Available() < kUploadBufferBytes) {
    sockaddr_in peer{};
    socklen_t peer_len = sizeof(peer);
    const int received =
        recvfrom(fd_, rx_buf, sizeof(rx_buf), 0,
                 reinterpret_cast<sockaddr *>(&peer), &peer_len);
    if (received > 0) {
      peer_ipv4_ = peer.sin_addr.s_addr;
      peer_port_ = peer.sin_port;

      const size_t available = upload_shaper_buffer_.Available();
      const size_t free_bytes =
          (available < kUploadBufferBytes)
              ? (static_cast<size_t>(kUploadBufferBytes) - available)
              : 0;
      if (free_bytes == 0) {
        on_upload_overflow(static_cast<unsigned>(received));
        break;
      }

      const size_t accepted =
          std::min(static_cast<size_t>(received), free_bytes);
      const size_t wrote = upload_shaper_buffer_.PushBlock(rx_buf, accepted);
      if (accepted < static_cast<size_t>(received)) {
        on_upload_overflow(
            static_cast<unsigned>(static_cast<size_t>(received) - accepted));
      }
      if (wrote < accepted) {
        on_upload_overflow(
            static_cast<unsigned>(static_cast<size_t>(accepted) - wrote));
        break;
      }
      if (wrote == static_cast<size_t>(received)) {
        upload_overflow_count_ = 0;
      }
      continue;
    }

    if (received < 0 && !IsWouldBlock(errno)) {
      ESP_LOGW(kTag, "recvfrom failed: errno=%d", errno);
    }
    break;
  }

  const size_t buffered = upload_shaper_buffer_.Available();
  if (buffered == 0 || upload_tokens_bytes_ == 0) {
    return 0;
  }

  const size_t out_budget = std::min(
      max_len, std::min(buffered, static_cast<size_t>(upload_tokens_bytes_)));
  size_t copied = 0;
  while (copied < out_budget) {
    const uint8_t *ptr = nullptr;
    const size_t contiguous = upload_shaper_buffer_.ContiguousReadable(ptr);
    if (contiguous == 0 || ptr == nullptr) {
      break;
    }
    const size_t take = std::min(contiguous, out_budget - copied);
    std::memcpy(dst + copied, ptr, take);
    upload_shaper_buffer_.Consume(take);
    copied += take;
  }

  upload_tokens_bytes_ -= static_cast<uint32_t>(copied);
  return static_cast<int>(copied);
}

int UdpServer::Send(const uint8_t *data, size_t len) {
  if (!running_ || fd_ < 0 || data == nullptr || len == 0) {
    return 0;
  }

  sockaddr_in peer{};
  peer.sin_family = AF_INET;
  if (peer_port_ != 0) {
    peer.sin_addr.s_addr = peer_ipv4_;
    peer.sin_port = peer_port_;
  } else {
    peer.sin_addr.s_addr = htonl(INADDR_BROADCAST);
    peer.sin_port = htons(cfg_.port);
  }

  const auto send_chunk = [&](const uint8_t *chunk, size_t chunk_len) {
    const int sent =
        sendto(fd_, chunk, chunk_len, 0,
               reinterpret_cast<const sockaddr *>(&peer), sizeof(peer));
    if (sent < 0 && !IsWouldBlock(errno)) {
      ESP_LOGW(kTag, "sendto failed: errno=%d", errno);
      ClearPeerState();
    }
    return (sent > 0) ? static_cast<size_t>(sent) : static_cast<size_t>(0);
  };

  const auto flush_download_buffer = [&]() {
    while (download_tokens_bytes_ > 0 && !download_shaper_buffer_.IsEmpty()) {
      const uint8_t *ptr = nullptr;
      const size_t contiguous = download_shaper_buffer_.ContiguousReadable(ptr);
      if (contiguous == 0 || ptr == nullptr) {
        break;
      }
      const size_t budget =
          std::min(contiguous, static_cast<size_t>(download_tokens_bytes_));
      const size_t sent = send_chunk(ptr, budget);
      if (sent == 0) {
        break;
      }
      download_shaper_buffer_.Consume(sent);
      download_tokens_bytes_ -= static_cast<uint32_t>(sent);
    }
  };

  if (!download_cap_enabled_) {
    return static_cast<int>(send_chunk(data, len));
  }

  const auto on_download_overflow = [this](unsigned dropped_bytes) {
    ++download_overflow_count_;
    ESP_LOGW(kTag, "download shaper overflow: dropped=%u threshold=%u count=%u",
             dropped_bytes, static_cast<unsigned>(cfg_.overflow_threshold),
             static_cast<unsigned>(download_overflow_count_));
    Sys().TonePlayer().PlayBuiltin(::TonePlayer::BuiltinTone::kError);
    if (download_overflow_count_ >= cfg_.overflow_threshold) {
      Panic(ErrorCode::kUdpServerDownloadOverflow);
    }
  };

  RefillTokens(download_cap_bytes_per_s_, kDownloadBufferBytes,
               download_tokens_bytes_, download_last_refill_us_);

  flush_download_buffer();

  const size_t buffered = download_shaper_buffer_.Available();
  const size_t free_bytes =
      (buffered < kDownloadBufferBytes)
          ? (static_cast<size_t>(kDownloadBufferBytes) - buffered)
          : 0;
  if (len > free_bytes) {
    on_download_overflow(static_cast<unsigned>(len));
    return 0;
  }

  const size_t wrote = download_shaper_buffer_.PushBlock(data, len);
  if (wrote != len) {
    on_download_overflow(static_cast<unsigned>(len - wrote));
    return 0;
  }
  download_overflow_count_ = 0;

  flush_download_buffer();

  return static_cast<int>(len);
}
