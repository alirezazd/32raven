#include "udp_server.hpp"

#include <fcntl.h>

#include <cerrno>
#include <cstring>

extern "C" {
#include "esp_log.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
}

static constexpr const char *kTag = "udp_server";

void UdpServer::Init(const Config &cfg) {
  cfg_ = cfg;
  ESP_LOGI(kTag, "initialized on port %u", static_cast<unsigned>(cfg_.port));
}

bool UdpServer::SetNonblock(int fd) {
  const int flags = fcntl(fd, F_GETFL, 0);
  if (flags < 0) {
    return false;
  }
  return fcntl(fd, F_SETFL, flags | O_NONBLOCK) == 0;
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

  if (cfg_.nonblocking && !SetNonblock(fd_)) {
    ESP_LOGE(kTag, "failed to enable non-blocking mode");
    close(fd_);
    fd_ = -1;
    return ESP_FAIL;
  }

  peer_ipv4_ = 0;
  peer_port_ = 0;
  running_ = true;
  ESP_LOGI(kTag, "listening on UDP port %u",
           static_cast<unsigned>(cfg_.port));
  return ESP_OK;
}

void UdpServer::Stop() {
  if (fd_ >= 0) {
    close(fd_);
    fd_ = -1;
  }

  ClearPeer();
  running_ = false;
}

void UdpServer::ClearPeer() {
  peer_ipv4_ = 0;
  peer_port_ = 0;
}

int UdpServer::Receive(uint8_t *dst, size_t max_len) {
  if (!running_ || fd_ < 0 || dst == nullptr || max_len == 0) {
    return 0;
  }

  sockaddr_in peer{};
  socklen_t peer_len = sizeof(peer);
  const int received =
      recvfrom(fd_, dst, max_len, 0, reinterpret_cast<sockaddr *>(&peer),
               &peer_len);
  if (received > 0) {
    peer_ipv4_ = peer.sin_addr.s_addr;
    peer_port_ = peer.sin_port;
    return received;
  }

  if (received < 0 && errno != EWOULDBLOCK && errno != EAGAIN) {
    ESP_LOGW(kTag, "recvfrom failed: errno=%d", errno);
  }
  return 0;
}

int UdpServer::Send(const uint8_t *data, size_t len) {
  if (!running_ || fd_ < 0 || data == nullptr || len == 0) {
    return 0;
  }

  sockaddr_in peer{};
  peer.sin_family = AF_INET;
  if (HasPeer()) {
    peer.sin_addr.s_addr = peer_ipv4_;
    peer.sin_port = peer_port_;
  } else {
    peer.sin_addr.s_addr = htonl(INADDR_BROADCAST);
    peer.sin_port = htons(cfg_.port);
  }

  const int sent =
      sendto(fd_, data, len, 0, reinterpret_cast<const sockaddr *>(&peer),
             sizeof(peer));
  if (sent < 0 && errno != EWOULDBLOCK && errno != EAGAIN) {
    ESP_LOGW(kTag, "sendto failed: errno=%d", errno);
    ClearPeer();
  }
  return (sent > 0) ? sent : 0;
}
