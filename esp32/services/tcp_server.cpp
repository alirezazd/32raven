// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "tcp_server.hpp"

#include <fcntl.h>

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <charconv>
#include <cstdio>
#include <cstring>
#include <string_view>

extern "C" {
#include "esp_log.h"
#include "freertos/FreeRTOS.h"  // IWYU pragma: keep
#include "freertos/task.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
}

static constexpr const char *kTag = "tcp_server";
// One data-pump cycle: bounded work per tick, and together the amount the
// download ring has to be able to swallow without asserting backpressure.
static constexpr size_t kDataPumpReadBytes = 1024;
static constexpr int kDataPumpMaxIterations = 4;

// ring buffer helpers
static inline size_t RbUsed(size_t head, size_t tail, size_t cap) {
  return (tail >= head) ? (tail - head) : (cap - head + tail);
}
static inline size_t RbFree(size_t head, size_t tail, size_t cap) {
  // keep 1 byte gap
  return (cap - 1) - RbUsed(head, tail, cap);
}

// TcpServer core

TcpServer::TcpServer() = default;

TcpServer &TcpServer::GetInstance() {
  static TcpServer instance;
  return instance;
}

void TcpServer::Init(const Config &cfg) {
  cfg_ = cfg;
  ESP_LOGI(kTag, "initialized on port %d", cfg.ctrl_port);
}

void TcpServer::Stop() {
  if (!running_) return;

  running_ = false;

  CloseAll();

  // Clear buffers/queues
  evt_head_ = evt_tail_ = 0;
  data_rx_head_ = data_rx_tail_ = 0;
  data_rx_open_ = false;
}

void TcpServer::Poll() {
  if (!running_) return;

  AcceptCtrl();
  AcceptData();
  PumpCtrlRx();
  PumpDataRx();
}

// SM-facing API (queue/buffer/status)

std::optional<TcpServer::Event> TcpServer::PopEvent() {
  if (evt_head_ == evt_tail_) {
    return std::nullopt;
  }
  Event out = evt_q_[evt_head_];
  evt_head_ = (evt_head_ + 1) % kEvtCap;
  return out;
}

bool TcpServer::PushEvent(const Event &e) {
  size_t next = (evt_tail_ + 1) % kEvtCap;
  if (next == evt_head_) {
    return false;
  }
  evt_q_[evt_tail_] = e;
  evt_tail_ = next;
  return true;
}

void TcpServer::QueueSimpleCommand(EventId id) {
  Event e{};
  e.id = id;
  if (!PushEvent(e)) {
    SendCtrlLine("ERR evt_queue_full\n");
    return;
  }
  SendCtrlLine("OK\n");
}

TcpServer::LinkDrops TcpServer::TakeLinkDrops() {
  const LinkDrops out = link_drops_;
  link_drops_ = LinkDrops{};
  return out;
}

void TcpServer::OpenDataRx() { data_rx_open_ = true; }

void TcpServer::CloseDataRx() {
  data_rx_open_ = false;
  data_rx_head_ = data_rx_tail_ = 0;
}

void TcpServer::BeginTransfer(size_t total_size) {
  data_rx_open_ = true;
  data_rx_head_ = data_rx_tail_ = 0;
  status_ = Status{};
  status_.total = total_size;
}

void TcpServer::EndTransfer() {
  data_rx_open_ = false;
  data_rx_head_ = data_rx_tail_ = 0;
  status_ = Status{};
}

size_t TcpServer::ReadDataRx(std::span<uint8_t> dst) {
  size_t n = 0;
  while (n < dst.size() && data_rx_head_ != data_rx_tail_) {
    dst[n++] = data_rx_[data_rx_head_];
    data_rx_head_ = (data_rx_head_ + 1) % kDataRxCap;
  }
  return n;
}

// len always fits: the pump caps its recv at ring free space while the sink
// is open, and a closed sink drains-and-drops here.
void TcpServer::StageDataRx(const uint8_t *data, size_t len) {
  if (!data_rx_open_) {
    return;
  }
  for (size_t i = 0; i < len; ++i) {
    data_rx_[data_rx_tail_] = data[i];
    data_rx_tail_ = (data_rx_tail_ + 1) % kDataRxCap;
  }
}

void TcpServer::SetStatus(const Status &s) { status_ = s; }

TcpServer::Status TcpServer::GetStatus() const { return status_; }

bool TcpServer::SetNonblock(int fd) {
  const int fl = fcntl(fd, F_GETFL, 0);
  if (fl < 0) return false;
  return fcntl(fd, F_SETFL, fl | O_NONBLOCK) == 0;
}

void TcpServer::SetKeepalive(int fd, const Config &cfg) {
  if (cfg.keepalive_idle_s <= 0) return;

  int yes = 1;
  if (setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &yes, sizeof(yes)) != 0) return;

  // LWIP uses these names; ESP-IDF maps them.
  int idle = cfg.keepalive_idle_s;
  int intvl = cfg.keepalive_intvl_s;
  int cnt = cfg.keepalive_cnt;

  (void)setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof(idle));
  (void)setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &intvl, sizeof(intvl));
  (void)setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT, &cnt, sizeof(cnt));
}

std::optional<int> TcpServer::MakeListenSocket(uint16_t port) {
  const int out_fd = (int)socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
  if (out_fd < 0) return std::nullopt;

  int opt = 1;
  (void)setsockopt(out_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  addr.sin_addr.s_addr = htonl(INADDR_ANY);

  if (bind(out_fd, (sockaddr *)&addr, sizeof(addr)) != 0) {
    close(out_fd);
    return std::nullopt;
  }

  // One control client and one data client, and both accept paths close
  // anything past those on the spot, so a deeper queue would only hold
  // connections that get accepted and dropped. Zero is not the next step down:
  // lwIP refuses every SYN once accepts_pending reaches the backlog.
  if (listen(out_fd, 1) != 0) {
    close(out_fd);
    return std::nullopt;
  }

  if (!SetNonblock(out_fd)) {
    close(out_fd);
    return std::nullopt;
  }

  return out_fd;
}

// Close helpers

void TcpServer::CloseCtrl() {
  if (ctrl_fd_ >= 0) {
    close(ctrl_fd_);
    ctrl_fd_ = -1;
    ctrl_peer_ipv4_ = 0;

    // also drop data when ctrl is gone (single-session rule)
    CloseData();

    link_drops_.ctrl = true;
  }
}

void TcpServer::CloseData() {
  if (data_fd_ >= 0) {
    close(data_fd_);
    data_fd_ = -1;

    link_drops_.data = true;
  }
}

void TcpServer::CloseAll() {
  CloseData();
  CloseCtrl();

  if (ctrl_listen_fd_ >= 0) {
    close(ctrl_listen_fd_);
    ctrl_listen_fd_ = -1;
  }
  if (data_listen_fd_ >= 0) {
    close(data_listen_fd_);
    data_listen_fd_ = -1;
  }
}

// Accept helpers

void TcpServer::AcceptCtrl() {
  if (ctrl_listen_fd_ < 0) return;

  // single control client: reject new connections if already connected
  if (ctrl_fd_ >= 0) {
    sockaddr_in tmp{};
    socklen_t len = sizeof(tmp);
    int fd = accept(ctrl_listen_fd_, (sockaddr *)&tmp, &len);
    if (fd >= 0) close(fd);  // reject
    return;
  }

  sockaddr_in cli{};
  socklen_t len = sizeof(cli);
  int fd = accept(ctrl_listen_fd_, (sockaddr *)&cli, &len);
  if (fd < 0) return;

  if (!SetNonblock(fd)) {
    close(fd);
    return;
  }
  SetKeepalive(fd, cfg_);

  ctrl_fd_ = fd;
  ctrl_peer_ipv4_ = cli.sin_addr.s_addr;  // network byte order

  ESP_LOGI(kTag, "Ctrl connected fd=%d ip=%x", fd,
           (unsigned)ctrl_peer_ipv4_);

  // reset parser buffer
  line_len_ = 0;
  line_overflow_ = false;
}

void TcpServer::AcceptData() {
  if (data_listen_fd_ < 0) return;

  // accept data only if ctrl exists (single session)
  if (ctrl_fd_ < 0) {
    sockaddr_in tmp{};
    socklen_t len = sizeof(tmp);
    int fd = accept(data_listen_fd_, (sockaddr *)&tmp, &len);
    if (fd >= 0) close(fd);  // reject when no control session
    return;
  }

  // single data client: reject if already connected
  if (data_fd_ >= 0) {
    sockaddr_in tmp{};
    socklen_t len = sizeof(tmp);
    int fd = accept(data_listen_fd_, (sockaddr *)&tmp, &len);
    if (fd >= 0) close(fd);
    return;
  }

  sockaddr_in cli{};
  socklen_t len = sizeof(cli);
  int fd = accept(data_listen_fd_, (sockaddr *)&cli, &len);
  if (fd < 0) return;

  ESP_LOGI(kTag, "Data connected fd=%d", fd);

  // optional binding by IP: accept data only from control peer
  if (ctrl_peer_ipv4_ != 0 && cli.sin_addr.s_addr != ctrl_peer_ipv4_) {
    close(fd);
    return;
  }

  if (!SetNonblock(fd)) {
    close(fd);
    return;
  }
  SetKeepalive(fd, cfg_);

  data_fd_ = fd;
}

// RX pumps

void TcpServer::PumpCtrlRx() {
  if (ctrl_fd_ < 0) return;

  uint8_t buf[256];
  for (int iter = 0; iter < 4; ++iter) {  // bounded work per tick
    // Re-checked each pass: a command handled below is free to drop the
    // connection, and the next recv would then run on a closed socket.
    if (ctrl_fd_ < 0) return;
    int r = recv(ctrl_fd_, buf, sizeof(buf), 0);
    if (r > 0) {
      for (int i = 0; i < r; i++) {
        const LineFeed fed = LinebufAdd(buf[i]);
        if (fed == LineFeed::kTruncated) {
          SendCtrlLine("ERR line_too_long\n");
          line_len_ = 0;
          continue;
        }
        if (fed == LineFeed::kComplete) {
          line_buf_[line_len_] = '\0';
          ESP_LOGI(kTag, "Cmd line: %s", line_buf_);
          HandleLine(line_buf_);
          line_len_ = 0;
        }
      }
      continue;
    }

    if (r == 0) {
      // peer closed
      CloseCtrl();
      return;
    }

    // r < 0
    if (errno == EWOULDBLOCK || errno == EAGAIN) return;

    // real error
    CloseCtrl();
    return;
  }
}

void TcpServer::PumpDataRx() {
  if (data_fd_ < 0) return;

  static_assert(kDataPumpReadBytes * kDataPumpMaxIterations <= kDataRxCap,
                "one pump cycle can outrun the download ring, so flow control "
                "would engage on every pump rather than under backpressure");

  uint8_t buf[kDataPumpReadBytes];
  for (int iter = 0; iter < kDataPumpMaxIterations; ++iter) {
    // Flow control: check space before pulling from TCP
    const bool open = data_rx_open_;
    const size_t free = RbFree(data_rx_head_, data_rx_tail_, kDataRxCap);

    // An open sink with no space stops reading to assert backpressure -- but
    // the peer close must still be seen.
    if (open && free == 0) {
      char dummy;
      int r = recv(data_fd_, &dummy, 1, MSG_PEEK | MSG_DONTWAIT);
      if (r == 0) {
        ESP_LOGW(kTag, "Data peer closed (recv=0 during backpressure)");
        CloseData();
        return;
      }
      if (r < 0) {
        if (errno != EWOULDBLOCK && errno != EAGAIN) {
          ESP_LOGE(kTag, "Data recv error during backpressure errno=%d", errno);
          CloseData();
          return;
        }
      }
      return;
    }

    // Determine read size
    size_t cap = sizeof(buf);
    if (open && free < cap) {
      cap = free;
    }

    int r = recv(data_fd_, buf, cap, 0);
    if (r > 0) {
      StageDataRx(buf, (size_t)r);
      continue;
    }

    if (r == 0) {
      CloseData();
      return;
    }

    if (errno == EWOULDBLOCK || errno == EAGAIN) return;

    CloseData();
    return;
  }
}

// CTRL TX

void TcpServer::SendCtrlLine(const char *line) {
  if (!line || ctrl_fd_ < 0) return;

  const size_t n = std::strlen(line);
  if (n == 0) return;

  // Best-effort non-blocking send
  int r = send(ctrl_fd_, line, n, 0);
  if (r < 0) {
    if (errno == EWOULDBLOCK || errno == EAGAIN) return;  // full: line dropped
    CloseCtrl();
    return;
  }
}

int TcpServer::SendData(const uint8_t *data, size_t len) {
  if (data_fd_ < 0 || !data || len == 0) return -1;

  // Best-effort non-blocking send
  int r = send(data_fd_, data, len, 0);
  if (r < 0) {
    if (errno == EWOULDBLOCK || errno == EAGAIN)
      return 0;  // buffer full, drop or retry later (streaming logic: drop used
                 // here)
    // CloseData(); // Optional: close on error? For now, just report error.
    return -1;
  }
  return r;
}

static inline const char *SkipSpace(const char *p) {
  while (*p && (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n')) ++p;
  return p;
}

// Control line buffer helpers


TcpServer::LineFeed TcpServer::LinebufAdd(char b) {
  if (b == '\r') return LineFeed::kIncomplete;

  if (b == '\n') {
    const bool truncated = line_overflow_;
    line_overflow_ = false;
    if (truncated) return LineFeed::kTruncated;
    return (line_len_ > 0) ? LineFeed::kComplete : LineFeed::kIncomplete;
  }

  if (line_len_ >= TcpServer::kMaxLineBytes) {
    line_overflow_ = true;
    return LineFeed::kIncomplete;
  }

  line_buf_[line_len_++] = (char)b;
  return LineFeed::kIncomplete;
}

// nullopt when the key is absent, unparseable, or wider than u32.
static std::optional<uint32_t> FindU32KV(const char *line, const char *key) {
  const char *p = line;
  const char *const end = line + std::strlen(line);
  const size_t len = std::strlen(key);

  while (*p) {
    p = SkipSpace(p);
    if (!*p) break;

    // find key at token start
    if (std::strncmp(p, key, len) == 0 && p[len] == '=') {
      p += len + 1;

      int base = 10;
      if (p[0] == '0' && (p[1] == 'x' || p[1] == 'X')) {
        base = 16;
        p += 2;
      }

      uint32_t v = 0;
      if (std::from_chars(p, end, v, base).ec != std::errc{}) {
        return std::nullopt;
      }
      return v;
    }

    // skip token
    while (*p && *p != ' ' && *p != '\t') ++p;
  }

  return std::nullopt;
}

// The value token for key, viewed into the line; nullopt when the key is
// absent.
static std::optional<std::string_view> FindStrKV(const char *p,
                                                 const char *key) {
  const size_t len = std::strlen(key);

  while (*p) {
    p = SkipSpace(p);
    if (!*p) break;

    // find key at token start
    if (std::strncmp(p, key, len) == 0 && p[len] == '=') {
      p += len + 1;
      const char *const start = p;
      while (*p && *p != ' ' && *p != '\t' && *p != '\r' && *p != '\n') {
        ++p;
      }
      return std::string_view{start, static_cast<size_t>(p - start)};
    }

    // skip token
    while (*p && *p != ' ' && *p != '\t' && *p != '\r' && *p != '\n') {
      ++p;
    }
  }
  return std::nullopt;
}

// The first token of s equals verb, case-insensitive: the match must end at
// space or end of line, so BEGINNING is not BEGIN.
static bool TokenEqCI(const char *s, const char *verb) {
  while (*verb) {
    char a = *s++;
    char b = *verb++;
    if (std::toupper((unsigned char)a) != std::toupper((unsigned char)b))
      return false;
  }
  return *s == '\0' || *s == ' ' || *s == '\t';
}

// Command handling

// BEGIN and LOG are answered by whoever pops the event, not here: only the
// current page knows whether it can serve them, and a host that has already
// been told OK commits to streaming before the refusal could reach it.
void TcpServer::HandleLine(const char *line) {
  if (!line) return;

  const char *p = SkipSpace(line);
  if (!*p) return;

  if (TokenEqCI(p, "BEGIN")) {
    const std::optional<uint32_t> parsed_size = FindU32KV(p, "size");
    const uint32_t size = parsed_size.value_or(0);
    const uint32_t crc = FindU32KV(p, "crc").value_or(0);

    if (!parsed_size || size == 0) {
      SendCtrlLine("ERR bad_size\n");
      return;
    }

    Event e{};
    e.id = EventId::kBegin;
    e.begin.size = size;
    e.begin.crc = crc;
    const std::optional<std::string_view> target = FindStrKV(p, "target");
    if (target) {
      const size_t n = std::min(target->size(), sizeof(e.begin.target) - 1);
      std::memcpy(e.begin.target, target->data(), n);
    }
    if (!PushEvent(e)) {
      SendCtrlLine("ERR evt_queue_full\n");
      return;
    }
    return;
  }

  if (TokenEqCI(p, "ABORT")) {
    QueueSimpleCommand(EventId::kAbort);
    return;
  }

  if (TokenEqCI(p, "RESET")) {
    QueueSimpleCommand(EventId::kReset);
    return;
  }

  if (TokenEqCI(p, "BRIDGE") || TokenEqCI(p, "MONITOR")) {
    QueueSimpleCommand(EventId::kBridge);
    return;
  }

  if (TokenEqCI(p, "LOG")) {
    const char *arg = SkipSpace(p + 3);
    if (TokenEqCI(arg, "LIST")) {
      Event e{};
      e.id = EventId::kLogList;
      if (!PushEvent(e)) {
        SendCtrlLine("ERR evt_queue_full\n");
        return;
      }
      return;
    }
    if (TokenEqCI(arg, "GET")) {
      const char *name = SkipSpace(arg + 3);
      Event e{};
      e.id = EventId::kLogGet;
      size_t n = 0;
      while (name[n] && name[n] != ' ' && name[n] != '\r' &&
             name[n] != '\n' && n < sizeof(e.log_name) - 1) {
        e.log_name[n] = name[n];
        ++n;
      }
      if (n == 0) {
        SendCtrlLine("ERR bad_name\n");
        return;
      }
      if (!PushEvent(e)) {
        SendCtrlLine("ERR evt_queue_full\n");
        return;
      }
      return;
    }
    SendCtrlLine("ERR unknown_cmd\n");
    return;
  }

  if (TokenEqCI(p, "STATUS?") || TokenEqCI(p, "STATUS")) {
    Status st = GetStatus();
    char buf[128];
    std::snprintf(buf, sizeof(buf), "STATUS rx=%u total=%u state=%u err=%u\n",
                  (unsigned)st.rx, (unsigned)st.total, (unsigned)st.state,
                  (unsigned)st.err);
    SendCtrlLine(buf);
    return;
  }

  SendCtrlLine("ERR unknown_cmd\n");
}

void TcpServer::Start() {
  if (running_) return;

  evt_head_ = evt_tail_ = 0;
  data_rx_head_ = data_rx_tail_ = 0;
  data_rx_open_ = false;
  link_drops_ = LinkDrops{};
  status_ = Status{};

  ctrl_fd_ = -1;
  data_fd_ = -1;
  ctrl_peer_ipv4_ = 0;
  line_len_ = 0;
  line_overflow_ = false;

  const std::optional<int> ctrl_fd = MakeListenSocket(cfg_.ctrl_port);
  const std::optional<int> data_fd = MakeListenSocket(cfg_.data_port);
  ctrl_listen_fd_ = ctrl_fd.value_or(-1);
  data_listen_fd_ = data_fd.value_or(-1);
  if (!ctrl_fd || !data_fd) {
    ESP_LOGE(kTag, "listen sockets failed; nothing listening");
    CloseAll();
    return;
  }

  running_ = true;
}
