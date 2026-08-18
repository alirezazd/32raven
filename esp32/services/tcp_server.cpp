// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "tcp_server.hpp"

#include <fcntl.h>

#include <cctype>
#include <cerrno>
#include <cstdio>
#include <cstring>

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
  down_head_ = down_tail_ = 0;
  download_overflow_ = false;
  download_enabled_ = false;
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

void TcpServer::EnableBridge() { download_enabled_ = true; }

void TcpServer::DisableBridge() {
  download_enabled_ = false;
  down_head_ = down_tail_ = 0;
  download_overflow_ = false;
}

void TcpServer::StartDownload(size_t total_size) {
  download_enabled_ = true;
  down_head_ = down_tail_ = 0;
  download_overflow_ = false;
  status_ = Status{};
  status_.total = total_size;
}

void TcpServer::StopDownload() {
  download_enabled_ = false;
  down_head_ = down_tail_ = 0;
  download_overflow_ = false;
  status_ = Status{};
}

size_t TcpServer::ReadDownload(std::span<uint8_t> dst) {
  size_t n = 0;
  while (n < dst.size() && down_head_ != down_tail_) {
    dst[n++] = down_[down_head_];
    down_head_ = (down_head_ + 1) % kDownCap;
  }
  return n;
}

bool TcpServer::WriteDownload(const uint8_t *data, size_t len) {
  if (!download_enabled_) {
    return false;
  }
  if (len > RbFree(down_head_, down_tail_, kDownCap)) {
    download_overflow_ = true;
    return false;
  }
  for (size_t i = 0; i < len; ++i) {
    down_[down_tail_] = data[i];
    down_tail_ = (down_tail_ + 1) % kDownCap;
  }
  return true;
}

void TcpServer::SetStatus(const Status &s) { status_ = s; }

TcpServer::Status TcpServer::GetStatus() const {
  Status s = status_;
  return s;
}

bool TcpServer::SetNonblock(int fd) {
  const int fl = fcntl(fd, F_GETFL, 0);
  if (fl < 0) return false;
  return fcntl(fd, F_SETFL, fl | O_NONBLOCK) == 0;
}

bool TcpServer::SetKeepalive(int fd, const Config &cfg) {
  if (cfg.keepalive_idle_s <= 0) return true;

  int yes = 1;
  if (setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &yes, sizeof(yes)) != 0)
    return false;

  // LWIP uses these names; ESP-IDF maps them.
  int idle = cfg.keepalive_idle_s;
  int intvl = cfg.keepalive_intvl_s;
  int cnt = cfg.keepalive_cnt;

  (void)setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof(idle));
  (void)setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &intvl, sizeof(intvl));
  (void)setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT, &cnt, sizeof(cnt));
  return true;
}

bool TcpServer::MakeListenSocket(int &out_fd, uint16_t port) {
  out_fd = (int)socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
  if (out_fd < 0) return false;

  int opt = 1;
  (void)setsockopt(out_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  addr.sin_addr.s_addr = htonl(INADDR_ANY);

  if (bind(out_fd, (sockaddr *)&addr, sizeof(addr)) != 0) {
    close(out_fd);
    out_fd = -1;
    return false;
  }

  // One control client and one data client, and both accept paths close
  // anything past those on the spot, so a deeper queue would only hold
  // connections that get accepted and dropped. Zero is not the next step down:
  // lwIP refuses every SYN once accepts_pending reaches the backlog.
  if (listen(out_fd, 1) != 0) {
    close(out_fd);
    out_fd = -1;
    return false;
  }

  (void)SetNonblock(out_fd);

  return true;
}

// Close helpers

void TcpServer::CloseCtrl() {
  if (ctrl_fd_ >= 0) {
    close(ctrl_fd_);
    ctrl_fd_ = -1;
    ctrl_peer_ipv4_ = 0;

    // also drop data when ctrl is gone (single-session rule)
    CloseData();

    Event e{};
    e.id = EventId::kCtrlDown;
    (void)PushEvent(e);
  }
}

void TcpServer::CloseData() {
  if (data_fd_ >= 0) {
    close(data_fd_);
    data_fd_ = -1;

    Event e{};
    e.id = EventId::kDataDown;
    (void)PushEvent(e);
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

  (void)SetNonblock(fd);
  (void)SetKeepalive(fd, cfg_);

  ctrl_fd_ = fd;
  ctrl_peer_ipv4_ = cli.sin_addr.s_addr;  // network byte order

  ESP_LOGI(kTag, "Ctrl connected fd=%d ip=%x", fd,
           (unsigned)ctrl_peer_ipv4_);

  // reset parser buffer
  line_len_ = 0;

  Event e{};
  e.id = EventId::kCtrlUp;
  (void)PushEvent(e);
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

  (void)SetNonblock(fd);
  (void)SetKeepalive(fd, cfg_);

  data_fd_ = fd;

  Event e{};
  e.id = EventId::kDataUp;
  (void)PushEvent(e);
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
      // feed bytes into line parser (PART 3 implements linebuf_add_ and
      // HandleLine_)
      for (int i = 0; i < r; i++) {
        if (LinebufAdd(buf[i])) {
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

  static_assert(kDataPumpReadBytes * kDataPumpMaxIterations <= kDownCap,
                "one pump cycle can outrun the download ring, so flow control "
                "would engage on every pump rather than under backpressure");

  uint8_t buf[kDataPumpReadBytes];
  for (int iter = 0; iter < kDataPumpMaxIterations; ++iter) {
    // Flow control: check space before pulling from TCP
    bool enabled = download_enabled_;
    size_t free = RbFree(down_head_, down_tail_, kDownCap);

    // If download is enabled but no space, stop reading to assert method
    // (backpressure). But we MUST check if the peer closed the connection!
    if (enabled && free == 0) {
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
    if (enabled && free < cap) {
      cap = free;
    }

    int r = recv(data_fd_, buf, cap, 0);
    if (r > 0) {
      // store bytes if allowed, else drop (policy controlled by App via
      // SetDownloadEnabled)
      if (!WriteDownload(buf, (size_t)r)) {
        // buffer full or download disabled
        // keep socket open; App can disable download and/or close data
        // connection by policy
      }
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


bool TcpServer::LinebufAdd(char b) {
  if (b == '\r') return false;

  if (b == '\n') {
    return line_len_ > 0;
  }

  if (line_len_ >= TcpServer::kMaxLineBytes) {
    // line too long -> saturate (truncate), drop extra chars
    return false;
  }

  line_buf_[line_len_++] = (char)b;
  return false;
}

// parse "key=value" where value is u32 decimal or hex (0x..). returns true if
// found.
static bool FindU32KV(const char *line, const char *key, uint32_t &out) {
  const char *p = line;
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

      uint64_t v = 0;
      bool any = false;
      while (*p) {
        char c = *p;
        unsigned d;
        if (c >= '0' && c <= '9')
          d = (unsigned)(c - '0');
        else if (base == 16 && c >= 'a' && c <= 'f')
          d = 10u + (unsigned)(c - 'a');
        else if (base == 16 && c >= 'A' && c <= 'F')
          d = 10u + (unsigned)(c - 'A');
        else
          break;

        if (d >= (unsigned)base) break;

        any = true;
        v = (v * (uint64_t)base) + (uint64_t)d;
        if (v > 0xFFFFFFFFull) return false;
        ++p;
      }

      if (!any) return false;

      out = (uint32_t)v;
      return true;
    }

    // skip token
    while (*p && *p != ' ' && *p != '\t') ++p;
  }

  return false;
}

static bool FindStrKV(const char *p, const char *key, char *out_buf,
                      size_t max_len) {
  if (!p || !key || !out_buf || max_len == 0) return false;

  const size_t len = std::strlen(key);

  while (*p) {
    p = SkipSpace(p);
    if (!*p) break;

    // find key at token start
    if (std::strncmp(p, key, len) == 0 && p[len] == '=') {
      p += len + 1;
      // Copy value until space or end
      size_t i = 0;
      while (*p && *p != ' ' && *p != '\t' && *p != '\r' && *p != '\n') {
        if (i < max_len - 1) {
          out_buf[i++] = *p;
        }
        ++p;
      }
      out_buf[i] = '\0';
      return true;
    }

    // skip token
    while (*p && *p != ' ' && *p != '\t' && *p != '\r' && *p != '\n') {
      ++p;
    }
  }
  return false;
}

static bool StartsWithCI(const char *s, const char *prefix) {
  while (*prefix) {
    char a = *s++;
    char b = *prefix++;
    if (std::toupper((unsigned char)a) != std::toupper((unsigned char)b))
      return false;
  }
  return true;
}

// Command handling

// BEGIN and LOG are answered by whoever pops the event, not here: only the
// current page knows whether it can serve them, and a host that has already
// been told OK commits to streaming before the refusal could reach it.
void TcpServer::HandleLine(const char *line) {
  if (!line) return;

  const char *p = SkipSpace(line);
  if (!*p) return;

  if (StartsWithCI(p, "BEGIN")) {
    uint32_t size = 0;
    uint32_t crc = 0;
    bool ok_size = FindU32KV(p, "size", size);
    (void)FindU32KV(p, "crc", crc);

    if (!ok_size || size == 0) {
      SendCtrlLine("ERR bad_size\n");
      return;
    }

    Event e{};
    e.id = EventId::kBegin;
    e.begin.size = size;
    e.begin.crc = crc;
    (void)FindStrKV(p, "target", e.begin.target, sizeof(e.begin.target));
    if (!PushEvent(e)) {
      SendCtrlLine("ERR evt_queue_full\n");
      return;
    }
    return;
  }

  if (StartsWithCI(p, "ABORT")) {
    Event e{};
    e.id = EventId::kAbort;
    (void)PushEvent(e);
    SendCtrlLine("OK\n");
    return;
  }

  if (StartsWithCI(p, "RESET")) {
    Event e{};
    e.id = EventId::kReset;
    (void)PushEvent(e);
    SendCtrlLine("OK\n");
    return;
  }

  if (StartsWithCI(p, "BRIDGE") || StartsWithCI(p, "MONITOR")) {
    Event e{};
    e.id = EventId::kBridge;
    (void)PushEvent(e);
    SendCtrlLine("OK\n");
    return;
  }

  if (StartsWithCI(p, "LOG")) {
    const char *arg = SkipSpace(p + 3);
    if (StartsWithCI(arg, "LIST")) {
      Event e{};
      e.id = EventId::kLogList;
      if (!PushEvent(e)) {
        SendCtrlLine("ERR evt_queue_full\n");
        return;
      }
      return;
    }
    if (StartsWithCI(arg, "GET")) {
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

  if (StartsWithCI(p, "STATUS?") || StartsWithCI(p, "STATUS")) {
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
  down_head_ = down_tail_ = 0;
  download_overflow_ = false;
  download_enabled_ = false;
  status_ = Status{};

  ctrl_fd_ = -1;
  data_fd_ = -1;
  ctrl_peer_ipv4_ = 0;
  line_len_ = 0;

  if (!MakeListenSocket(ctrl_listen_fd_, cfg_.ctrl_port) ||
      !MakeListenSocket(data_listen_fd_, cfg_.data_port)) {
    ESP_LOGE(kTag, "listen sockets failed; nothing listening");
    CloseAll();
    return;
  }

  running_ = true;
}
