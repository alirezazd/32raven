#include "tcp_server.hpp"

#include <cctype>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
extern "C" {
#include "esp_log.h"
#include "freertos/FreeRTOS.h" // IWYU pragma: keep
#include "freertos/task.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
}

static constexpr const char *kTag = "tcp_server";

// ----- ring buffer helpers -----
static inline size_t RbUsed(size_t head, size_t tail, size_t cap) {
  return (tail >= head) ? (tail - head) : (cap - head + tail);
}
static inline size_t RbFree(size_t head, size_t tail, size_t cap) {
  // keep 1 byte gap
  return (cap - 1) - RbUsed(head, tail, cap);
}

// ---------------- TcpServer core ----------------

void TcpServer::Init(const Config &cfg, ErrorHandler error_handler) {
  cfg_ = cfg;

  // wire internal SM context
  ctx_.self = this;
  ctx_.sm = &sm_;

  ESP_LOGI(kTag, "initialized on port %d", cfg.ctrl_port);
}

void TcpServer::Stop() {
  if (!running_)
    return;

  running_ = false;

  CloseAll();

  // Clear buffers/queues
  evt_head_ = evt_tail_ = 0;
  up_head_ = up_tail_ = 0;
  upload_overflow_ = false;
  upload_enabled_ = false;
}

void TcpServer::Poll(SmTick now) {
  if (!running_)
    return;

  ctx_.now = now;

  sm_.Step(now);
}

// ---------------- SM-facing API (queue/buffer/status) ----------------

bool TcpServer::PopEvent(Event &out) {
  if (evt_head_ == evt_tail_) {
    return false;
  }
  out = evt_q_[evt_head_];
  evt_head_ = (evt_head_ + 1) % kEvtCap;
  return true;
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

void TcpServer::SetUploadEnabled(bool en) {
  upload_enabled_ = en;
  if (!upload_enabled_) {
    up_head_ = up_tail_ = 0;
    upload_overflow_ = false;
  }
}

int TcpServer::ReadUpload(uint8_t *dst, size_t max_len) {
  if (!dst || max_len == 0)
    return 0;

  size_t n = 0;
  while (n < max_len && up_head_ != up_tail_) {
    dst[n++] = up_[up_head_];
    up_head_ = (up_head_ + 1) % kUpCap;
  }
  return (int)n;
}

bool TcpServer::WriteUpload(const uint8_t *data, size_t len) {
  if (!upload_enabled_) {
    return false;
  }
  if (len > RbFree(up_head_, up_tail_, kUpCap)) {
    upload_overflow_ = true;
    return false;
  }
  for (size_t i = 0; i < len; ++i) {
    up_[up_tail_] = data[i];
    up_tail_ = (up_tail_ + 1) % kUpCap;
  }
  return true;
}

void TcpServer::SetStatus(const Status &s) { status_ = s; }

TcpServer::Status TcpServer::GetStatus() const {
  Status s = status_;
  return s;
}

bool TcpServer::SetNonblock(int fd) {
  const int kFl = fcntl(fd, F_GETFL, 0);
  if (kFl < 0)
    return false;
  return fcntl(fd, F_SETFL, kFl | O_NONBLOCK) == 0;
}

bool TcpServer::SetKeepalive(int fd, const Config &cfg) {
  if (cfg.keepalive_idle_s <= 0)
    return true;

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

bool TcpServer::MakeListenSocket(int &out_fd, uint16_t port, int backlog,
                                 bool nonblocking) {
  out_fd = (int)socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
  if (out_fd < 0)
    return false;

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

  if (listen(out_fd, backlog) != 0) {
    close(out_fd);
    out_fd = -1;
    return false;
  }

  if (nonblocking)
    (void)SetNonblock(out_fd);

  return true;
}

// ---------------- Close helpers ----------------

void TcpServer::CloseCtrl() {
  if (ctx_.ctrl_fd >= 0) {
    close(ctx_.ctrl_fd);
    ctx_.ctrl_fd = -1;
    ctx_.ctrl_peer_ipv4 = 0;

    // also drop data when ctrl is gone (single-session rule)
    CloseData();

    Event e{};
    e.id = EventId::kCtrlDown;
    (void)PushEvent(e);
  }
}

void TcpServer::CloseData() {
  if (ctx_.data_fd >= 0) {
    close(ctx_.data_fd);
    ctx_.data_fd = -1;

    Event e{};
    e.id = EventId::kDataDown;
    (void)PushEvent(e);
  }
}

void TcpServer::CloseAll() {
  CloseData();
  CloseCtrl();

  if (ctx_.ctrl_listen_fd >= 0) {
    close(ctx_.ctrl_listen_fd);
    ctx_.ctrl_listen_fd = -1;
  }
  if (ctx_.data_listen_fd >= 0) {
    close(ctx_.data_listen_fd);
    ctx_.data_listen_fd = -1;
  }
}

// ---------------- Accept helpers ----------------

void TcpServer::AcceptCtrl() {
  if (ctx_.ctrl_listen_fd < 0)
    return;

  // single control client: reject new connections if already connected
  if (ctx_.ctrl_fd >= 0) {
    sockaddr_in tmp{};
    socklen_t len = sizeof(tmp);
    int fd = accept(ctx_.ctrl_listen_fd, (sockaddr *)&tmp, &len);
    if (fd >= 0)
      close(fd); // reject
    return;
  }

  sockaddr_in cli{};
  socklen_t len = sizeof(cli);
  int fd = accept(ctx_.ctrl_listen_fd, (sockaddr *)&cli, &len);
  if (fd < 0)
    return;

  if (cfg_.nonblocking)
    (void)SetNonblock(fd);
  (void)SetKeepalive(fd, cfg_);

  ctx_.ctrl_fd = fd;
  ctx_.ctrl_peer_ipv4 = cli.sin_addr.s_addr; // network byte order

  ESP_LOGI(kTag, "Ctrl connected fd=%d ip=%x", fd,
           (unsigned)ctx_.ctrl_peer_ipv4);

  // reset parser buffer
  ctx_.line_len = 0;

  Event e{};
  e.id = EventId::kCtrlUp;
  (void)PushEvent(e);
}

void TcpServer::AcceptData() {
  if (ctx_.data_listen_fd < 0)
    return;

  // accept data only if ctrl exists (single session)
  if (ctx_.ctrl_fd < 0) {
    sockaddr_in tmp{};
    socklen_t len = sizeof(tmp);
    int fd = accept(ctx_.data_listen_fd, (sockaddr *)&tmp, &len);
    if (fd >= 0)
      close(fd); // reject when no control session
    return;
  }

  // single data client: reject if already connected
  if (ctx_.data_fd >= 0) {
    sockaddr_in tmp{};
    socklen_t len = sizeof(tmp);
    int fd = accept(ctx_.data_listen_fd, (sockaddr *)&tmp, &len);
    if (fd >= 0)
      close(fd);
    return;
  }

  sockaddr_in cli{};
  socklen_t len = sizeof(cli);
  int fd = accept(ctx_.data_listen_fd, (sockaddr *)&cli, &len);
  if (fd < 0)
    return;

  ESP_LOGI(kTag, "Data connected fd=%d", fd);

  // optional binding by IP: accept data only from control peer
  if (ctx_.ctrl_peer_ipv4 != 0 && cli.sin_addr.s_addr != ctx_.ctrl_peer_ipv4) {
    close(fd);
    return;
  }

  if (cfg_.nonblocking)
    (void)SetNonblock(fd);
  (void)SetKeepalive(fd, cfg_);

  ctx_.data_fd = fd;

  Event e{};
  e.id = EventId::kDataUp;
  (void)PushEvent(e);
}

// ---------------- RX pumps ----------------

void TcpServer::PumpCtrlRx() {
  if (ctx_.ctrl_fd < 0)
    return;

  uint8_t buf[256];
  for (int iter = 0; iter < 4; ++iter) { // bounded work per tick
    int r = recv(ctx_.ctrl_fd, buf, sizeof(buf), 0);
    if (r > 0) {
      // ESP_LOGI(kTag, "Ctrl rx %d bytes", r);
      // feed bytes into line parser (PART 3 implements linebuf_add_ and
      // HandleLine_)
      for (int i = 0; i < r; i++) {
        if (LinebufAdd(buf[i])) {
          ctx_.line_buf[ctx_.line_len] = '\0';
          ESP_LOGI(kTag, "Cmd line: %s", ctx_.line_buf);
          HandleLine(ctx_.line_buf);
          ctx_.line_len = 0;
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
    if (errno == EWOULDBLOCK || errno == EAGAIN)
      return;

    // real error
    CloseCtrl();
    return;
  }
}

void TcpServer::PumpDataRx() {
  if (ctx_.data_fd < 0)
    return;

  uint8_t buf[1024];
  for (int iter = 0; iter < 4; ++iter) { // bounded work
    // Flow control: check space before pulling from TCP
    bool enabled = upload_enabled_;
    size_t free = RbFree(up_head_, up_tail_, kUpCap);

    // If upload is enabled but no space, stop reading to assert method
    // (backpressure). But we MUST check if the peer closed the connection!
    if (enabled && free == 0) {
      char dummy;
      int r = recv(ctx_.data_fd, &dummy, 1, MSG_PEEK | MSG_DONTWAIT);
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

    int r = recv(ctx_.data_fd, buf, cap, 0);
    if (r > 0) {
      // store bytes if allowed, else drop (policy controlled by App via
      // SetUploadEnabled)
      if (!WriteUpload(buf, (size_t)r)) {
        // buffer full or upload disabled
        // keep socket open; App can disable upload and/or close data connection
        // by policy
      }
      continue;
    }

    if (r == 0) {
      CloseData();
      return;
    }

    if (errno == EWOULDBLOCK || errno == EAGAIN)
      return;

    CloseData();
    return;
  }
}

// ---------------- CTRL TX ----------------

esp_err_t TcpServer::SendCtrlLine(const char *line) {
  if (!line || ctx_.ctrl_fd < 0)
    return ESP_FAIL;

  const size_t kN = std::strlen(line);
  if (kN == 0)
    return ESP_OK;

  // Best-effort non-blocking send
  int r = send(ctx_.ctrl_fd, line, kN, 0);
  if (r < 0) {
    if (errno == EWOULDBLOCK || errno == EAGAIN)
      return ESP_ERR_TIMEOUT;
    CloseCtrl();
    return ESP_FAIL;
  }
  return ESP_OK;
}

int TcpServer::SendData(const uint8_t *data, size_t len) {
  if (ctx_.data_fd < 0 || !data || len == 0)
    return -1;

  // Best-effort non-blocking send
  int r = send(ctx_.data_fd, data, len, 0);
  if (r < 0) {
    if (errno == EWOULDBLOCK || errno == EAGAIN)
      return 0; // buffer full, drop or retry later (streaming logic: drop used
                // here)
    // CloseData(); // Optional: close on error? For now, just report error.
    return -1;
  }
  return r;
}

static inline const char *SkipSpace(const char *p) {
  while (*p && (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n'))
    ++p;
  return p;
}

// ---------------- Control line buffer helpers ----------------

void TcpServer::ResetLinebuf() { ctx_.line_len = 0; }

bool TcpServer::LinebufAdd(char b) {
  if (b == '\r')
    return false;

  if (b == '\n') {
    return ctx_.line_len > 0;
  }

  const size_t kCap = sizeof(ctx_.line_buf);
  const size_t kMax = (cfg_.max_line < (kCap - 1)) ? cfg_.max_line : (kCap - 1);

  if (ctx_.line_len >= kMax) {
    // line too long -> saturate (truncate), drop extra chars
    return false;
  }

  ctx_.line_buf[ctx_.line_len++] = (char)b;
  return false;
}

// parse "key=value" where value is u32 decimal or hex (0x..). returns true if
// found.
static bool FindU32KV(const char *line, const char *key, uint32_t &out) {
  const char *p = line;
  const size_t kLen = std::strlen(key);

  while (*p) {
    p = SkipSpace(p);
    if (!*p)
      break;

    // find key at token start
    if (std::strncmp(p, key, kLen) == 0 && p[kLen] == '=') {
      p += kLen + 1;

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

        if (d >= (unsigned)base)
          break;

        any = true;
        v = v * (uint64_t)base + (uint64_t)d;
        if (v > 0xFFFFFFFFull)
          return false;
        ++p;
      }

      if (!any)
        return false;

      out = (uint32_t)v;
      return true;
    }

    // skip token
    while (*p && *p != ' ' && *p != '\t')
      ++p;
  }

  return false;
}

static bool FindStrKV(const char *p, const char *key, char *out_buf,
                      size_t max_len) {
  if (!p || !key || !out_buf || max_len == 0)
    return false;

  const size_t kLen = std::strlen(key);

  while (*p) {
    p = SkipSpace(p);
    if (!*p)
      break;

    // find key at token start
    if (std::strncmp(p, key, kLen) == 0 && p[kLen] == '=') {
      p += kLen + 1;
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

// ---------------- Command handling ----------------

void TcpServer::HandleLine(const char *line) {
  if (!line)
    return;

  const char *p = SkipSpace(line);
  if (!*p)
    return;

  if (StartsWithCI(p, "BEGIN")) {
    uint32_t size = 0;
    uint32_t crc = 0;
    bool ok_size = FindU32KV(p, "size", size);
    (void)FindU32KV(p, "crc", crc);

    char target_str[32];
    Target target = Target::kStm32;
    if (FindStrKV(p, "target", target_str, sizeof(target_str))) {
      if (StartsWithCI(target_str, "esp32")) {
        target = Target::kEsp32;
      }
    }

    if (!ok_size || size == 0) {
      (void)SendCtrlLine("ERR bad_size\n");
      return;
    }

    Event e{};
    e.id = EventId::kBegin;
    e.begin.size = size;
    e.begin.crc = crc;
    e.begin.target = target;
    if (!PushEvent(e)) {
      (void)SendCtrlLine("ERR evt_queue_full\n");
      return;
    }

    (void)SendCtrlLine("OK\n");
    return;
  }

  if (StartsWithCI(p, "ABORT")) {
    Event e{};
    e.id = EventId::kAbort;
    (void)PushEvent(e);
    (void)SendCtrlLine("OK\n");
    return;
  }

  if (StartsWithCI(p, "RESET")) {
    Event e{};
    e.id = EventId::kReset;
    (void)PushEvent(e);
    (void)SendCtrlLine("OK\n");
    return;
  }

  if (StartsWithCI(p, "STATUS?") || StartsWithCI(p, "STATUS")) {
    Status st = GetStatus();
    char buf[128];
    std::snprintf(buf, sizeof(buf), "STATUS rx=%u total=%u state=%u err=%u\n",
                  (unsigned)st.rx, (unsigned)st.total, (unsigned)st.state,
                  (unsigned)st.err);
    (void)SendCtrlLine(buf);
    return;
  }

  (void)SendCtrlLine("ERR unknown_cmd\n");
}

// ---------------- Internal SM states ----------------

class TcpServer::StStopped : public IState<Ctx> {
public:
  const char *Name() const override { return "Stopped"; }
  void SetListen(IState<Ctx> *s) { listen_ = s; }
  IState<Ctx> *GetListen() const { return listen_; }

  void OnEnter(Ctx &ctx, SmTick) override {
    if (!ctx.self || !ctx.sm)
      return;
    ctx.self->CloseAll();
    ctx.line_len = 0;
    // stay here until Start() moves us; Start() will Start() from Listening.
  }

  void OnStep(Ctx &, SmTick) override {}
  void OnExit(Ctx &, SmTick) override {}

private:
  IState<Ctx> *listen_ = nullptr;
};

class TcpServer::StListening : public IState<Ctx> {
public:
  const char *Name() const override { return "Listening"; }
  void SetCtrl(IState<Ctx> *s) { ctrl_ = s; }

  void OnEnter(Ctx &ctx, SmTick) override {
    if (!ctx.self || !ctx.sm)
      return;

    // Ensure listen sockets exist
    if (ctx.ctrl_listen_fd < 0) {
      int fd = -1;
      if (TcpServer::MakeListenSocket(fd, ctx.self->cfg_.ctrl_port,
                                      ctx.self->cfg_.backlog,
                                      ctx.self->cfg_.nonblocking)) {
        ctx.ctrl_listen_fd = fd;
      }
    }
    if (ctx.data_listen_fd < 0) {
      int fd = -1;
      if (TcpServer::MakeListenSocket(fd, ctx.self->cfg_.data_port,
                                      ctx.self->cfg_.backlog,
                                      ctx.self->cfg_.nonblocking)) {
        ctx.data_listen_fd = fd;
      }
    }
  }

  void OnStep(Ctx &ctx, SmTick) override {
    if (!ctx.self || !ctx.sm)
      return;

    // Accept control first (session anchor)
    ctx.self->AcceptCtrl();
    // Accept data only if control exists (AcceptData_ enforces)
    ctx.self->AcceptData();

    if (ctx.ctrl_fd >= 0 && ctrl_) {
      ctx.sm->ReqTransition(*ctrl_);
    }
  }

  void OnExit(Ctx &, SmTick) override {}

private:
  IState<Ctx> *ctrl_ = nullptr;
};

class TcpServer::StCtrl : public IState<Ctx> {
public:
  const char *Name() const override { return "Ctrl"; }
  void SetListen(IState<Ctx> *s) { listen_ = s; }
  void SetCtrlData(IState<Ctx> *s) { ctrldata_ = s; }

  void OnEnter(Ctx &ctx, SmTick) override {
    if (!ctx.self)
      return;
    ctx.self->ResetLinebuf();
  }

  void OnStep(Ctx &ctx, SmTick) override {
    if (!ctx.self || !ctx.sm)
      return;

    // keep accepting data while control is up
    ctx.self->AcceptData();

    // pump control commands
    ctx.self->PumpCtrlRx();

    // ctrl might have dropped during pump
    if (ctx.ctrl_fd < 0) {
      if (listen_)
        ctx.sm->ReqTransition(*listen_);
      return;
    }

    if (ctx.data_fd >= 0 && ctrldata_) {
      ctx.sm->ReqTransition(*ctrldata_);
    }
  }

  void OnExit(Ctx &, SmTick) override {}

private:
  IState<Ctx> *listen_ = nullptr;
  IState<Ctx> *ctrldata_ = nullptr;
};

class TcpServer::StCtrlData : public IState<Ctx> {
public:
  const char *Name() const override { return "CtrlData"; }
  void SetListen(IState<Ctx> *s) { listen_ = s; }
  void SetCtrl(IState<Ctx> *s) { ctrl_ = s; }

  void OnEnter(Ctx &, SmTick) override {}

  void OnStep(Ctx &ctx, SmTick) override {
    if (!ctx.self || !ctx.sm)
      return;

    // pump both channels
    ctx.self->PumpCtrlRx();

    // ctrl might have dropped; CloseCtrl_ also closes data
    if (ctx.ctrl_fd < 0) {
      if (listen_)
        ctx.sm->ReqTransition(*listen_);
      return;
    }

    ctx.self->PumpDataRx();

    if (ctx.data_fd < 0) {
      if (ctrl_)
        ctx.sm->ReqTransition(*ctrl_);
      return;
    }
  }

  void OnExit(Ctx &, SmTick) override {}

private:
  IState<Ctx> *listen_ = nullptr;
  IState<Ctx> *ctrl_ = nullptr;
};

// ---------------- Finish Start(): wire states + start SM ----------------

esp_err_t TcpServer::Start() {
  if (running_)
    return ESP_OK;

  // reset internal state (same as PART 1)
  evt_head_ = evt_tail_ = 0;
  up_head_ = up_tail_ = 0;
  upload_overflow_ = false;
  upload_enabled_ = false;
  status_ = Status{};

  ctx_.ctrl_listen_fd = -1;
  ctx_.data_listen_fd = -1;
  ctx_.ctrl_fd = -1;
  ctx_.data_fd = -1;
  ctx_.ctrl_peer_ipv4 = 0;
  ctx_.line_len = 0;

  // Construct/wire state instances once (static lifetime is fine)
  static StStopped st_stopped;
  static StListening st_listen;
  static StCtrl st_ctrl;
  static StCtrlData st_ctrldata;

  s_stopped_ = &st_stopped;
  s_listen_ = &st_listen;
  s_ctrl_ = &st_ctrl;
  s_ctrldata_ = &st_ctrldata;

  // Wire transitions (only needs to happen once, but cheap to redo)
  if (!st_stopped.GetListen()) {
    st_stopped.SetListen(s_listen_);
    st_listen.SetCtrl(s_ctrl_);
    st_ctrl.SetListen(s_listen_);
    st_ctrl.SetCtrlData(s_ctrldata_);
    st_ctrldata.SetListen(s_listen_);
    st_ctrldata.SetCtrl(s_ctrl_);
  }

  running_ = true;

  // Start SM from Listening; Listening::OnEnter will open listen sockets.
  sm_.Start(*s_listen_, 0);

  // Give it one immediate step so sockets are created right away
  sm_.Step(0);

  // If listen sockets failed, stop and signal error
  if (ctx_.ctrl_listen_fd < 0 || ctx_.data_listen_fd < 0) {
    Stop();
    return ESP_FAIL;
  }

  return ESP_OK;
}