// tcp_server.hpp
#pragma once
#include <cstddef>
#include <cstdint>

extern "C" {
#include "esp_err.h"
}

#include "programmer.hpp"
#include "state_machine.hpp"

class TcpServer {
public:
  static TcpServer &GetInstance() {
    static TcpServer inst;
    return inst;
  }

  struct Config {
    uint16_t ctrl_port = 9000;
    uint16_t data_port = 9001;
    int backlog = 1;

    bool nonblocking = true;

    // Keepalive (0 disables)
    int keepalive_idle_s = 10;
    int keepalive_intvl_s = 5;
    int keepalive_cnt = 3;

    // Parser limits
    size_t max_line = 160;
  };

  esp_err_t Start();
  void Stop();

  bool Running() const { return running_; }

  // Tick entry point (non-blocking)
  void Poll(SmTick now);

  enum class EventId : uint8_t {
    kNone = 0,
    kBegin,
    kAbort,
    kReset,
    kCtrlUp,
    kCtrlDown,
    kDataUp,
    kDataDown,
  };

  using Target = Programmer::Target;

  struct BeginArgs {
    uint32_t size = 0;
    uint32_t crc = 0;
    Target target = Target::kStm32;
  };

  struct Event {
    EventId id = EventId::kNone;
    BeginArgs begin{};
  };

  bool PopEvent(Event &out);

  // Upload policy/backpressure owned by App SM
  void SetUploadEnabled(bool en);
  bool UploadEnabled() const { return upload_enabled_; }

  // Binary bytes buffered from DATA socket
  int ReadUpload(uint8_t *dst, size_t max_len);

  bool UploadOverflowed() const { return upload_overflow_; }
  void ClearUploadOverflow() { upload_overflow_ = false; }

  // Status snapshot (App updates, client may query via STATUS?)
  struct Status {
    uint32_t rx = 0;
    uint32_t total = 0;
    uint32_t state = 0;
    uint32_t err = 0;
  };

  void SetStatus(const Status &s);
  Status GetStatus() const;

  // Optional: send a single line response to CTRL client
  esp_err_t SendCtrlLine(const char *line);

  // Send raw data to DATA client
  int SendData(const uint8_t *data, size_t len);

private:
  friend class System;
  TcpServer() = default;
  ~TcpServer() = default;
  TcpServer(const TcpServer &) = delete;
  TcpServer &operator=(const TcpServer &) = delete;

  // ---------- Internal SM wiring ----------
  struct Ctx {
    TcpServer *self = nullptr;
    StateMachine<Ctx> *sm = nullptr;

    // sockets
    int ctrl_listen_fd = -1;
    int data_listen_fd = -1;
    int ctrl_fd = -1;
    int data_fd = -1;

    // peer bind (optional, for “single client session”)
    uint32_t ctrl_peer_ipv4 = 0;

    // parser
    char line_buf[192]{};
    size_t line_len = 0;

    // timing (if you want accept throttling etc.)
    SmTick now = 0;
  };

  class StStopped;
  class StListening;
  class StCtrl;
  class StCtrlData;

  void Init(const Config &cfg);

  // Helpers used by states
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
  bool PushEvent(const Event &e);
  bool WriteUpload(const uint8_t *data, size_t len);

  // Line parser helper
  void ResetLinebuf();
  bool LinebufAdd(char c);

  // Socket helpers
  static bool SetNonblock(int fd);
  static bool SetKeepalive(int fd, const Config &cfg);
  static bool MakeListenSocket(int &out_fd, uint16_t port, int backlog,
                               bool nonblocking);

  Config cfg_{};
  bool running_ = false;

  // Internal SM instance + states (no heap)
  Ctx ctx_{};
  StateMachine<Ctx> sm_{ctx_};

  StStopped *s_stopped_ = nullptr;
  StListening *s_listen_ = nullptr;
  StCtrl *s_ctrl_ = nullptr;
  StCtrlData *s_ctrldata_ = nullptr;

  // ---- event queue (tiny, bounded) ----
  static constexpr size_t kEvtCap = 8;
  Event evt_q_[kEvtCap]{};
  size_t evt_head_ = 0;
  size_t evt_tail_ = 0;

  // ---- upload ring buffer ----
  static constexpr size_t kUpCap = 8192;
  uint8_t up_[kUpCap]{};
  size_t up_head_ = 0;
  size_t up_tail_ = 0;
  bool upload_overflow_ = false;
  bool upload_enabled_ = false;

  // status
  Status status_{};
};
