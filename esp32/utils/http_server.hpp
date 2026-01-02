#pragma once
#include <cstddef>
#include <cstdint>

extern "C" {
#include "esp_err.h"
}

class HttpServer {
public:
  static HttpServer &GetInstance() {
    static HttpServer inst;
    return inst;
  }

  struct Config {
    uint16_t port = 80;
    uint16_t ctrl_port = 32768; // internal control socket port used by httpd
    int max_uri_handlers = 8;
    int max_open_sockets = 4;
    int recv_wait_timeout_s = 15; // seconds
    int send_wait_timeout_s = 15; // seconds
    bool lru_purge_enable = true;
  };

  esp_err_t Start();
  void Stop();

  bool Running() const { return running_; }

  // ---- SM-facing API (SM consumes these) ----

  enum class EventId : uint8_t { kNone = 0, kBegin, kAbort, kReset };

  struct BeginArgs {
    uint32_t size = 0;
    uint32_t crc = 0;
  };

  struct Event {
    EventId id = EventId::kNone;
    BeginArgs begin{};
  };

  // Pop one pending event (non-blocking). Returns true if an event was popped.
  bool PopEvent(Event &out);

  // Upload session control (SM owns policy)
  void SetUploadEnabled(bool en);
  bool UploadEnabled() const { return upload_enabled_; }

  // Binary upload bytes buffered from HTTP POST /api/upload
  // Returns bytes copied, 0 if none available.
  int ReadUpload(uint8_t *dst, size_t max_len);

  bool UploadOverflowed() const { return upload_overflow_; }
  void ClearUploadOverflow() { upload_overflow_ = false; }

  // Status data that SM can update and HTTP can serve
  struct Status {
    uint32_t rx = 0;
    uint32_t total = 0;
    uint32_t state = 0; // you can map ProgramState to an int later
  };

  void SetStatus(const Status &s);
  Status GetStatus() const;

private:
  friend class System;
  HttpServer() = default;
  ~HttpServer() = default;
  HttpServer(const HttpServer &) = delete;
  HttpServer &operator=(const HttpServer &) = delete;

  void Init(const Config &cfg);

  // HTTP handlers
  static esp_err_t HPing(void *req);
  static esp_err_t HStatus(void *req);
  static esp_err_t HBegin(void *req);
  static esp_err_t HAbort(void *req);
  static esp_err_t HReset(void *req);
  static esp_err_t HUpload(void *req);

  // internal helpers
  bool PushEvent(const Event &e);
  bool WriteUpload(const uint8_t *data, size_t len);

  Config cfg_{};

  // esp_http_server handle (opaque)
  void *server_ = nullptr;
  bool running_ = false;

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

  // lightweight lock
  mutable unsigned int lock_ = 0;
};
