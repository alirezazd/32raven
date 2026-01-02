#include "http_server.hpp"

extern "C" {
#include "esp_http_server.h"
#include "esp_log.h"
#include "freertos/task.h"
}

#include <cctype>
#include <cstdio>
#include <cstring>

static constexpr char kTag[] = "http_server";

// ----- tiny lock helpers (cheap spinlock) -----
// Replace with portMUX if you prefer; this is minimal and works fine for short
// critical sections.
static inline void LockTake(unsigned int &l) {
  while (__atomic_test_and_set(&l, __ATOMIC_ACQUIRE)) {
    taskYIELD();
  }
}
static inline void LockGive(unsigned int &l) {
  __atomic_clear(&l, __ATOMIC_RELEASE);
}

static inline size_t RbUsed(size_t head, size_t tail, size_t cap) {
  return (tail >= head) ? (tail - head) : (cap - head + tail);
}
static inline size_t RbFree(size_t head, size_t tail, size_t cap) {
  // keep 1 byte gap
  return (cap - 1) - RbUsed(head, tail, cap);
}

void HttpServer::Init(const Config &cfg) { cfg_ = cfg; }

esp_err_t HttpServer::Start() {
  if (running_)
    return ESP_OK;

  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  cfg.server_port = cfg_.port;
  cfg.ctrl_port = cfg_.ctrl_port;
  cfg.max_uri_handlers = cfg_.max_uri_handlers;
  cfg.max_open_sockets = cfg_.max_open_sockets;
  cfg.recv_wait_timeout = cfg_.recv_wait_timeout_s;
  cfg.send_wait_timeout = cfg_.send_wait_timeout_s;
  cfg.lru_purge_enable = cfg_.lru_purge_enable;

  httpd_handle_t h = nullptr;
  esp_err_t err = httpd_start(&h, &cfg);
  if (err != ESP_OK) {
    ESP_LOGE(kTag, "httpd_start failed: %s", esp_err_to_name(err));
    return err;
  }

  server_ = (void *)h;

  // register endpoints (URI + method + handler)
  // :contentReference[oaicite:1]{index=1}
  static httpd_uri_t uri_ping = {
      .uri = "/api/ping",
      .method = HTTP_GET,
      .handler = [](httpd_req_t *r) { return HttpServer::HPing((void *)r); },
      .user_ctx = nullptr};

  static httpd_uri_t uri_status = {
      .uri = "/api/status",
      .method = HTTP_GET,
      .handler = [](httpd_req_t *r) { return HttpServer::HStatus((void *)r); },
      .user_ctx = nullptr};

  static httpd_uri_t uri_begin = {
      .uri = "/api/begin",
      .method = HTTP_POST,
      .handler = [](httpd_req_t *r) { return HttpServer::HBegin((void *)r); },
      .user_ctx = nullptr};

  static httpd_uri_t uri_abort = {
      .uri = "/api/abort",
      .method = HTTP_POST,
      .handler = [](httpd_req_t *r) { return HttpServer::HAbort((void *)r); },
      .user_ctx = nullptr};

  static httpd_uri_t uri_reset = {
      .uri = "/api/reset",
      .method = HTTP_POST,
      .handler = [](httpd_req_t *r) { return HttpServer::HReset((void *)r); },
      .user_ctx = nullptr};

  static httpd_uri_t uri_upload = {
      .uri = "/api/upload",
      .method = HTTP_POST,
      .handler = [](httpd_req_t *r) { return HttpServer::HUpload((void *)r); },
      .user_ctx = nullptr};

  httpd_register_uri_handler(h, &uri_ping);
  httpd_register_uri_handler(h, &uri_status);
  httpd_register_uri_handler(h, &uri_begin);
  httpd_register_uri_handler(h, &uri_abort);
  httpd_register_uri_handler(h, &uri_reset);
  httpd_register_uri_handler(h, &uri_upload);

  running_ = true;
  ESP_LOGI(kTag, "HTTP server started on port %u", (unsigned)cfg_.port);
  return ESP_OK;
}

void HttpServer::Stop() {
  if (!running_)
    return;
  httpd_handle_t h = (httpd_handle_t)server_;
  if (h)
    httpd_stop(h);
  server_ = nullptr;
  running_ = false;

  LockTake(lock_);
  evt_head_ = evt_tail_ = 0;
  up_head_ = up_tail_ = 0;
  upload_overflow_ = false;
  upload_enabled_ = false;
  LockGive(lock_);
  ESP_LOGI(kTag, "HTTP server stopped");
}

bool HttpServer::PushEvent(const Event &e) {
  LockTake(lock_);
  size_t next = (evt_tail_ + 1) % kEvtCap;
  if (next == evt_head_) {
    LockGive(lock_);
    return false;
  }
  evt_q_[evt_tail_] = e;
  evt_tail_ = next;
  LockGive(lock_);
  return true;
}

bool HttpServer::PopEvent(Event &out) {
  LockTake(lock_);
  if (evt_head_ == evt_tail_) {
    LockGive(lock_);
    return false;
  }
  out = evt_q_[evt_head_];
  evt_head_ = (evt_head_ + 1) % kEvtCap;
  LockGive(lock_);
  return true;
}

void HttpServer::SetUploadEnabled(bool en) {
  LockTake(lock_);
  upload_enabled_ = en;
  if (!upload_enabled_) {
    up_head_ = up_tail_ = 0;
    upload_overflow_ = false;
  }
  LockGive(lock_);
}

int HttpServer::ReadUpload(uint8_t *dst, size_t max_len) {
  if (!dst || max_len == 0)
    return 0;

  LockTake(lock_);
  size_t n = 0;
  while (n < max_len && up_head_ != up_tail_) {
    dst[n++] = up_[up_head_];
    up_head_ = (up_head_ + 1) % kUpCap;
  }
  LockGive(lock_);
  return (int)n;
}

bool HttpServer::WriteUpload(const uint8_t *data, size_t len) {
  LockTake(lock_);
  if (!upload_enabled_) {
    LockGive(lock_);
    return false;
  }
  if (len > RbFree(up_head_, up_tail_, kUpCap)) {
    upload_overflow_ = true;
    LockGive(lock_);
    return false;
  }
  for (size_t i = 0; i < len; ++i) {
    up_[up_tail_] = data[i];
    up_tail_ = (up_tail_ + 1) % kUpCap;
  }
  LockGive(lock_);
  return true;
}

void HttpServer::SetStatus(const Status &s) {
  LockTake(lock_);
  status_ = s;
  LockGive(lock_);
}

HttpServer::Status HttpServer::GetStatus() const {
  LockTake(lock_);
  Status s = status_;
  LockGive(lock_);
  return s;
}

// ----- Minimal JSON helper for begin -----
// Accepts {"size":123,"crc":0x1234} or {"size":123,"crc":4660}
// Very small and forgiving (not a full JSON parser).
static bool JsonFindU32(const char *buf, size_t len, const char *key,
                        uint32_t &out) {
  const char *end = buf + len;
  const char *p = buf;
  size_t keylen = std::strlen(key);

  while (p + keylen < end) {
    if (std::memcmp(p, key, keylen) == 0) {
      p += keylen;
      while (p < end &&
             (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r' || *p == ':'))
        ++p;

      int base = 10;
      if (p + 2 < end && p[0] == '0' && (p[1] == 'x' || p[1] == 'X')) {
        base = 16;
        p += 2;
      }

      uint64_t v = 0;
      bool any = false;
      while (p < end) {
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
    ++p;
  }
  return false;
}

esp_err_t HttpServer::HPing(void *reqp) {
  httpd_req_t *req = (httpd_req_t *)reqp;
  httpd_resp_set_type(req, "text/plain");
  httpd_resp_sendstr(req, "OK\n");
  return ESP_OK;
}

esp_err_t HttpServer::HStatus(void *reqp) {
  httpd_req_t *req = (httpd_req_t *)reqp;
  HttpServer &s = HttpServer::GetInstance();
  Status st = s.GetStatus();

  char body[128];
  snprintf(body, sizeof(body), "{\"rx\":%u,\"total\":%u,\"state\":%u}\n",
           (unsigned)st.rx, (unsigned)st.total, (unsigned)st.state);

  httpd_resp_set_type(req, "application/json");
  httpd_resp_sendstr(req, body);
  return ESP_OK;
}

esp_err_t HttpServer::HBegin(void *reqp) {
  httpd_req_t *req = (httpd_req_t *)reqp;
  HttpServer &s = HttpServer::GetInstance();

  // read body (small JSON)
  const int kMax = 256;
  char buf[kMax];
  int to_read = (req->content_len < (size_t)(kMax - 1)) ? (int)req->content_len
                                                        : (kMax - 1);
  int got = 0;

  while (got < to_read) {
    int r = httpd_req_recv(req, buf + got, to_read - got);
    if (r <= 0)
      return ESP_FAIL;
    got += r;
  }
  buf[got] = '\0';

  uint32_t size = 0, crc = 0;
  bool ok_size = JsonFindU32(buf, (size_t)got, "\"size\"", size);
  (void)JsonFindU32(buf, (size_t)got, "\"crc\"", crc);

  if (!ok_size || size == 0) {
    httpd_resp_set_status(req, "400 Bad Request");
    httpd_resp_sendstr(req, "{\"ok\":false,\"err\":\"bad_size\"}\n");
    return ESP_OK;
  }

  Event e{};
  e.id = EventId::kBegin;
  e.begin.size = size;
  e.begin.crc = crc;

  if (!s.PushEvent(e)) {
    httpd_resp_set_status(req, "503 Service Unavailable");
    httpd_resp_sendstr(req, "{\"ok\":false,\"err\":\"queue_full\"}\n");
    return ESP_OK;
  }

  httpd_resp_set_type(req, "application/json");
  httpd_resp_sendstr(req, "{\"ok\":true}\n");
  return ESP_OK;
}

esp_err_t HttpServer::HAbort(void *reqp) {
  httpd_req_t *req = (httpd_req_t *)reqp;
  HttpServer &s = HttpServer::GetInstance();

  Event e{};
  e.id = EventId::kAbort;
  if (!s.PushEvent(e)) {
    httpd_resp_set_status(req, "503 Service Unavailable");
    httpd_resp_sendstr(req, "{\"ok\":false,\"err\":\"queue_full\"}\n");
    return ESP_OK;
  }

  httpd_resp_sendstr(req, "{\"ok\":true}\n");
  return ESP_OK;
}

esp_err_t HttpServer::HReset(void *reqp) {
  httpd_req_t *req = (httpd_req_t *)reqp;
  HttpServer &s = HttpServer::GetInstance();

  Event e{};
  e.id = EventId::kReset;
  if (!s.PushEvent(e)) {
    httpd_resp_set_status(req, "503 Service Unavailable");
    httpd_resp_sendstr(req, "{\"ok\":false,\"err\":\"queue_full\"}\n");
    return ESP_OK;
  }

  httpd_resp_sendstr(req, "{\"ok\":true}\n");
  return ESP_OK;
}

esp_err_t HttpServer::HUpload(void *reqp) {
  httpd_req_t *req = (httpd_req_t *)reqp;
  HttpServer &s = HttpServer::GetInstance();

  if (!s.UploadEnabled()) {
    httpd_resp_set_status(req, "409 Conflict");
    httpd_resp_sendstr(req, "{\"ok\":false,\"err\":\"not_ready\"}\n");
    return ESP_OK;
  }

  uint8_t buf[1024];
  size_t remaining = req->content_len;

  while (remaining > 0) {
    int want = (remaining > sizeof(buf)) ? (int)sizeof(buf) : (int)remaining;
    int r = httpd_req_recv(req, (char *)buf, want);
    if (r <= 0)
      return ESP_FAIL;

    // Backpressure: if buffer full, fail fast (simple).
    // Alternative: wait and retry a few times.
    if (!s.WriteUpload(buf, (size_t)r)) {
      httpd_resp_set_status(req, "503 Service Unavailable");
      httpd_resp_sendstr(req, "{\"ok\":false,\"err\":\"buffer_full\"}\n");
      return ESP_OK;
    }

    remaining -= (size_t)r;
  }

  httpd_resp_sendstr(req, "{\"ok\":true}\n");
  return ESP_OK;
}
