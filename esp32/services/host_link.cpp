// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "host_link.hpp"

#include <algorithm>
#include <cctype>
#include <charconv>
#include <cstdio>
#include <cstring>
#include <string_view>

extern "C" {
#include "esp_log.h"
}

static constexpr const char *kTag = "host_link";

// SM-facing API (queue/buffer/status)

std::optional<HostLink::Event> HostLink::PopEvent() {
  Event out;
  if (!evt_q_.Pop(out)) {
    return std::nullopt;
  }
  return out;
}

bool HostLink::PushEvent(const Event &e) {
  Event stamped = e;
  stamped.origin = this;
  return evt_q_.Push(stamped);
}

void HostLink::QueueSimpleCommand(EventId id) {
  Event e{};
  e.id = id;
  if (!PushEvent(e)) {
    SendCtrlLine("ERR evt_queue_full\n");
    return;
  }
  SendCtrlLine("OK\n");
}

bool HostLink::TakeLinkDrop() {
  const bool out = link_dropped_;
  link_dropped_ = false;
  return out;
}

void HostLink::ClearLinkDrop() { link_dropped_ = false; }

void HostLink::MarkDrop() { link_dropped_ = true; }

void HostLink::CloseDataRx() {
  data_rx_open_ = false;
  DiscardDataRx();
}

void HostLink::BeginTransfer(const BeginArgs &begin) {
  data_rx_open_ = true;
  DiscardDataRx();
  begin_ = begin;
  status_ = Status{};
  status_.total = begin.size;
}

void HostLink::EndTransfer() {
  CloseDataRx();
  begin_ = BeginArgs{};
  status_ = Status{};
}

void HostLink::ResetLine() {
  line_len_ = 0;
  line_overflow_ = false;
}

void HostLink::ResetSession() {
  EndTransfer();
  ResetLine();
  evt_q_.Clear();
  link_dropped_ = false;
}

void HostLink::SetStatus(const Status &s) { status_ = s; }
HostLink::Status HostLink::GetStatus() const { return status_; }

// Control line assembly

void HostLink::FeedCtrl(uint8_t byte) {
  const LineFeed fed = LinebufAdd(static_cast<char>(byte));
  if (fed == LineFeed::kTruncated) {
    SendCtrlLine("ERR line_too_long\n");
    line_len_ = 0;
    return;
  }
  if (fed == LineFeed::kComplete) {
    line_buf_[line_len_] = '\0';
    ESP_LOGI(kTag, "Cmd line: %s", line_buf_);
    HandleLine(line_buf_);
    line_len_ = 0;
  }
}

HostLink::LineFeed HostLink::LinebufAdd(char b) {
  if (b == '\r') return LineFeed::kIncomplete;
  if (b == '\n') {
    const bool truncated = line_overflow_;
    line_overflow_ = false;
    if (truncated) return LineFeed::kTruncated;
    return (line_len_ > 0) ? LineFeed::kComplete : LineFeed::kIncomplete;
  }
  if (line_len_ >= kMaxLineBytes) {
    line_overflow_ = true;
    return LineFeed::kIncomplete;
  }
  line_buf_[line_len_++] = b;
  return LineFeed::kIncomplete;
}

static inline const char *SkipSpace(const char *p) {
  while (*p && (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n')) ++p;
  return p;
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
void HostLink::HandleLine(const char *line) {
  if (!line) return;

  const char *p = SkipSpace(line);
  if (!*p) return;

  if (TokenEqCI(p, "BEGIN")) {
    const std::optional<uint32_t> parsed_size = FindU32KV(p, "size");
    // Not optional: the programmer's verify proves the flash holds what
    // arrived, and only the host knows what it meant to send.
    const std::optional<uint32_t> parsed_crc = FindU32KV(p, "crc");

    if (!parsed_size || *parsed_size == 0) {
      SendCtrlLine("ERR bad_size\n");
      return;
    }
    if (!parsed_crc) {
      SendCtrlLine("ERR bad_crc\n");
      return;
    }

    Event e{};
    e.id = EventId::kBegin;
    e.begin.size = *parsed_size;
    e.begin.crc = *parsed_crc;
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
      while (name[n] && name[n] != ' ' && name[n] != '\r' && name[n] != '\n' &&
             n < sizeof(e.log_name) - 1) {
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
