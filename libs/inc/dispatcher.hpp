// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include <span>
#include <utility>

#include "message.hpp"

// The span does not own the table, so the entries have to outlive the
// Dispatcher; construct it from a static array, not a local one.
template <typename Context>
class Dispatcher {
 public:
  using Handler = void (*)(Context &ctx, const message::Packet &pkt);

  struct Entry {
    message::MsgId id;
    Handler handler;
  };

  constexpr explicit Dispatcher(std::span<const Entry> entries)
      : entries_(entries) {}

  // False means no entry matched, which is the caller's cue to treat the
  // packet as unknown rather than as handled.
  [[nodiscard]] bool Dispatch(Context &ctx, const message::Packet &pkt) const {
    for (const auto &[id, handler] : entries_) {
      if (std::to_underlying(id) == pkt.header.id && handler) {
        handler(ctx, pkt);
        return true;
      }
    }
    return false;
  }

 private:
  std::span<const Entry> entries_;
};
