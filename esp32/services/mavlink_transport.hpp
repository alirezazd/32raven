// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include <cstdint>
#include <span>

// Swapped at run time by the state machine, not chosen at build time: the same
// Mavlink instance moves between the telem UART, UDP and USB CDC as states
// change. Implementations therefore differ in what a read returns and in
// whether a peer exists at all, which is what the methods below pin down.
class IMavlinkTransport {
 public:
  virtual ~IMavlinkTransport() = default;

  // Non-blocking: bytes copied, 0 when nothing is buffered, <0 on error. May
  // return a whole datagram (UDP) or part of a stream (USB CDC).
  virtual int Receive(std::span<uint8_t> dst) = 0;

  // Bytes written, <0 on error.
  virtual int Send(std::span<const uint8_t> bytes) = 0;

  // Ready means there is somewhere to send: UDP needs an associated station,
  // USB CDC an enumerated host.
  virtual bool IsReady() const = 0;

  // Forget the peer. UDP drops the last-seen client; USB has no peer and no-ops.
  virtual void ClearPeer() = 0;
};
