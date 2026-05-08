#pragma once

#include <cstddef>
#include <cstdint>

// Abstract transport for the Mavlink service. Selected at build time via the
// `Mavlink -> Transport` Kconfig choice (default WiFi UDP). The Mavlink class
// owns an IMavlinkTransport* and is agnostic to the underlying I/O — adding
// a new physical link (e.g. SiK telemetry over UART) is one new impl + one
// branch in system.cpp.
class IMavlinkTransport {
 public:
  virtual ~IMavlinkTransport() = default;

  // Non-blocking. Returns the number of bytes copied into `dst`, 0 if no
  // data available, <0 on error. Implementations may return a single
  // datagram (UDP) or a stream chunk (USB CDC); the MAVLink parser handles
  // either.
  virtual int Receive(uint8_t *dst, size_t max_len) = 0;

  // Returns the number of bytes actually written, <0 on error.
  virtual int Send(const uint8_t *data, size_t len) = 0;

  // True when the transport is ready to deliver outbound frames to a peer
  // (UDP: associated WiFi station present; USB CDC: USB host enumerated).
  virtual bool IsReady() const = 0;

  // Drop any latched peer state on hard reset signals. UDP forgets the
  // last-seen client; USB has no peer notion and may no-op.
  virtual void ClearPeer() = 0;
};
