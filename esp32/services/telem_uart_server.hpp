#pragma once

#include <cstddef>
#include <cstdint>

#include "mavlink_transport.hpp"
#include "uart.hpp"

// MAVLink transport over the telem UART (UART1). Stateless adapter that
// presents the IMavlinkTransport contract on top of the UartTelem driver
// singleton, which is owned and configured by System (Component::kTelemUart).
// Intended for tethered SiK telemetry radios (Holybro / RFD900 / etc.) but
// works for any transparent serial peer.
class TelemUartServer : public IMavlinkTransport {
 public:
  static TelemUartServer &GetInstance() {
    static TelemUartServer instance;
    return instance;
  }

  int Receive(uint8_t *dst, size_t max_len) override;
  int Send(const uint8_t *data, size_t len) override;
  bool IsReady() const override;
  void ClearPeer() override;

 private:
  TelemUartServer() = default;
  ~TelemUartServer() = default;
  TelemUartServer(const TelemUartServer &) = delete;
  TelemUartServer &operator=(const TelemUartServer &) = delete;
};
