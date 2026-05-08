#include "telem_uart_server.hpp"

int TelemUartServer::Receive(uint8_t *dst, size_t max_len) {
  if (dst == nullptr || max_len == 0) {
    return 0;
  }
  return UartTelem::GetInstance().Read(dst, max_len, /*timeout_ms=*/0);
}

int TelemUartServer::Send(const uint8_t *data, size_t len) {
  if (data == nullptr || len == 0) {
    return 0;
  }
  return UartTelem::GetInstance().Write(data, len);
}

bool TelemUartServer::IsReady() const {
  // SiK / generic MAVLink-over-UART peers do not signal readiness over the
  // wire — the modem drains the TX FIFO whenever it has air bandwidth. Treat
  // the link as always-ready once UartTelem has been Init'd by System; if it
  // hasn't, Read/Write fall through to ESP-IDF errors that the caller
  // tolerates as "no data / send drop."
  return true;
}

void TelemUartServer::ClearPeer() {
  // No peer state — point-to-point with the radio modem.
}
