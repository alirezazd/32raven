// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "telem_uart_server.hpp"

int TelemUartServer::Receive(std::span<uint8_t> dst) {
  if (dst.empty()) {
    return 0;
  }
  return UartTelem::GetInstance().ReadBytes(dst, /*timeout_ms=*/0);
}

int TelemUartServer::Send(std::span<const uint8_t> bytes) {
  if (bytes.empty()) {
    return 0;
  }
  return UartTelem::GetInstance().WriteBytes(bytes);
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
