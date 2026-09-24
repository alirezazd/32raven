// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "fc_link.hpp"

#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <optional>

#include "checksum.hpp"
#include "error_code.hpp"
#include "panic.hpp"
#include "system.hpp"
#include "uart.hpp"

FcLink &FcLink::GetInstance() {
  static FcLink instance;
  return instance;
}

void FcLink::Init(Uart1 &uart, SharedState &blackboard) {
  if (initialized_) {
    Panic(ErrorCode::Stm32::kFcLinkReinit);
  }
  initialized_ = true;
  uart_ = &uart;
  blackboard_ = &blackboard;
}

std::optional<message::Packet> FcLink::PopPacket(uint32_t now_us) {
  while (rx_budget_left_ > 0) {
    const std::optional<uint8_t> next = uart_->ReadByte();
    if (!next) {
      break;
    }
    const uint8_t byte = next.value();
    rx_budget_left_--;
    message::Header &header = rx_pkt_.header;
    switch (rx_state_) {
      case RxState::kMagic1:
        if (byte == message::kMagic1) rx_state_ = RxState::kMagic2;
        break;
      case RxState::kMagic2:
        rx_state_ = byte == message::kMagic2 ? RxState::kId : RxState::kMagic1;
        break;
      case RxState::kId:
        header.id = byte;
        rx_state_ = RxState::kLen;
        break;
      case RxState::kLen:
        header.len = byte;
        rx_idx_ = 0;
        if (!message::IsPayloadLengthValid(
                static_cast<message::MsgId>(header.id), header.len)) {
          rx_state_ = RxState::kMagic1;
          break;
        }
        rx_state_ = header.len > 0 ? RxState::kPayload : RxState::kCrc1;
        break;
      case RxState::kPayload:
        rx_pkt_.payload[rx_idx_++] = byte;
        if (rx_idx_ >= header.len) rx_state_ = RxState::kCrc1;
        break;
      case RxState::kCrc1:
        rx_pkt_.crc = byte;
        rx_state_ = RxState::kCrc2;
        break;
      case RxState::kCrc2: {
        rx_pkt_.crc |= static_cast<uint16_t>(byte << 8);
        rx_state_ = RxState::kMagic1;

        uint16_t crc = 0;
        crc = checksum::XModemUpdate(crc, message::kMagic1);
        crc = checksum::XModemUpdate(crc, message::kMagic2);
        crc = checksum::XModemUpdate(crc, header.id);
        crc = checksum::XModemUpdate(crc, header.len);
        for (uint8_t i = 0; i < header.len; ++i) {
          crc = checksum::XModemUpdate(crc, rx_pkt_.payload[i]);
        }

        FcLinkData link = blackboard_->GetFcLink();
        // Before the id/len check: a corrupt id rejected as unknown would leave
        // the fault that caused it uncounted.
        if (crc != rx_pkt_.crc) {
          link.checksum_failures++;
          blackboard_->UpdateFcLink(link);
          break;
        }
        if (!message::IsPacketValid(header.id, rx_pkt_.payload, header.len)) {
          break;
        }
        // Only a frame that passed its CRC is the peer being heard from.
        link.timestamp_us = now_us;
        blackboard_->UpdateFcLink(link);
        return rx_pkt_;
      }
    }
  }
  return std::nullopt;
}

void FcLink::BeginRx() { rx_budget_left_ = kRxByteBudget; }

void FcLink::FlushTx() {
  flushed_ = true;
  // Never pop more than the UART can take: popped bytes have left tx_rb_, and
  // Send is all or nothing, so a refused chunk would be lost.
  uint8_t chunk[64];
  const size_t room = std::min(sizeof(chunk), uart_->TxFree());
  size_t n = 0;
  while (n < room && tx_rb_.Pop(chunk[n])) {
    ++n;
  }
  if (n > 0) {
    (void)uart_->Send(chunk, n);
  }
}

bool FcLink::Send(const message::Packet &pkt) {
  uint8_t buf[sizeof(message::Packet)];
  size_t len = message::Serialize((message::MsgId)pkt.header.id,
                                  {pkt.payload, pkt.header.len}, buf);
  if (len == 0) {
    return false;
  }

  const size_t free_bytes = tx_rb_.Capacity() - tx_rb_.Available();
  if (len > free_bytes) {
    return false;
  }

  return tx_rb_.PushBlock(buf, len) == len;
}

void FcLink::SendLog(const char *format, ...) {
  char buf[message::kMaxLogTextPayload + 1];
  va_list args;
  va_start(args, format);
  int len = vsnprintf(buf, sizeof(buf), format, args);
  va_end(args);

  if (len > 0) {
    if (len > message::kMaxLogTextPayload) {
      len = message::kMaxLogTextPayload;
    }
    message::Packet pkt;
    pkt.header.id = (uint8_t)message::MsgId::kLog;
    pkt.header.len = (uint8_t)len;
    memcpy(pkt.payload, buf, len);
    Send(pkt);

    // Until the first FlushTx, a boot-time log would sit in the ring and vanish
    // if init wedges. The UART is named directly: Init may not have run yet.
    if (!flushed_) {
      auto &uart = Uart1::GetInstance();
      uint8_t byte = 0;
      uint8_t chunk[64];
      size_t n = 0;
      // Unguarded, unlike FlushTx: this may be the last code to run, so a chunk
      // the UART refuses would never be sent anyway; what fits beats stopping.
      while (tx_rb_.Pop(byte)) {
        chunk[n++] = byte;
        if (n == sizeof(chunk)) {
          (void)uart.Send(chunk, n);
          n = 0;
        }
      }
      if (n > 0) {
        (void)uart.Send(chunk, n);
      }
      // Panic masks the TX interrupt and writes the UART raw, so bytes still
      // pending here would splice mid-packet into the panic stream.
      const uint32_t drain_start = System::GetInstance().Time().Micros();
      while (uart.TxPending() > 0 &&
             (System::GetInstance().Time().Micros() - drain_start) < 20000u) {
      }
    }
  }
}
