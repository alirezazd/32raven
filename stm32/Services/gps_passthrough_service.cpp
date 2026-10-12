// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "gps_passthrough_service.hpp"

#include <algorithm>
#include <cstddef>
#include <optional>

#include "error_code.hpp"
#include "panic.hpp"

namespace {

// The M10's UART range; a host rate outside it is not applied.
constexpr uint32_t kMinBaud = 4800;
constexpr uint32_t kMaxBaud = 921600;

// Per direction per poll: 921600 baud is ~92 bytes per 1 kHz tick.
constexpr size_t kBytesPerPoll = 256;

}  // namespace

GpsPassthroughService &GpsPassthroughService::GetInstance() {
  static GpsPassthroughService instance;
  return instance;
}

void GpsPassthroughService::Init(UsbCdc &usb, Uart2 &uart,
                                 SharedState &blackboard) {
  if (initialized_) {
    Panic(ErrorCode::Stm32::kGpsPassthroughReinit);
  }
  usb_ = &usb;
  uart_ = &uart;
  blackboard_ = &blackboard;
  initialized_ = true;
}

void GpsPassthroughService::SetGpsPassthroughMode(bool enabled) {
  if (enabled && blackboard_->IsArmed()) {
    enabled = false;
  }
  if (enabled == granted_) {
    return;
  }

  if (enabled) {
    uart_baud_ = 0;
    // Leftovers from an earlier session's host are not for the GPS.
    while (usb_->Read()) {
    }
    usb_->SetAttached(true);
    granted_ = true;
    return;
  }

  usb_->SetAttached(false);
  granted_ = false;
}

void GpsPassthroughService::Poll(uint32_t now_us) {
  if (!granted_) {
    return;
  }
  PublishUsbStatus(now_us);
  FollowHostBaud();
  PumpToGps();
  PumpToHost();
}

void GpsPassthroughService::PublishUsbStatus(uint32_t now_us) {
  // Byte counts reuse the msp_ frame fields: same "data moved" meaning.
  UsbStatusData next{};
  next.attached = usb_->IsAttached();
  next.configured = usb_->IsConfigured();
  next.port_open = usb_->IsConnected();
  next.mode =
      granted_ ? message::UsbMode::kGpsPassthrough : message::UsbMode::kNone;
  next.msp_requests = to_gps_bytes_;
  next.msp_replies = to_host_bytes_;

  const UsbStatusData &current = blackboard_->GetUsbStatus();
  if (current.attached == next.attached &&
      current.configured == next.configured &&
      current.port_open == next.port_open && current.mode == next.mode &&
      current.msp_requests == next.msp_requests &&
      current.msp_replies == next.msp_replies) {
    return;
  }
  next.timestamp_us = now_us;
  blackboard_->UpdateUsbStatus(next);
}

void GpsPassthroughService::FollowHostBaud() {
  const uint32_t baud = usb_->LineCodingBaud();
  if (baud == uart_baud_ || baud < kMinBaud || baud > kMaxBaud) {
    return;
  }
  // Bytes still queued would go out at the new rate, garbled.
  if (uart_->TxPending() != 0u) {
    return;
  }
  uart_->SetBaudRate(baud);
  uart_baud_ = baud;
}

void GpsPassthroughService::PumpToGps() {
  uint8_t chunk[kBytesPerPoll];
  const size_t room = std::min(uart_->TxFree(), sizeof(chunk));
  size_t len = 0;
  while (len < room) {
    const std::optional<uint8_t> byte = usb_->Read();
    if (!byte) {
      break;
    }
    chunk[len++] = *byte;
  }
  // Sized to TxFree, so the all-or-nothing Send takes it whole.
  if (len != 0u && uart_->Send(chunk, len) == Outcome::kOk) {
    to_gps_bytes_ += static_cast<uint32_t>(len);
  }
}

void GpsPassthroughService::PumpToHost() {
  uint8_t chunk[kBytesPerPoll];
  const size_t room = std::min(usb_->TxFree(), sizeof(chunk));
  size_t len = 0;
  while (len < room) {
    const std::optional<uint8_t> byte = uart_->ReadByte();
    if (!byte) {
      break;
    }
    chunk[len++] = *byte;
  }
  if (len != 0u) {
    to_host_bytes_ += static_cast<uint32_t>(usb_->Send(chunk, len));
  }
}
