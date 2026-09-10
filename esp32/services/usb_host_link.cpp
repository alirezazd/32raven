// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "usb_host_link.hpp"

#include <algorithm>
#include <cstring>
#include <span>

#include "esp32_config.hpp"
#include "usb_cdc_server.hpp"

// The driver's ring is what overflows, and OK goes out only after the whole
// previous chunk has been read out of it, so a chunk need only fit it once.
static_assert(UsbHostLink::kChunkBytes <= kUsbCdcServerConfig.rx_buffer_bytes,
              "a chunk must fit the USB driver ring -- raise "
              "ESP32_USB_CDC_SERVER_RX_BUFFER_BYTES");

UsbHostLink &UsbHostLink::GetInstance() {
  static UsbHostLink instance;
  return instance;
}

void UsbHostLink::Init(UsbCdcServer &usb) { usb_ = &usb; }

void UsbHostLink::Start() {
  if (running_) return;
  ResetSession();
  running_ = true;
  attached_ = usb_->HostAttached();
}

void UsbHostLink::Stop() { running_ = false; }

void UsbHostLink::Poll() {
  if (!running_) return;
  const bool attached = usb_->HostAttached();
  if (attached_ && !attached) {
    MarkDrop();
  }
  attached_ = attached;

  // Until the image is complete the stream is the page's, through ReadDataRx.
  if (DataRxOpen() && !image_complete_) return;

  uint8_t buf[kMaxLineBytes];
  const int got = usb_->Receive(buf);
  for (int i = 0; i < got; ++i) {
    FeedCtrl(buf[i]);
  }
}

size_t UsbHostLink::ReadDataRx(std::span<uint8_t> dst) {
  if (!DataRxOpen() || image_complete_) return 0;
  const uint32_t size = Begin().size;
  const size_t take =
      std::min(dst.size(), static_cast<size_t>(size - received_));
  const int got = usb_->Receive(dst.first(take));
  if (got <= 0) return 0;
  received_ += static_cast<uint32_t>(got);
  image_complete_ = received_ == size;
  // The host sends whole chunks and waits, so a boundary here is the end of
  // one of its writes, and none of it is left in the driver's ring.
  if (received_ % kChunkBytes == 0 || image_complete_) {
    SendCtrlLine("OK\n");
  }
  return static_cast<size_t>(got);
}

void UsbHostLink::DiscardDataRx() {
  if (!image_complete_) Drain();
  image_complete_ = false;
  received_ = 0;
}

void UsbHostLink::Drain() {
  uint8_t sink[128];
  constexpr size_t kReads =
      (kUsbCdcServerConfig.rx_buffer_bytes / sizeof(sink)) + 1;
  for (size_t i = 0; i < kReads && usb_->Receive(sink) > 0; ++i) {
  }
}

void UsbHostLink::SendCtrlLine(const char *line) {
  if (line == nullptr) return;
  const size_t n = std::strlen(line);
  if (n == 0) return;
  (void)usb_->Send({reinterpret_cast<const uint8_t *>(line), n});
}
