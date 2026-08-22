// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>

#include "ring_buffer.hpp"
#include "stm32_limits.hpp"

// Bare-metal USB CDC-ACM device on the STM32F407's OTG FS core (Synopsys DWC2).
//
// Two silicon facts shape the whole driver:
//   * OTG FS needs exactly 48 MHz from PLLQ; changing the PLL invalidates USB.
//   * ID (PA10) and VBUS (PA9) are wired to USART1, so the core is forced into
//     device mode with VBUS sensing disabled. Cable state is inferred from the
//     SOF frame counter instead.

struct UsbCdcConfig {
  uint16_t vendor_id;
  uint16_t product_id;
  const char *manufacturer;
  const char *product;
  const char *product_msc;
};

class UsbCdc {
 public:
  // Full-speed bulk and control endpoints are capped at 64 bytes by the USB
  // 2.0 spec, not by anything this driver chooses.
  static constexpr size_t kBulkMaxPacketBytes = 64;
  static constexpr size_t kEp0MaxPacketBytes = 64;

  // Largest single frame either USB dialect can emit. The payload is a
  // 256-byte ESC page for four-way and an MSP v1 maximum for MSP; both
  // services size their frame buffers from this and static_assert against it.
  static constexpr size_t kMaxPayloadBytes = 256;
  static constexpr size_t kFrameOverheadBytes = 16;
  static constexpr size_t kMaxFrameBytes =
      kMaxPayloadBytes + kFrameOverheadBytes;

  // +1 is RingBuffer's full/empty slot. 1 KB refilled per 1 kHz tick is the
  // full-speed bulk ceiling; RX needs a spare bulk packet or the driver will
  // not re-arm the OUT endpoint.
  static constexpr size_t kTxRingSize = 1024u + 1u;
  static_assert(kTxRingSize > kMaxFrameBytes);
  static constexpr size_t kRxRingSize =
      kMaxFrameBytes + kBulkMaxPacketBytes + 1;

  // Switched only while detached: a live host has cached the descriptors.
  enum class ClassMode : uint8_t { kCdc, kMsc };

  static UsbCdc &GetInstance();

  void SetClassMode(ClassMode mode);
  ClassMode GetClassMode() const { return mode_; }

  // MSC's failed-command signalling: BOT stalls the bulk IN endpoint and the
  // host clears it with CLEAR_FEATURE. CDC never stalls a data endpoint.
  void StallBulkIn();
  bool BulkInHalted() const { return ep_in_halted_; }

  // Bumped by the Bulk-Only Mass Storage Reset class request. The consumer
  // compares and resynchronises; nothing is cleared from interrupt context.
  uint32_t MscResetCount() const { return msc_reset_count_; }

  size_t TxFree() const { return tx_ring_.Capacity() - tx_ring_.Available(); }

  // Samples the SOF frame counter. Unplugging raises no interrupt with VBUS
  // sensing off, and the core's suspend flag was measured never to set on
  // detach either, so a stalled counter is the only evidence the cable is
  // gone. Skip this and the enumerated state outlives the cable.
  void Poll(uint32_t now_us);

  size_t Send(const uint8_t *data, size_t len);
  [[nodiscard]] std::optional<uint8_t> Read();

  // Drives the D+ pull-up. Detached when ESC configuration is not authorised,
  // so the host sees no COM port rather than an unresponsive one.
  void SetAttached(bool attached);

  bool IsAttached() const { return attached_; }
  bool IsHostPresent() const { return host_present_; }
  bool IsConfigured() const { return configured_ && host_present_; }

  // Host selected our configuration and asserted DTR.
  bool IsConnected() const { return configured_ && dtr_ && host_present_; }

  uint32_t ResetCount() const { return reset_count_; }

  void IrqHandler();

 private:
  friend class System;
  void Init(const UsbCdcConfig &config);

  UsbCdc() = default;
  ~UsbCdc() = default;
  UsbCdc(const UsbCdc &) = delete;
  UsbCdc &operator=(const UsbCdc &) = delete;

  // CDC's line-coding record, byte-for-byte as it crosses the wire.
  struct __attribute__((packed)) LineCoding {
    uint32_t baud;
    uint8_t stop_bits;
    uint8_t parity;
    uint8_t data_bits;
  };

  struct __attribute__((packed)) SetupPacket {
    uint8_t bm_request_type;
    uint8_t b_request;
    uint16_t w_value;
    uint16_t w_index;
    uint16_t w_length;
  };

  void CoreInit();
  void DeviceInit();
  void FlushFifo(uint32_t rstctl_write, uint32_t busy_mask);
  void FlushTxFifo(uint32_t fifo_num);
  void FlushRxFifo();

  void OnReset();
  void OnEnumDone();
  void OnRxFifoLevel();
  void OnInEndpoint();
  void OnOutEndpoint();

  void ReadFifo(uint8_t *dest, uint16_t len);
  void WriteFifo(uint8_t ep_num, const uint8_t *src, uint16_t len);

  void HandleSetup();
  bool HandleStandardRequest(const SetupPacket &setup);
  bool HandleClassRequest(const SetupPacket &setup);
  uint16_t BuildStringDescriptor(uint8_t index);
  void ControlSend(const uint8_t *data, uint16_t len);
  void ControlInPacket();
  void ControlSendZlp();
  void ControlStall();
  void PrepareEp0Setup();
  void PrepareEp0Out(uint16_t len);

  void OpenDataEndpoints();
  void CloseDataEndpoints();
  void ArmBulkOut();
  void PumpTx();
  void SetBulkHalt(bool in, bool halt);

  UsbCdcConfig cfg_{};
  bool initialized_ = false;

  RingBuffer<uint8_t, kRxRingSize> rx_ring_;
  RingBuffer<uint8_t, kTxRingSize> tx_ring_;

  SetupPacket setup_{};
  LineCoding line_coding_{115200u, 0u, 0u, 8u};

  // Descriptors are all static, so only the cursor needs storing.
  const uint8_t *ctrl_in_ptr_ = nullptr;
  uint16_t ctrl_in_remaining_ = 0;
  bool ctrl_in_zlp_ = false;
  bool ctrl_out_is_line_coding_ = false;

  uint8_t bulk_out_buf_[kBulkMaxPacketBytes]{};
  // Reassembles a packet the TX ring's wrap split in two.
  uint8_t tx_stage_[kBulkMaxPacketBytes]{};
  uint8_t ctrl_out_buf_[kEp0MaxPacketBytes]{};

  // One buffer suffices: only one control transfer is ever live.
  uint8_t string_desc_buf_[stm32_limits::kUsbCdcStringDescriptorBytes]{};
  uint8_t status_reply_[2]{};
  uint8_t max_lun_reply_ = 0;

  ClassMode mode_ = ClassMode::kCdc;
  volatile bool ep_in_halted_ = false;
  volatile bool ep_out_halted_ = false;
  volatile uint32_t msc_reset_count_ = 0;

  volatile bool attached_ = false;
  volatile bool bulk_out_stalled_ = false;
  volatile bool configured_ = false;
  volatile bool dtr_ = false;
  volatile bool tx_in_flight_ = false;
  bool tx_zlp_pending_ = false;
  uint16_t last_tx_len_ = 0;
  uint8_t current_configuration_ = 0;

  volatile uint32_t reset_count_ = 0;

  bool host_present_ = false;
  uint32_t last_frame_number_ = 0;
  uint32_t last_frame_change_us_ = 0;
};
