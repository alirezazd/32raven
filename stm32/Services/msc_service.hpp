// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

#include "sdio.hpp"

class UsbCdc;
class LogService;
class SharedState;

// SCSI-over-BOT for the SD card: the host owns the volume for the whole
// session -- reads, writes, deletes, even a reformat -- and the firmware
// touches nothing until the mode ends. Block I/O is bounded per poll so a
// card stall cannot wedge the main loop past the watchdog.
class MscService {
 public:
  void Poll(uint32_t now_us);

  // Public for the same reason as MspService::PublishUsbStatus: the state
  // that held the session publishes this record on its way out.
  void PublishUsbStatus(uint32_t now_us);

  // Refuses while armed. Enabling hands the card to the host; disabling
  // remounts it and preallocates the next log.
  void SetMscMode(bool enabled);
  bool MscGranted() const { return granted_; }

 private:
  friend class System;
  void Init(UsbCdc &usb, LogService &log, SharedState &blackboard,
            Sdio &sd);

  enum class Bot : uint8_t {
    kWaitCbw,
    kDataIn,
    kDataOut,
    kDiscardOut,
    kStalledIn,
    kSendCsw,
  };

  struct __attribute__((packed)) Cbw {
    uint32_t signature;
    uint32_t tag;
    uint32_t data_length;
    uint8_t flags;
    uint8_t lun;
    uint8_t cb_length;
    uint8_t cb[16];
  };
  static_assert(sizeof(Cbw) == 31);

  struct __attribute__((packed)) Csw {
    uint32_t signature;
    uint32_t tag;
    uint32_t residue;
    uint8_t status;
  };
  static_assert(sizeof(Csw) == 13);

  void ResetBot();
  void HandleCbw();
  void ReplyFixed(std::span<const uint8_t> data);
  void FailCommand(uint8_t key, uint8_t asc);
  void QueueCsw(uint8_t status, uint32_t residue);
  void PollDataIn();
  void PollDataOut();
  void PollDiscard();

  UsbCdc *usb_ = nullptr;
  LogService *log_ = nullptr;
  SharedState *blackboard_ = nullptr;
  Sdio *sd_ = nullptr;
  bool initialized_ = false;
  bool granted_ = false;

  Bot bot_ = Bot::kWaitCbw;
  Cbw cbw_{};
  size_t cbw_fill_ = 0;
  Csw csw_{};
  size_t csw_sent_ = 0;

  // One block in flight either direction; BOT is half-duplex by design.
  alignas(4) uint8_t block_[Sdio::kBlockBytes] = {};
  size_t block_fill_ = 0;
  uint32_t xfer_lba_ = 0;
  uint32_t xfer_blocks_left_ = 0;
  size_t block_sent_ = 0;
  uint32_t discard_left_ = 0;

  // Fixed-reply staging (INQUIRY and friends): sized by the largest reply.
  uint8_t reply_[36] = {};
  size_t reply_len_ = 0;
  size_t reply_sent_ = 0;

  // REQUEST SENSE state, set on failure, consumed by the next read.
  uint8_t sense_key_ = 0;
  uint8_t sense_asc_ = 0;

  uint32_t last_usb_reset_count_ = 0;
  uint32_t last_msc_reset_count_ = 0;

  uint32_t blocks_read_ = 0;
  uint32_t blocks_written_ = 0;
};
