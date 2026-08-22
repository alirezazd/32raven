// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "msc_service.hpp"

#include <cstring>
#include <optional>

#include "error_code.hpp"
#include "log_service.hpp"
#include "panic.hpp"
#include "shared_state.hpp"
#include "usb_cdc.hpp"

namespace {

constexpr uint32_t kCbwSignature = 0x43425355u;
constexpr uint32_t kCswSignature = 0x53425355u;
constexpr uint8_t kCbwFlagDataIn = 0x80u;

constexpr uint8_t kScsiTestUnitReady = 0x00u;
constexpr uint8_t kScsiRequestSense = 0x03u;
constexpr uint8_t kScsiInquiry = 0x12u;
constexpr uint8_t kScsiModeSense6 = 0x1Au;
constexpr uint8_t kScsiStartStopUnit = 0x1Bu;
constexpr uint8_t kScsiPreventAllowRemoval = 0x1Eu;
constexpr uint8_t kScsiReadFormatCapacities = 0x23u;
constexpr uint8_t kScsiReadCapacity10 = 0x25u;
constexpr uint8_t kScsiRead10 = 0x28u;
constexpr uint8_t kScsiWrite10 = 0x2Au;
constexpr uint8_t kScsiSynchronizeCache = 0x35u;

constexpr uint8_t kSenseIllegalRequest = 0x05u;
constexpr uint8_t kSenseNotReady = 0x02u;
constexpr uint8_t kSenseMediumError = 0x03u;
constexpr uint8_t kAscInvalidCommand = 0x20u;
constexpr uint8_t kAscMediumNotPresent = 0x3Au;
constexpr uint8_t kAscWriteError = 0x0Cu;
constexpr uint8_t kAscReadError = 0x11u;
constexpr uint8_t kAscLbaOutOfRange = 0x21u;

// A write can park on the card's busy phase for hundreds of milliseconds, so
// PollDataOut starts at most one per tick; reads only cost ring drain time.
constexpr uint32_t kReadBlocksPerPoll = 4;

uint32_t BeU32(const uint8_t *p) {
  return (static_cast<uint32_t>(p[0]) << 24) |
         (static_cast<uint32_t>(p[1]) << 16) |
         (static_cast<uint32_t>(p[2]) << 8) | static_cast<uint32_t>(p[3]);
}

uint16_t BeU16(const uint8_t *p) {
  return static_cast<uint16_t>((static_cast<uint16_t>(p[0]) << 8) | p[1]);
}

void PutBeU32(uint8_t *p, uint32_t v) {
  p[0] = static_cast<uint8_t>(v >> 24);
  p[1] = static_cast<uint8_t>(v >> 16);
  p[2] = static_cast<uint8_t>(v >> 8);
  p[3] = static_cast<uint8_t>(v);
}

}  // namespace

void MscService::Init(UsbCdc &usb, LogService &log, SharedState &blackboard,
                      Sdio &sd) {
  if (initialized_) {
    Panic(ErrorCode::Stm32::kMscServiceReinit);
  }
  usb_ = &usb;
  log_ = &log;
  blackboard_ = &blackboard;
  sd_ = &sd;
  initialized_ = true;
}

void MscService::SetMscMode(bool enabled) {
  if (!initialized_) {
    return;
  }
  if (enabled && blackboard_->IsArmed()) {
    enabled = false;
  }
  if (enabled == granted_) {
    return;
  }

  if (enabled) {
    // ReleaseCard first: a host enumerating mid-close races the log's close.
    log_->ReleaseCard();
    usb_->SetClassMode(UsbCdc::ClassMode::kMsc);
    usb_->SetAttached(true);
    ResetBot();
    granted_ = true;
    return;
  }

  usb_->SetAttached(false);
  usb_->SetClassMode(UsbCdc::ClassMode::kCdc);
  log_->RemountAfterMsc();
  granted_ = false;
}

void MscService::PublishUsbStatus(uint32_t now_us) {
  // Block counts reuse the msp_ frame fields: same "data moved" meaning.
  UsbStatusData next{};
  next.attached = usb_->IsAttached();
  next.configured = usb_->IsConfigured();
  next.msc_active = granted_;
  next.msp_requests = blocks_written_;
  next.msp_replies = blocks_read_;

  const UsbStatusData &current = blackboard_->GetUsbStatus();
  if (current.attached == next.attached &&
      current.configured == next.configured &&
      current.msc_active == next.msc_active &&
      current.msp_requests == next.msp_requests &&
      current.msp_replies == next.msp_replies) {
    return;
  }
  next.timestamp_us = now_us;
  blackboard_->UpdateUsbStatus(next);
}

void MscService::ResetBot() {
  bot_ = Bot::kWaitCbw;
  cbw_fill_ = 0;
  csw_sent_ = 0;
  block_fill_ = 0;
  block_sent_ = 0;
  xfer_blocks_left_ = 0;
  discard_left_ = 0;
  reply_len_ = 0;
  reply_sent_ = 0;
  while (usb_->Read()) {
  }
}

void MscService::QueueCsw(uint8_t status, uint32_t residue) {
  csw_.signature = kCswSignature;
  csw_.tag = cbw_.tag;
  csw_.residue = residue;
  csw_.status = status;
  csw_sent_ = 0;
  bot_ = Bot::kSendCsw;
}

void MscService::FailCommand(uint8_t key, uint8_t asc) {
  sense_key_ = key;
  sense_asc_ = asc;
  if (cbw_.data_length == 0u) {
    QueueCsw(1u, 0u);
    return;
  }
  if ((cbw_.flags & kCbwFlagDataIn) != 0u) {
    // BOT failed-data-in: stall IN, wait for the host to clear, then the CSW.
    usb_->StallBulkIn();
    QueueCsw(1u, cbw_.data_length);
    csw_sent_ = 0;
    bot_ = Bot::kStalledIn;
    return;
  }
  discard_left_ = cbw_.data_length;
  QueueCsw(1u, cbw_.data_length);
  bot_ = Bot::kDiscardOut;
}

void MscService::ReplyFixed(std::span<const uint8_t> data) {
  size_t len = data.size();
  if (len > cbw_.data_length) {
    len = cbw_.data_length;
  }
  std::memcpy(reply_, data.data(), len);
  reply_len_ = len;
  reply_sent_ = 0;
  xfer_blocks_left_ = 0;
  bot_ = Bot::kDataIn;
}

void MscService::HandleCbw() {
  const uint8_t op = cbw_.cb[0];
  const bool card_ok = (*sd_).CardPresent();

  switch (op) {
    case kScsiTestUnitReady:
      if (card_ok) {
        QueueCsw(0u, 0u);
      } else {
        FailCommand(kSenseNotReady, kAscMediumNotPresent);
      }
      return;

    case kScsiStartStopUnit:
    case kScsiPreventAllowRemoval:
    case kScsiSynchronizeCache:
      QueueCsw(0u, 0u);
      return;

    case kScsiInquiry: {
      static constexpr uint8_t kInquiry[36] = {
          0x00, 0x80, 0x02, 0x02, 0x1F, 0x00, 0x00, 0x00,
          '3',  '2',  'R',  'a',  'v',  'e',  'n',  ' ',  // vendor, 8
          'S',  'D',  ' ',  'C',  'a',  'r',  'd',  ' ',  // product, 16
          'L',  'o',  'g',  ' ',  ' ',  ' ',  ' ',  ' ',
          '1',  '.',  '0',  ' ',  // revision, 4
      };
      ReplyFixed(kInquiry);
      return;
    }

    case kScsiRequestSense: {
      uint8_t sense[18] = {};
      sense[0] = 0x70;
      sense[2] = sense_key_;
      sense[7] = 10;
      sense[12] = sense_asc_;
      sense_key_ = 0;
      sense_asc_ = 0;
      ReplyFixed(sense);
      return;
    }

    case kScsiModeSense6: {
      static constexpr uint8_t kModeSense[4] = {3, 0, 0x00, 0};
      ReplyFixed(kModeSense);
      return;
    }

    case kScsiReadFormatCapacities: {
      uint8_t caps[12] = {};
      caps[3] = 8;
      PutBeU32(&caps[4], (*sd_).BlockCount());
      caps[8] = 0x02;  // formatted media
      caps[10] = static_cast<uint8_t>(Sdio::kBlockBytes >> 8);
      ReplyFixed(caps);
      return;
    }

    case kScsiReadCapacity10: {
      uint8_t cap[8] = {};
      PutBeU32(&cap[0], (*sd_).BlockCount() - 1u);
      PutBeU32(&cap[4], Sdio::kBlockBytes);
      ReplyFixed(cap);
      return;
    }

    case kScsiRead10:
    case kScsiWrite10: {
      const uint32_t lba = BeU32(&cbw_.cb[2]);
      const uint32_t count = BeU16(&cbw_.cb[7]);
      if (!card_ok) {
        FailCommand(kSenseNotReady, kAscMediumNotPresent);
        return;
      }
      if (count == 0u) {
        QueueCsw(0u, cbw_.data_length);
        return;
      }
      if ((lba + count) > (*sd_).BlockCount() ||
          cbw_.data_length != count * Sdio::kBlockBytes) {
        FailCommand(kSenseIllegalRequest, kAscLbaOutOfRange);
        return;
      }
      xfer_lba_ = lba;
      xfer_blocks_left_ = count;
      block_fill_ = 0;
      block_sent_ = Sdio::kBlockBytes;  // forces the first block read
      reply_len_ = 0;
      bot_ = (op == kScsiRead10) ? Bot::kDataIn : Bot::kDataOut;
      return;
    }

    default:
      FailCommand(kSenseIllegalRequest, kAscInvalidCommand);
      return;
  }
}

void MscService::PollDataIn() {
  // Pushed only when the whole remainder fits: a split would put a short
  // packet mid-transfer, and a short packet is how a bulk transfer ends.
  if (reply_len_ > 0u) {
    if (usb_->TxFree() >= reply_len_ - reply_sent_) {
      reply_sent_ +=
          usb_->Send(&reply_[reply_sent_], reply_len_ - reply_sent_);
    }
    if (reply_sent_ == reply_len_) {
      const uint32_t residue =
          cbw_.data_length - static_cast<uint32_t>(reply_len_);
      reply_len_ = 0;
      QueueCsw(0u, residue);
    }
    return;
  }

  uint32_t blocks_this_poll = 0;
  while (true) {
    if (block_sent_ < Sdio::kBlockBytes) {
      // Align to whole packets; a partial one would end the read early.
      const size_t want = Sdio::kBlockBytes - block_sent_;
      const size_t free = usb_->TxFree();
      const size_t aligned = free - (free % UsbCdc::kBulkMaxPacketBytes);
      block_sent_ +=
          usb_->Send(&block_[block_sent_], (want < aligned) ? want : aligned);
      if (block_sent_ < Sdio::kBlockBytes) {
        return;  // TX ring full; USB will drain it
      }
    }
    if (xfer_blocks_left_ == 0u) {
      QueueCsw(0u, 0u);
      return;
    }
    if (blocks_this_poll == kReadBlocksPerPoll) {
      return;
    }
    if (!(*sd_).ReadBlocks(xfer_lba_, block_)) {
      usb_->StallBulkIn();
      sense_key_ = kSenseMediumError;
      sense_asc_ = kAscReadError;
      QueueCsw(1u, xfer_blocks_left_ * Sdio::kBlockBytes);
      bot_ = Bot::kStalledIn;
      return;
    }
    ++blocks_read_;
    ++blocks_this_poll;
    ++xfer_lba_;
    --xfer_blocks_left_;
    block_sent_ = 0;
  }
}

void MscService::PollDataOut() {
  while (block_fill_ < Sdio::kBlockBytes) {
    const std::optional<uint8_t> byte = usb_->Read();
    if (!byte) {
      break;
    }
    block_[block_fill_++] = byte.value();
  }
  if (block_fill_ < Sdio::kBlockBytes) {
    return;
  }

  if (!(*sd_).WriteBlocks(xfer_lba_, block_)) {
    sense_key_ = kSenseMediumError;
    sense_asc_ = kAscWriteError;
    discard_left_ = (xfer_blocks_left_ - 1u) * Sdio::kBlockBytes;
    QueueCsw(1u, discard_left_);
    bot_ = (discard_left_ > 0u) ? Bot::kDiscardOut : Bot::kSendCsw;
    block_fill_ = 0;
    return;
  }
  ++blocks_written_;
  ++xfer_lba_;
  --xfer_blocks_left_;
  block_fill_ = 0;
  if (xfer_blocks_left_ == 0u) {
    QueueCsw(0u, 0u);
  }
}

void MscService::PollDiscard() {
  while (discard_left_ > 0u && usb_->Read()) {
    --discard_left_;
  }
  if (discard_left_ == 0u) {
    bot_ = Bot::kSendCsw;
  }
}

void MscService::Poll(uint32_t now_us) {
  if (!granted_) {
    return;
  }
  PublishUsbStatus(now_us);

  if (usb_->ResetCount() != last_usb_reset_count_ ||
      usb_->MscResetCount() != last_msc_reset_count_) {
    last_usb_reset_count_ = usb_->ResetCount();
    last_msc_reset_count_ = usb_->MscResetCount();
    ResetBot();
  }
  if (!usb_->IsConfigured()) {
    return;
  }

  switch (bot_) {
    case Bot::kWaitCbw: {
      auto *raw = reinterpret_cast<uint8_t *>(&cbw_);
      while (cbw_fill_ < sizeof(Cbw)) {
        const std::optional<uint8_t> byte = usb_->Read();
        if (!byte) {
          break;
        }
        raw[cbw_fill_++] = byte.value();
      }
      if (cbw_fill_ < sizeof(Cbw)) {
        return;
      }
      cbw_fill_ = 0;
      if (cbw_.signature != kCbwSignature || cbw_.cb_length == 0u ||
          cbw_.cb_length > 16u) {
        // BOT requires a stall here until the host issues a reset.
        usb_->StallBulkIn();
        ResetBot();
        return;
      }
      HandleCbw();
      return;
    }
    case Bot::kDataIn:
      PollDataIn();
      return;
    case Bot::kDataOut:
      PollDataOut();
      return;
    case Bot::kDiscardOut:
      PollDiscard();
      return;
    case Bot::kStalledIn:
      if (!usb_->BulkInHalted()) {
        bot_ = Bot::kSendCsw;
      }
      return;
    case Bot::kSendCsw: {
      const auto *raw = reinterpret_cast<const uint8_t *>(&csw_);
      if (usb_->TxFree() >= sizeof(Csw) - csw_sent_) {
        csw_sent_ += usb_->Send(&raw[csw_sent_], sizeof(Csw) - csw_sent_);
      }
      if (csw_sent_ == sizeof(Csw)) {
        bot_ = Bot::kWaitCbw;
      }
      return;
    }
  }
}
