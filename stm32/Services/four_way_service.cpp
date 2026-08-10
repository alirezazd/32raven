// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "four_way_service.hpp"

#include <span>
#include <utility>

#include "checksum.hpp"
#include "dshot_codec.hpp"
#include "error_code.hpp"
#include "panic.hpp"

namespace {

constexpr uint8_t kLocalEscape = 0x2F;   // host  -> flight controller
constexpr uint8_t kRemoteEscape = 0x2E;  // flight controller -> host

constexpr uint8_t kCmdInterfaceTestAlive = 0x30;
constexpr uint8_t kCmdProtocolGetVersion = 0x31;
constexpr uint8_t kCmdInterfaceGetName = 0x32;
constexpr uint8_t kCmdInterfaceGetVersion = 0x33;
constexpr uint8_t kCmdInterfaceExit = 0x34;
constexpr uint8_t kCmdDeviceReset = 0x35;
constexpr uint8_t kCmdDeviceInitFlash = 0x37;
constexpr uint8_t kCmdDeviceEraseAll = 0x38;
constexpr uint8_t kCmdDevicePageErase = 0x39;
constexpr uint8_t kCmdDeviceRead = 0x3A;
constexpr uint8_t kCmdDeviceWrite = 0x3B;
constexpr uint8_t kCmdDeviceReadEeprom = 0x3D;
constexpr uint8_t kCmdDeviceWriteEeprom = 0x3E;
constexpr uint8_t kCmdInterfaceSetMode = 0x3F;
constexpr uint8_t kCmdDeviceVerify = 0x40;

// Long enough that a host pausing between the bytes of one frame is never
// mistaken for a dead one, short enough to recover well inside the seconds a
// configurator waits before giving up on the port.
constexpr uint32_t kFrameStallTimeoutUs = 250000u;

constexpr uint8_t kProtocolVersion = 107;
constexpr uint8_t kInterfaceVersionMain = 20;
constexpr uint8_t kInterfaceVersionSub = 4;
constexpr char kInterfaceName[] = "m4wFCIntf";

// The bootloader dialects, of which only imARM_BLB is implemented here. The
// value is accepted and dropped because Connect derives the real mode from the
// device signature; rejecting the others here would refuse modes every shipping
// flight controller accepts.
constexpr uint8_t kModeSilBlb = 1;
constexpr uint8_t kModeArmBlb = 4;

}  // namespace

void FourWayService::Init(UsbCdc &usb, EscBootloader &bootloader) {
  if (initialized_) {
    Panic(ErrorCode::Stm32::kFourWayServiceReinit);
  }
  initialized_ = true;
  usb_ = &usb;
  bootloader_ = &bootloader;
}

void FourWayService::Enter() {
  active_ = true;
  bootloader_->HoldAll();
  Reset();
}

void FourWayService::Exit() {
  bootloader_->ReleaseAll();
  active_ = false;
  Reset();
}

void FourWayService::Reset() {
  parse_ = Parse::kEscape;
  param_index_ = 0;
  crc_ = 0;
}

void FourWayService::Poll(uint32_t now_us) {
  // Between frames there is no partial state to lose, and the escape byte
  // resynchronises on its own.
  if (parse_ == Parse::kEscape) {
    return;
  }

  if (feed_count_ != last_feed_count_) {
    last_feed_count_ = feed_count_;
    last_progress_us_ = now_us;
    return;
  }

  if ((now_us - last_progress_us_) > kFrameStallTimeoutUs) {
    ++stall_count_;
    Reset();
  }
}

void FourWayService::Feed(uint8_t byte) {
  ++feed_count_;

  switch (parse_) {
    case Parse::kEscape:
      if (byte == kLocalEscape) {
        crc_ = checksum::XModemUpdate(0, byte);
        parse_ = Parse::kCommand;
      }
      break;

    case Parse::kCommand:
      command_ = byte;
      crc_ = checksum::XModemUpdate(crc_, byte);
      parse_ = Parse::kAddressHi;
      break;

    case Parse::kAddressHi:
      address_ = static_cast<uint16_t>(byte) << 8;
      crc_ = checksum::XModemUpdate(crc_, byte);
      parse_ = Parse::kAddressLo;
      break;

    case Parse::kAddressLo:
      address_ |= byte;
      crc_ = checksum::XModemUpdate(crc_, byte);
      parse_ = Parse::kLength;
      break;

    case Parse::kLength:
      param_count_ = (byte == 0u) ? 256u : byte;
      crc_ = checksum::XModemUpdate(crc_, byte);
      param_index_ = 0;
      parse_ = Parse::kParams;
      break;

    case Parse::kParams:
      params_[param_index_++] = byte;
      crc_ = checksum::XModemUpdate(crc_, byte);
      if (param_index_ >= param_count_) {
        parse_ = Parse::kCrcHi;
      }
      break;

    case Parse::kCrcHi:
      crc_received_ = static_cast<uint16_t>(byte) << 8;
      parse_ = Parse::kCrcLo;
      break;

    case Parse::kCrcLo:
      crc_received_ |= byte;
      if (crc_received_ == crc_) {
        Dispatch();
      } else {
        ++crc_error_count_;
        reply_len_ = 0;
        Respond(Ack::kInvalidCrc);
      }
      Reset();
      break;
  }
}

void FourWayService::Dispatch() {
  ++request_count_;
  last_command_ = command_;
  reply_len_ = 0;

  switch (command_) {
    // Only meaningful once a device is attached; before that the host is
    // asking whether this interface answers, which it just did.
    case kCmdInterfaceTestAlive:
      if (bootloader_->IsConnected() && !bootloader_->KeepAlive()) {
        bootloader_->Disconnect();
        Respond(Ack::kDeviceGeneralError);
        return;
      }
      Respond(Ack::kOk);
      return;

    case kCmdProtocolGetVersion:
      ReplyBuf()[reply_len_++] = kProtocolVersion;
      Respond(Ack::kOk);
      return;

    case kCmdInterfaceGetName:
      for (size_t i = 0; i < sizeof(kInterfaceName) - 1u; ++i) {
        ReplyBuf()[reply_len_++] = static_cast<uint8_t>(kInterfaceName[i]);
      }
      Respond(Ack::kOk);
      return;

    case kCmdInterfaceGetVersion:
      ReplyBuf()[reply_len_++] = kInterfaceVersionMain;
      ReplyBuf()[reply_len_++] = kInterfaceVersionSub;
      Respond(Ack::kOk);
      return;

    // Acknowledged before the teardown, because Exit hands the motor pins back
    // to TIM1 and the reply has nothing to do with them.
    case kCmdInterfaceExit:
      Respond(Ack::kOk);
      Exit();
      return;

    case kCmdInterfaceSetMode:
      if (param_count_ < 1u || params_[0] < kModeSilBlb ||
          params_[0] > kModeArmBlb) {
        Respond(Ack::kInvalidParam);
        return;
      }
      Respond(Ack::kOk);
      return;

    case kCmdDeviceInitFlash: {
      if (param_count_ < 1u || params_[0] >= DShotCodec::kMotorCount) {
        Respond(Ack::kInvalidChannel);
        return;
      }
      selected_esc_ = params_[0];

      EscBootloader::DeviceInfo info{};
      if (!bootloader_->Connect(selected_esc_, info)) {
        Respond(Ack::kDeviceGeneralError);
        return;
      }
      // Low byte first. Transposed, this is not a corrupt reply but a valid one
      // naming an MCU that does not exist, and the host drops the ESC silently.
      ReplyBuf()[reply_len_++] = info.signature_lo;
      ReplyBuf()[reply_len_++] = info.signature_hi;
      ReplyBuf()[reply_len_++] = info.boot_version;
      ReplyBuf()[reply_len_++] = info.interface_mode;
      Respond(Ack::kOk);
      return;
    }

    case kCmdDeviceReset: {
      if (param_count_ < 1u || params_[0] >= DShotCodec::kMotorCount) {
        Respond(Ack::kInvalidChannel);
        return;
      }
      selected_esc_ = params_[0];
      // A 1 in the address low byte asks for the line to be pulled down as
      // well, which is what stops the ESC re-entering its bootloader.
      bootloader_->Reset(selected_esc_, (address_ & 0xFFu) == 1u);
      Respond(Ack::kOk);
      return;
    }
    case kCmdDeviceRead: {
      const uint16_t len = (params_[0] == 0u) ? kMaxParams : params_[0];
      if (!bootloader_->ReadFlash(address_, ReplyBuf(), len)) {
        Respond(Ack::kDeviceGeneralError);
        return;
      }
      reply_len_ = len;
      Respond(Ack::kOk);
      return;
    }

    case kCmdDevicePageErase: {
      if (param_count_ < 1u) {
        Respond(Ack::kInvalidParam);
        return;
      }
      const uint8_t page = params_[0];
      if (page < EscBootloader::kFirstErasablePage) {
        Respond(Ack::kInvalidParam);
        return;
      }
      if (!bootloader_->PageErase(page)) {
        Respond(Ack::kDeviceGeneralError);
        return;
      }
      ReplyBuf()[reply_len_++] = page;
      Respond(Ack::kOk);
      return;
    }

    case kCmdDeviceWrite: {
      // Erasing the bootloader is not the only way to lose it: flash writes
      // clear bits, so programming over a live page corrupts it just as well.
      if (address_ < EscBootloader::kFirstWritableAddress) {
        Respond(Ack::kInvalidParam);
        return;
      }
      const auto params = std::span{params_}.first(param_count_);
      if (!bootloader_->WriteFlash(address_, params)) {
        Respond(Ack::kDeviceGeneralError);
        return;
      }
      Respond(Ack::kOk);
      return;
    }

    case kCmdDeviceVerify: {
      const auto params = std::span{params_}.first(param_count_);
      switch (bootloader_->VerifyFlash(address_, params)) {
        case EscBootloader::VerifyResult::kOk:
          Respond(Ack::kOk);
          return;
        case EscBootloader::VerifyResult::kMismatch:
          Respond(Ack::kVerifyError);
          return;
        case EscBootloader::VerifyResult::kFailed:
          Respond(Ack::kDeviceGeneralError);
          return;
      }
      return;
    }

    // SiLabs and Atmel paths, which no ARM target serves. The two eeprom
    // commands answer differently on purpose: configurators read that pairing
    // as "wrong device family" rather than "this ESC failed".
    case kCmdDeviceEraseAll:
    case kCmdDeviceReadEeprom:
      Respond(Ack::kInvalidCommand);
      return;

    case kCmdDeviceWriteEeprom:
      Respond(Ack::kDeviceGeneralError);
      return;

    default:
      Respond(Ack::kInvalidCommand);
      return;
  }
}

void FourWayService::Respond(Ack ack) {
  // Zero length means 256 parameters, so an empty reply cannot be encoded --
  // the host would wait for 256 bytes that never come. Ack-only responses
  // carry one padding byte instead.
  if (reply_len_ == 0u) {
    ReplyBuf()[0] = 0u;
    reply_len_ = 1u;
  }

  // Dispatch staged the payload at kHeaderBytes already, so only the bytes
  // around it are written here.
  frame_[0] = kRemoteEscape;
  frame_[1] = command_;
  frame_[2] = static_cast<uint8_t>(address_ >> 8);
  frame_[3] = static_cast<uint8_t>(address_ & 0xFFu);
  frame_[4] = static_cast<uint8_t>(reply_len_ & 0xFFu);

  size_t n = kHeaderBytes + reply_len_;
  frame_[n++] = std::to_underlying(ack);

  const uint16_t crc = checksum::XModem(std::span{frame_}.first(n));
  frame_[n++] = static_cast<uint8_t>(crc >> 8);
  frame_[n++] = static_cast<uint8_t>(crc & 0xFFu);

  // A short write would let the host splice the next frame into this one's
  // payload. Dropping it whole costs a timeout instead of desynchronising the
  // stream for the rest of the session.
  ++reply_count_;
  if (usb_->Send(frame_, n) != n) {
    ++tx_drop_count_;
  }
}
