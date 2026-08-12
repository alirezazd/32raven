// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "msp_service.hpp"

#include <cmath>
#include <cstring>
#include <span>

#include "checksum.hpp"
#include "dshot_codec.hpp"
#include "error_code.hpp"
#include "panic.hpp"
#include "stm32f4xx.h"

namespace {

// The identification block (1-5) is mandatory; the rest populate the status
// view.
constexpr uint16_t kMspApiVersion = 1;
constexpr uint16_t kMspFcVariant = 2;
constexpr uint16_t kMspFcVersion = 3;
constexpr uint16_t kMspBoardInfo = 4;
constexpr uint16_t kMspBuildInfo = 5;
constexpr uint16_t kMspName = 10;
constexpr uint16_t kMspStatus = 101;
constexpr uint16_t kMspMotor = 104;
constexpr uint16_t kMspAttitude = 108;
constexpr uint16_t kMspAnalog = 110;
constexpr uint16_t kMspBatteryState = 130;
constexpr uint16_t kMspMotorConfig = 131;
constexpr uint16_t kMspUid = 160;
constexpr uint16_t kMspSetMotor = 214;
constexpr uint16_t kMspSetPassthrough = 245;

// Only four-way is offered; anything else is answered with a count of zero.
constexpr uint8_t kPassthroughEsc4Way = 0xFF;

// A four-way page write is 262 bytes, so this clears one frame per tick.
constexpr uint16_t kBytesPerPoll = 320u;

// Describes the DShot command space, not a stored setting; never read back.
constexpr uint16_t kMinThrottle = 1070;
constexpr uint16_t kMaxThrottle = 2000;
constexpr uint16_t kMinCommand = 1000;

// Configurators gate features on this, so it must match BuildReply's layouts.
constexpr uint8_t kMspProtocolVersion = 0;
constexpr uint8_t kApiVersionMajor = 1;
constexpr uint8_t kApiVersionMinor = 46;

// A dialect identifier, not a claim to be Betaflight: it is the only variant
// string the ESC tooling recognises.
constexpr char kFcVariant[4] = {'B', 'T', 'F', 'L'};
constexpr uint8_t kFcVersionMajor = 4;
constexpr uint8_t kFcVersionMinor = 5;
constexpr uint8_t kFcVersionPatch = 0;

constexpr uint8_t kMcuTypeF40x = 2;
constexpr uint8_t kSensorAcc = 1u << 0;
constexpr uint8_t kSensorGps = 1u << 3;
constexpr uint8_t kSensorGyro = 1u << 5;

constexpr size_t kMotorReportCount = 8;

constexpr uint8_t kFrameStart = '$';
constexpr uint8_t kFrameV1 = 'M';
constexpr uint8_t kFrameV2 = 'X';
constexpr uint8_t kDirectionToDevice = '<';
constexpr uint8_t kDirectionToHost = '>';
constexpr uint8_t kDirectionError = '!';

// Out-of-range float conversion is undefined and none of these inputs are
// bounded, so saturate. NaN maps to zero: `!(v > x)` would carry it into the
// cast.
uint16_t SaturateU16(float value) {
  if (!(value > 0.0f)) {
    return 0u;
  }
  return (value >= 65535.0f) ? 65535u : static_cast<uint16_t>(value);
}

int16_t SaturateI16(float value) {
  if (!(value > -32768.0f)) {
    return (value != value) ? static_cast<int16_t>(0)
                            : static_cast<int16_t>(-32768);
  }
  return (value >= 32767.0f) ? static_cast<int16_t>(32767)
                             : static_cast<int16_t>(value);
}

uint8_t SaturateU8(float value) {
  if (!(value > 0.0f)) {
    return 0u;
  }
  return (value >= 255.0f) ? 255u : static_cast<uint8_t>(value);
}

}  // namespace

void MspService::Init(const Config &cfg, UsbCdc &usb,
                      VehicleState &vehicle_state, FourWayService &four_way,
                      EscService &esc) {
  if (initialized_) {
    Panic(ErrorCode::Stm32::kMspServiceReinit);
  }
  initialized_ = true;
  cfg_ = cfg;
  usb_ = &usb;
  vehicle_state_ = &vehicle_state;
  four_way_ = &four_way;
  esc_ = &esc;
}

void MspService::RevokeEscConfigMode() {
  esc_config_granted_ = false;
  four_way_->Exit();
  usb_->SetAttached(false);
}

void MspService::SetEscConfigMode(bool enabled) {
  // The mixer is still writing motor commands, and the device commands this
  // unlocks drive the same lines.
  if (!enabled || esc_->IsArmed()) {
    RevokeEscConfigMode();
    return;
  }
  esc_config_granted_ = true;
  usb_->SetAttached(true);
}

// The arm state is re-read every tick rather than only when the mode is set,
// because the mode is latched: nothing else would notice a vehicle that armed
// after the port was opened.
void MspService::CheckArmed() {
  if (!initialized_ || !esc_config_granted_) {
    return;
  }
  if (esc_->IsArmed()) {
    RevokeEscConfigMode();
  }
}

void MspService::Poll(uint32_t now_us) {
  if (!initialized_) {
    return;
  }

  // A bus reset ends the session even if the host never sent cmd_InterfaceExit.
  // Latched four-way would route the next session's MSP probes into a parser
  // that only recognises 0x2F.
  const uint32_t resets = usb_->ResetCount();
  if (resets != last_usb_reset_) {
    last_usb_reset_ = resets;
    four_way_->Exit();
    Reset();
  }

  // Closing the port drops DTR without resetting the bus, so the check above
  // never fires for a host that simply goes away mid-session. Without this the
  // interface stays latched and the next session's MSP probes are eaten by a
  // parser that only recognises 0x2F.
  const bool connected = usb_->IsConnected();
  if (last_usb_connected_ && !connected) {
    four_way_->Exit();
    Reset();
  }
  last_usb_connected_ = connected;

  // Detached means there is no session to serve. Dropping any half-parsed
  // frame here stops a truncated request from splicing onto the first bytes of
  // the next session.
  if (!usb_->IsAttached()) {
    Reset();
    return;
  }

  // Bounded like the GPS drain in the same slow tick: an unbounded loop can
  // consume the whole 1 KB ring plus a reply per frame, blowing the tick budget
  // and starving the fast cascade behind it. Leftovers wait for the next tick.
  uint8_t byte = 0;
  uint16_t budget = kBytesPerPoll;
  while (budget-- > 0u && usb_->Read(byte)) {
    if (four_way_->IsActive()) {
      four_way_->Feed(byte);
    } else {
      Feed(byte);
    }
  }

  // After the drain, so a frame that completed this tick is already back at
  // its escape state and never looks stalled.
  four_way_->Poll(now_us);
}

// Framing

void MspService::Feed(uint8_t byte) {
  switch (parse_) {
    case Parse::kIdle:
      if (byte == kFrameStart) {
        parse_ = Parse::kVersion;
      }
      break;

    case Parse::kVersion:
      if (byte == kFrameV1) {
        is_v2_ = false;
        parse_ = Parse::kDirection;
      } else if (byte == kFrameV2) {
        is_v2_ = true;
        parse_ = Parse::kDirection;
      } else {
        Reset();
      }
      break;

    case Parse::kDirection:
      // Only host-to-device frames are ours to answer; seeing our own replies
      // echoed back is normal on a shared line and must not desynchronise us.
      if (byte != kDirectionToDevice) {
        Reset();
      } else {
        parse_ = is_v2_ ? Parse::kV2Flag : Parse::kV1Size;
        checksum_ = 0;
      }
      break;

    // v1
    case Parse::kV1Size:
      payload_size_ = byte;
      payload_index_ = 0;
      checksum_ = byte;
      if (payload_size_ > kMaxPayload) {
        Reset();
      } else {
        parse_ = Parse::kV1Command;
      }
      break;

    case Parse::kV1Command:
      command_ = byte;
      checksum_ ^= byte;
      parse_ = (payload_size_ == 0u) ? Parse::kV1Checksum : Parse::kV1Payload;
      break;

    case Parse::kV1Payload:
      payload_[payload_index_++] = byte;
      checksum_ ^= byte;
      if (payload_index_ >= payload_size_) {
        parse_ = Parse::kV1Checksum;
      }
      break;

    case Parse::kV1Checksum:
      if (checksum_ == byte) {
        Dispatch();
      } else {
        ++crc_error_count_;
      }
      Reset();
      break;

    // v2
    case Parse::kV2Flag:
      v2_flag_ = byte;
      checksum_ = checksum::Dvbs2Update(0, byte);
      parse_ = Parse::kV2CommandLo;
      break;

    case Parse::kV2CommandLo:
      command_ = byte;
      checksum_ = checksum::Dvbs2Update(checksum_, byte);
      parse_ = Parse::kV2CommandHi;
      break;

    case Parse::kV2CommandHi:
      command_ |= static_cast<uint16_t>(byte) << 8;
      checksum_ = checksum::Dvbs2Update(checksum_, byte);
      parse_ = Parse::kV2SizeLo;
      break;

    case Parse::kV2SizeLo:
      payload_size_ = byte;
      checksum_ = checksum::Dvbs2Update(checksum_, byte);
      parse_ = Parse::kV2SizeHi;
      break;

    case Parse::kV2SizeHi:
      payload_size_ |= static_cast<uint16_t>(byte) << 8;
      checksum_ = checksum::Dvbs2Update(checksum_, byte);
      payload_index_ = 0;
      if (payload_size_ > kMaxPayload) {
        Reset();
      } else {
        parse_ = (payload_size_ == 0u) ? Parse::kV2Checksum : Parse::kV2Payload;
      }
      break;

    case Parse::kV2Payload:
      payload_[payload_index_++] = byte;
      checksum_ = checksum::Dvbs2Update(checksum_, byte);
      if (payload_index_ >= payload_size_) {
        parse_ = Parse::kV2Checksum;
      }
      break;

    case Parse::kV2Checksum:
      if (checksum_ == byte) {
        Dispatch();
      } else {
        ++crc_error_count_;
      }
      Reset();
      break;
  }
}

void MspService::Dispatch() {
  ++request_count_;
  last_command_ = command_;
  reply_len_ = 0;
  reply_overflow_ = false;
  const bool ok = BuildReply(command_) && !reply_overflow_;
  if (!ok) {
    ++unknown_command_count_;
    reply_len_ = 0;
  }
  const bool sent = SendReply(ok);

  // Handing the port over after a lost reply strands the host: it never learns
  // passthrough began, retries in MSP framing, and the four-way parser eats
  // those bytes with no way back.
  if (passthrough_pending_) {
    passthrough_pending_ = false;
    if (sent) {
      four_way_->Enter();
    }
  }
}

bool MspService::SendReply(bool ok) {
  const uint8_t direction = ok ? kDirectionToHost : kDirectionError;

  // Dispatch staged the payload at kV2HeaderBytes, so both dialects write only
  // the bytes around it. v1's header is shorter, so its frame begins partway
  // into the buffer rather than at zero.
  size_t start = 0;
  size_t n = 0;

  if (is_v2_) {
    frame_[0] = kFrameStart;
    frame_[1] = kFrameV2;
    frame_[2] = direction;
    frame_[3] = v2_flag_;
    frame_[4] = static_cast<uint8_t>(command_ & 0xFFu);
    frame_[5] = static_cast<uint8_t>(command_ >> 8);
    frame_[6] = static_cast<uint8_t>(reply_len_ & 0xFFu);
    frame_[7] = static_cast<uint8_t>(reply_len_ >> 8);
    n = kV2HeaderBytes + reply_len_;
    // Covers flags through payload: everything after the direction byte.
    frame_[n] = checksum::Dvbs2(std::span{frame_}.subspan(3u, n - 3u));
    ++n;
  } else {
    // v1 carries an 8-bit length, so an oversized reply cannot be framed at
    // all; answering with the error direction beats emitting a truncated
    // frame the host would parse as valid.
    const bool oversize = reply_len_ > 255u;
    if (oversize) {
      reply_len_ = 0;
    }
    start = kV2HeaderBytes - kV1HeaderBytes;
    const uint8_t size = static_cast<uint8_t>(reply_len_);
    const uint8_t cmd = static_cast<uint8_t>(command_ & 0xFFu);
    frame_[start] = kFrameStart;
    frame_[start + 1u] = kFrameV1;
    frame_[start + 2u] = oversize ? kDirectionError : direction;
    frame_[start + 3u] = size;
    frame_[start + 4u] = cmd;
    n = kV2HeaderBytes + reply_len_;
    uint8_t crc = size ^ cmd;
    for (uint16_t i = 0; i < reply_len_; ++i) {
      crc ^= frame_[kV2HeaderBytes + i];
    }
    frame_[n++] = crc;
  }

  // As in FourWayService::Respond: a truncated frame desynchronises the host's
  // parser permanently, so a short write drops the whole reply.
  ++reply_count_;
  const size_t len = n - start;
  if (usb_->Send(&frame_[start], len) != len) {
    ++tx_drop_count_;
    return false;
  }
  return true;
}

// Payload builders

void MspService::Push8(uint8_t v) {
  if (reply_len_ >= kMaxPayload) {
    // Dropping bytes silently would emit a frame whose declared length does not
    // match its contents, and every field after the gap is then read at the
    // wrong offset. Record it so Dispatch answers with an error instead.
    reply_overflow_ = true;
    return;
  }
  ReplyBuf()[reply_len_++] = v;
}

void MspService::Push16(uint16_t v) {
  Push8(static_cast<uint8_t>(v & 0xFFu));
  Push8(static_cast<uint8_t>(v >> 8));
}

void MspService::Push32(uint32_t v) {
  Push16(static_cast<uint16_t>(v & 0xFFFFu));
  Push16(static_cast<uint16_t>(v >> 16));
}

void MspService::PushString(const char *s) {
  if (s == nullptr) {
    return;
  }
  for (const char *p = s; *p != '\0'; ++p) {
    Push8(static_cast<uint8_t>(*p));
  }
}

void MspService::PushLengthPrefixedString(const char *s) {
  size_t len = (s == nullptr) ? 0u : std::strlen(s);
  // The prefix is a single byte and the fixed-layout tail of MSP_BOARD_INFO is
  // read at an offset derived from it, so a narrowed count would shift every
  // field after this string.
  if (len > 255u) {
    len = 255u;
  }
  Push8(static_cast<uint8_t>(len));
  for (size_t i = 0; i < len; ++i) {
    Push8(static_cast<uint8_t>(s[i]));
  }
}

// Command handlers

bool MspService::BuildReply(uint16_t command) {
  const BatteryData &bat = vehicle_state_->GetBattery();

  switch (command) {
    case kMspApiVersion:
      Push8(kMspProtocolVersion);
      Push8(kApiVersionMajor);
      Push8(kApiVersionMinor);
      return true;

    case kMspFcVariant:
      for (char c : kFcVariant) {
        Push8(static_cast<uint8_t>(c));
      }
      return true;

    case kMspFcVersion:
      Push8(kFcVersionMajor);
      Push8(kFcVersionMinor);
      Push8(kFcVersionPatch);
      return true;

    case kMspBoardInfo: {
      // The field is a fixed four bytes and shorter names are space-padded,
      // so the terminator has to latch — testing id[i] per byte would index
      // past the literal once the string is shorter than the field.
      const char *id = cfg_.board_identifier;
      bool ended = (id == nullptr);
      for (size_t i = 0; i < 4u; ++i) {
        if (!ended && id[i] == '\0') {
          ended = true;
        }
        Push8(static_cast<uint8_t>(ended ? ' ' : id[i]));
      }
      Push16(0);  // hardware revision
      Push8(0);   // no OSD
      Push8(0);   // target capabilities
      PushLengthPrefixedString(cfg_.board_name);
      PushLengthPrefixedString(cfg_.board_name);
      PushLengthPrefixedString(cfg_.manufacturer_id);
      for (size_t i = 0; i < 32u; ++i) {
        Push8(0);  // signature
      }
      Push8(kMcuTypeF40x);
      Push8(0);      // configuration state
      Push16(cfg_.loop_rate_hz);
      Push32(0);     // configuration problems
      return true;
    }

    case kMspBuildInfo:
      PushString("Jan 01 2026");
      PushString("00:00:00");
      PushString("000000");
      Push8(0);
      return true;

    case kMspName:
      PushString(cfg_.craft_name);
      return true;

    case kMspUid: {
      const volatile uint32_t *uid =
          reinterpret_cast<const volatile uint32_t *>(UID_BASE);
      Push32(uid[0]);
      Push32(uid[1]);
      Push32(uid[2]);
      return true;
    }

    case kMspStatus: {
      uint16_t sensors = kSensorAcc | kSensorGyro;
      if (vehicle_state_->GetGps().fix_type >= 2u) {
        sensors |= kSensorGps;
      }
      Push16(cfg_.loop_period_us);
      Push16(0);     // i2c errors
      Push16(sensors);
      Push32(0);  // flight mode flags
      Push8(0);   // config profile
      return true;
    }

    case kMspAttitude: {
      const Eigen::Quaternionf &q =
          vehicle_state_->GetImu().attitude_world_to_body;
      const float sinr = 2.0f * (q.w() * q.x() + q.y() * q.z());
      const float cosr = 1.0f - 2.0f * (q.x() * q.x() + q.y() * q.y());
      const float roll = std::atan2(sinr, cosr);
      float sinp = 2.0f * (q.w() * q.y() - q.z() * q.x());
      sinp = (sinp > 1.0f) ? 1.0f : ((sinp < -1.0f) ? -1.0f : sinp);
      const float pitch = std::asin(sinp);
      const float siny = 2.0f * (q.w() * q.z() + q.x() * q.y());
      const float cosy = 1.0f - 2.0f * (q.y() * q.y() + q.z() * q.z());
      const float yaw = std::atan2(siny, cosy);

      constexpr float rad_to_decidegrees = 572.9578f;
      constexpr float rad_to_degrees = 57.29578f;
      Push16(static_cast<uint16_t>(
          static_cast<int16_t>(roll * rad_to_decidegrees)));
      Push16(static_cast<uint16_t>(
          static_cast<int16_t>(pitch * rad_to_decidegrees)));
      Push16(static_cast<uint16_t>(static_cast<int16_t>(yaw * rad_to_degrees)));
      return true;
    }

    // Configurators cross-check MSP_ANALOG against MSP_BATTERY_STATE, so both
    // report the same pack from one set of conversions.
    case kMspAnalog:
    case kMspBatteryState: {
      const uint8_t decivolts = SaturateU8(bat.voltage * 10.0f);
      const uint16_t centivolts = SaturateU16(bat.voltage * 100.0f);
      const uint16_t centiamps =
          static_cast<uint16_t>(SaturateI16(bat.current * 100.0f));
      const uint16_t mah = SaturateU16(bat.mah_drawn);

      if (command == kMspAnalog) {
        Push8(decivolts);
        Push16(mah);
        Push16(0);  // RSSI
        Push16(centiamps);
        Push16(centivolts);
      } else {
        Push8(0);   // cell count, unknown without a per-cell divider
        Push16(0);  // pack capacity, mAh
        Push8(decivolts);
        Push16(mah);
        Push16(centiamps);
        Push8(0);  // battery state: OK
        Push16(centivolts);
      }
      return true;
    }

    // Spins props on command, so it is refused unless the ESC config lease is
    // held -- that is granted only to a disarmed vehicle and revoked the moment
    // it arms. Passthrough excludes it too: the pins belong to the bit-banged
    // link by then, not the timer.
    case kMspSetMotor: {
      if (!esc_config_granted_ || four_way_->IsActive()) {
        return false;
      }

      std::array<float, 4> thrust{};
      for (size_t i = 0; i < thrust.size(); ++i) {
        const size_t lo = i * 2u;
        if (lo + 1u >= payload_size_) {
          break;
        }
        const uint16_t pulse_us = static_cast<uint16_t>(
            payload_[lo] | (static_cast<uint16_t>(payload_[lo + 1u]) << 8));
        if (pulse_us > kMinCommand) {
          const float span = static_cast<float>(kMaxThrottle - kMinCommand);
          const float value = static_cast<float>(pulse_us - kMinCommand) / span;
          thrust[i] = (value > 1.0f) ? 1.0f : value;
        }
      }
      esc_->SetTestThrottle(thrust);
      return true;
    }

    case kMspSetPassthrough: {
      const uint8_t mode =
          (payload_size_ >= 1u) ? payload_[0] : kPassthroughEsc4Way;
      // A count of zero is the protocol's way of saying "no ESCs are
      // reachable", which is exactly true when the mode is not granted.
      if (mode != kPassthroughEsc4Way || !esc_config_granted_) {
        Push8(0);
        return true;
      }
      Push8(DShotCodec::kMotorCount);
      passthrough_pending_ = true;
      return true;
    }

    case kMspMotorConfig:
      Push16(kMinThrottle);
      Push16(kMaxThrottle);
      Push16(kMinCommand);
      Push8(DShotCodec::kMotorCount);
      Push8(esc_->MotorPoles());
      Push8(0);  // bidirectional DShot: telemetry arrives on its own UART
      Push8(1);  // ESC sensor present
      return true;

    // The inverse of MSP_SET_MOTOR, so the configurator's slider reads back
    // what it set -- and reads back stopped once the test deadman expires.
    case kMspMotor: {
      const DShotCodec::MotorValues &outputs = esc_->Outputs();
      for (size_t i = 0; i < kMotorReportCount; ++i) {
        if (i >= outputs.size()) {
          Push16(0);
          continue;
        }
        const float thrust = EscService::DshotToThrust(outputs[i]);
        const float span = static_cast<float>(kMaxThrottle - kMinCommand);
        Push16(static_cast<uint16_t>(static_cast<float>(kMinCommand) +
                                     thrust * span));
      }
      return true;
    }

    default:
      return false;
  }
}
