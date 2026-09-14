// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "command_handler.hpp"

#include <cstring>

#include "ctx.hpp"
#include "error_code.hpp"
#include "message.hpp"
#include "panic.hpp"
#include "rc_receiver.hpp"
#include "state_machine.hpp"
#include "states.hpp"  // IWYU pragma: keep
#include "system.hpp"

CommandHandler &CommandHandler::GetInstance() {
  static CommandHandler instance;
  return instance;
}

// Refused here rather than left to surface as a message type that never
// arrives. Panicking sends kPanic, which the ESP32 shows in place of its own
// handshake timeout.
static void OnHandshake(const AppContext &ctx, const message::Packet &pkt) {
  const auto &peer = message::PayloadAs<message::HandshakeMsg>(pkt);
  if (peer.wire_hash != message::kWireContractHash) {
    Panic(ErrorCode::Stm32::kFcLinkWireMismatch);
  }

  const message::HandshakeMsg reply{.wire_hash = message::kWireContractHash};
  message::Packet tx_pkt;
  tx_pkt.header.id = (uint8_t)message::MsgId::kHandshakeReply;
  tx_pkt.header.len = message::PayloadLength<message::HandshakeMsg>();
  std::memcpy(tx_pkt.payload, &reply, sizeof(reply));
  ctx.sys->FcLinkSvc().Send(tx_pkt);
}

static void OnRcChannels(const AppContext &ctx, const message::Packet &pkt) {
  const auto &rc = message::PayloadAs<message::RcChannelsMsg>(pkt);
  ctx.sys->RcRx().ProcessRawState(rc, ctx.sys->Time().Micros());
}

static void OnReqRcMap(const AppContext &ctx, const message::Packet &) {
  const message::RcMapConfigMsg rc_map = ctx.sys->RcRx().GetRcMapConfig();
  if (!message::IsRcMapConfigValid(rc_map)) {
    Panic(ErrorCode::Stm32::kRcReceiverInvalidConfig);
  }
  ctx.sys->FcLinkSvc().SendRcMapConfig(rc_map);
}

static void OnReqRcCalibration(const AppContext &ctx, const message::Packet &) {
  const auto &cal = ctx.sys->RcRx().GetCalibration();
  message::RcCalibrationConfigMsg rc_cal{};
  static_assert(sizeof(rc_cal.min_us) == sizeof(cal.min_us));
  static_assert(sizeof(rc_cal.max_us) == sizeof(cal.max_us));
  static_assert(sizeof(rc_cal.trim_us) == sizeof(cal.trim_us));
  static_assert(sizeof(rc_cal.rev) == sizeof(cal.rev));
  memcpy(rc_cal.min_us, cal.min_us, sizeof(rc_cal.min_us));
  memcpy(rc_cal.max_us, cal.max_us, sizeof(rc_cal.max_us));
  memcpy(rc_cal.trim_us, cal.trim_us, sizeof(rc_cal.trim_us));
  memcpy(rc_cal.rev, cal.rev, sizeof(rc_cal.rev));
  if (!message::IsRcCalibrationConfigValid(rc_cal)) {
    Panic(ErrorCode::Common::kFcLinkInvalidRcCalibrationConfig);
  }
  ctx.sys->FcLinkSvc().SendRcCalibrationConfig(rc_cal);
}

static message::RcCalibrationConfigMsg GetRcCalibrationConfigMsg(
    RcReceiver &receiver) {
  const auto &cal = receiver.GetCalibration();
  message::RcCalibrationConfigMsg rc_cal{};
  static_assert(sizeof(rc_cal.min_us) == sizeof(cal.min_us));
  static_assert(sizeof(rc_cal.max_us) == sizeof(cal.max_us));
  static_assert(sizeof(rc_cal.trim_us) == sizeof(cal.trim_us));
  static_assert(sizeof(rc_cal.rev) == sizeof(cal.rev));
  memcpy(rc_cal.min_us, cal.min_us, sizeof(rc_cal.min_us));
  memcpy(rc_cal.max_us, cal.max_us, sizeof(rc_cal.max_us));
  memcpy(rc_cal.trim_us, cal.trim_us, sizeof(rc_cal.trim_us));
  memcpy(rc_cal.rev, cal.rev, sizeof(rc_cal.rev));
  return rc_cal;
}

static void OnSetRcMapConfig(const AppContext &ctx,
                             const message::Packet &pkt) {
  // Outside Standby the config belongs to the flight or to the host: the
  // write is a blocking EEPROM transfer on SPI1 from the main tick, and it
  // would retune the channel map the aircraft is being flown by. Echoing the
  // unchanged map below is what tells the sender the write did not take.
  const auto &req = message::PayloadAs<message::RcMapConfigMsg>(pkt);
  if (ctx.sm->CurrentState() == ctx.standby_state &&
      message::IsRcMapConfigValid(req)) {
    (void)ctx.sys->RcRx().SetRcMapConfig(req);
  }

  const message::RcMapConfigMsg rc_map = ctx.sys->RcRx().GetRcMapConfig();
  if (!message::IsRcMapConfigValid(rc_map)) {
    Panic(ErrorCode::Stm32::kRcReceiverInvalidConfig);
  }
  ctx.sys->FcLinkSvc().SendRcMapConfig(rc_map);
}

static void OnSetRcCalibration(const AppContext &ctx,
                               const message::Packet &pkt) {
  // Standby only, for the reasons in OnSetRcMapConfig.
  const auto &req = message::PayloadAs<message::RcCalibrationConfigMsg>(pkt);
  if (ctx.sm->CurrentState() == ctx.standby_state &&
      message::IsRcCalibrationConfigValid(req)) {
    (void)ctx.sys->RcRx().SetCalibrationConfig(req);
  }

  const message::RcCalibrationConfigMsg rc_cal =
      GetRcCalibrationConfigMsg(ctx.sys->RcRx());
  if (!message::IsRcCalibrationConfigValid(rc_cal)) {
    Panic(ErrorCode::Common::kFcLinkInvalidRcCalibrationConfig);
  }
  ctx.sys->FcLinkSvc().SendRcCalibrationConfig(rc_cal);
}

static void OnReqReceiverBind(const AppContext &ctx, const message::Packet &) {
  ctx.sys->CrsfLinkSvc().RequestReceiverBind();
  ctx.sys->FcLinkSvc().SendLog("CRSF RX bind requested");
}

static void OnReqReceiverCancelBind(const AppContext &ctx,
                                    const message::Packet &) {
  ctx.sys->CrsfLinkSvc().RequestReceiverCancelBind();
  ctx.sys->FcLinkSvc().SendLog("CRSF RX bind cancelled");
}

// No arm check here: SensorCalService owns that and the busy test both. The
// outcome is not a reply -- the run outlasts this packet. The gyro ends in a
// tone; the accel and the compass, which the operator has to turn the airframe
// through, report every captured pose.
static void OnCalibrate(const AppContext &ctx, const message::Packet &pkt) {
  const uint8_t sensor = message::PayloadAs<message::CalibrateMsg>(pkt).sensor;
  if (!message::IsCalSensorValid(sensor)) {
    return;
  }
  bool started = false;
  const char *hint = "";
  switch (static_cast<message::CalSensor>(sensor)) {
    case message::CalSensor::kGyro:
      started = ctx.sys->SensorCalSvc().StartGyro(ctx.now_us);
      break;
    case message::CalSensor::kAccel:
      started = ctx.sys->SensorCalSvc().StartAccel(ctx.now_us);
      hint = ": hold each side";
      break;
    case message::CalSensor::kMag:
      started = ctx.sys->SensorCalSvc().StartMag(ctx.now_us);
      hint = ": turn on each side";
      break;
    case message::CalSensor::kCount:
      break;
  }
  if (started) {
    ctx.sys->FcLinkSvc().SendLog("%s calibration started%s",
                                 message::kCalSensorNames[sensor], hint);
    return;
  }
  // 🖕 if asked outside Standby, return the finger.
  ctx.sys->FcLinkSvc().SendLog("%s calibration refused: armed or busy",
                               message::kCalSensorNames[sensor]);
  ctx.sys->FcLinkSvc().SendPacket(
      message::MsgId::kTone,
      message::ToneMsg{.tone = static_cast<uint8_t>(message::Tone::kWarning)});
}

static void OnCancelCalibration(const AppContext &ctx,
                                const message::Packet &) {
  ctx.sys->SensorCalSvc().Cancel();
}

// The gyro and the accel are one chip, so they answer with one id.
static void OnReqCalibrationId(const AppContext &ctx,
                               const message::Packet &pkt) {
  const uint8_t sensor =
      message::PayloadAs<message::ReqCalibrationIdMsg>(pkt).sensor;
  if (!message::IsCalSensorValid(sensor)) {
    return;
  }
  message::CalibrationIdConfigMsg cfg = {.sensor = sensor, .id = 0u};
  switch (static_cast<message::CalSensor>(sensor)) {
    case message::CalSensor::kGyro:
    case message::CalSensor::kAccel:
      cfg.id = ctx.sys->Imu().GetDeviceId();
      break;
    case message::CalSensor::kMag:
      cfg.id = ctx.sys->SensorCalSvc().MagCalibrationId();
      break;
    case message::CalSensor::kCount:
      break;
  }
  if (!message::IsCalibrationIdConfigValid(cfg)) {
    Panic(ErrorCode::Common::kFcLinkInvalidCalibrationIdConfig);
  }
  ctx.sys->FcLinkSvc().SendPacket(message::MsgId::kCalibrationIdConfig, cfg);
}

// Privileged only in that FcLink access is the whole gate, as for kReboot and
// kBootload -- it bypasses nothing. Sentinel answers this request against the
// same interlocks the arm switch faces, and owns the refusal, so this end just
// forwards it. Its use is the bench, where there is no transmitter to arm with.
static void OnPrivilegedArm(const AppContext &ctx, const message::Packet &pkt) {
  const auto &req = message::PayloadAs<message::PrivilegedArmMsg>(pkt);
  ctx.sys->SentinelSvc().RequestArm(req.armed != 0u);
}

// Revoke before grant: UsbCdc refuses to swap class descriptors while
// attached, so granting first leaves the new dialect on the old descriptors.
static void OnSetUsbMode(const AppContext &ctx, const message::Packet &pkt) {
  const auto mode = static_cast<message::UsbMode>(
      message::PayloadAs<message::SetUsbModeMsg>(pkt).mode);

  if (mode != message::UsbMode::kEscConfig) {
    ctx.sys->MspSvc().SetEscConfigMode(false);
  }
  if (mode != message::UsbMode::kMsc) {
    ctx.sys->MscSvc().SetMscMode(false);
  }
  if (mode == message::UsbMode::kEscConfig) {
    ctx.sys->MspSvc().SetEscConfigMode(true);
  } else if (mode == message::UsbMode::kMsc) {
    ctx.sys->MscSvc().SetMscMode(true);
  }
}

// Outside Standby the card belongs to the flight (armed) or to the host (MSC).
static void OnLogList(const AppContext &ctx, const message::Packet &pkt) {
  const auto &req = message::PayloadAs<message::LogListMsg>(pkt);
  message::LogListReplyMsg reply{};
  if (ctx.sm->CurrentState() != ctx.standby_state) {
    reply.status = static_cast<uint8_t>(message::LogStatus::kBusy);
  } else {
    ctx.sys->LogSvc().ListLogs(req.first, reply);
  }
  ctx.sys->FcLinkSvc().SendPacket(message::MsgId::kLogListReply, reply);
}

static void OnLogRead(const AppContext &ctx, const message::Packet &pkt) {
  const auto &req = message::PayloadAs<message::LogReadMsg>(pkt);
  message::LogDataMsg reply{};
  if (ctx.sm->CurrentState() != ctx.standby_state) {
    reply.offset = req.offset;
    reply.status = static_cast<uint8_t>(message::LogStatus::kBusy);
  } else {
    ctx.sys->LogSvc().ReadLog(req, reply);
  }
  ctx.sys->FcLinkSvc().SendPacket(message::MsgId::kLogData, reply);
}

static const Dispatcher<const AppContext>::Entry kHandlers[] = {
    {message::MsgId::kHandshake, OnHandshake},
    {message::MsgId::kReqRcMap, OnReqRcMap},
    {message::MsgId::kReqRcCalibration, OnReqRcCalibration},
    {message::MsgId::kSetRcMapConfig, OnSetRcMapConfig},
    {message::MsgId::kSetRcCalibrationConfig, OnSetRcCalibration},
    {message::MsgId::kReqReceiverBind, OnReqReceiverBind},
    {message::MsgId::kReqReceiverCancelBind, OnReqReceiverCancelBind},
    {message::MsgId::kCalibrate, OnCalibrate},
    {message::MsgId::kCancelCalibration, OnCancelCalibration},
    {message::MsgId::kReqCalibrationId, OnReqCalibrationId},
    {message::MsgId::kRcChannels, OnRcChannels},
    {message::MsgId::kPrivilegedArm, OnPrivilegedArm},
    {message::MsgId::kSetUsbMode, OnSetUsbMode},
    {message::MsgId::kLogList, OnLogList},
    {message::MsgId::kLogRead, OnLogRead},
};

static const Dispatcher<const AppContext> kDispatcher(kHandlers);

void CommandHandler::Init() {
  if (initialized_) {
    Panic(ErrorCode::Stm32::kCommandHandlerReinit);
  }
  initialized_ = true;
}

bool CommandHandler::Dispatch(const AppContext &ctx,
                              const message::Packet &pkt) {
  if (!message::IsPacketValid(pkt.header.id, pkt.payload, pkt.header.len)) {
    return false;
  }

  return kDispatcher.Dispatch(ctx, pkt);
}
