// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "command_handler.hpp"

#include <cstring>

#include "dispatcher.hpp"
#include "error_code.hpp"
#include "message.hpp"
#include "panic.hpp"
#include "rc_receiver.hpp"
#include "state_machine_context.hpp"
#include "system.hpp"

CommandHandler &CommandHandler::GetInstance() {
  static CommandHandler instance;
  return instance;
}

// Refused here rather than left to surface as a message type that never
// arrives. Panicking sends kPanic, which the ESP32 shows in place of its own
// handshake timeout.
static void OnHandshake(const StateMachineContext &,
                        const message::Packet &pkt) {
  const auto &peer = message::PayloadAs<message::HandshakeMsg>(pkt);
  if (peer.wire_hash != message::kWireContractHash) {
    Panic(ErrorCode::Stm32::kFcLinkWireMismatch);
  }

  const message::HandshakeMsg reply{.wire_hash = message::kWireContractHash};
  message::Packet tx_pkt;
  tx_pkt.header.id = (uint8_t)message::MsgId::kHandshakeReply;
  tx_pkt.header.len = message::PayloadLength<message::HandshakeMsg>();
  std::memcpy(tx_pkt.payload, &reply, sizeof(reply));
  System::GetInstance().FcLinkSvc().Send(tx_pkt);
}

static void OnRcChannels(const StateMachineContext &,
                         const message::Packet &pkt) {
  const auto &rc = message::PayloadAs<message::RcChannelsMsg>(pkt);
  System::GetInstance().RcRx().ProcessRawState(
      rc, System::GetInstance().Time().Micros());
}

static void OnReqRcMap(const StateMachineContext &, const message::Packet &) {
  const message::RcMapConfigMsg rc_map =
      System::GetInstance().RcRx().GetRcMapConfig();
  if (!message::IsRcMapConfigValid(rc_map)) {
    Panic(ErrorCode::Stm32::kRcReceiverInvalidConfig);
  }
  System::GetInstance().FcLinkSvc().SendPacket(message::MsgId::kRcMapConfig,
                                               rc_map);
}

static void OnSetRcMapConfig(const StateMachineContext &ctx,
                             const message::Packet &pkt) {
  // Outside Standby the config belongs to the flight or to the host: the
  // write is a blocking EEPROM transfer on SPI1 from the main tick, and it
  // would retune the channel map the aircraft is being flown by. Echoing the
  // unchanged map below is what tells the sender the write did not take.
  const auto &req = message::PayloadAs<message::RcMapConfigMsg>(pkt);
  if (ctx.sm->CurrentState() == &ctx.standby_state &&
      message::IsRcMapConfigValid(req)) {
    (void)System::GetInstance().RcRx().SetRcMapConfig(req);
  }

  const message::RcMapConfigMsg rc_map =
      System::GetInstance().RcRx().GetRcMapConfig();
  if (!message::IsRcMapConfigValid(rc_map)) {
    Panic(ErrorCode::Stm32::kRcReceiverInvalidConfig);
  }
  System::GetInstance().FcLinkSvc().SendPacket(message::MsgId::kRcMapConfig,
                                               rc_map);
}

static void OnReqReceiverBind(const StateMachineContext &,
                              const message::Packet &) {
  System::GetInstance().CrsfLinkSvc().RequestReceiverBind();
  System::GetInstance().FcLinkSvc().SendLog("CRSF RX bind requested");
}

static void OnReqReceiverCancelBind(const StateMachineContext &,
                                    const message::Packet &) {
  System::GetInstance().CrsfLinkSvc().RequestReceiverCancelBind();
  System::GetInstance().FcLinkSvc().SendLog("CRSF RX bind cancelled");
}

// No arm check here: SensorCalService owns that and the busy test both. The
// outcome is not a reply -- the run outlasts this packet and reports its own
// edges, every captured pose included for the two the operator turns the
// airframe through.
static void OnCalibrate(const StateMachineContext &ctx,
                        const message::Packet &pkt) {
  const uint8_t sensor = message::PayloadAs<message::CalibrateMsg>(pkt).sensor;
  if (!message::IsCalSensorValid(sensor)) {
    return;
  }
  bool started = false;
  const char *hint = "";
  switch (static_cast<message::CalSensor>(sensor)) {
    case message::CalSensor::kGyro:
      started = System::GetInstance().SensorCalSvc().StartGyro(ctx.now_us);
      break;
    case message::CalSensor::kAccel:
      started = System::GetInstance().SensorCalSvc().StartAccel(ctx.now_us);
      hint = ": hold each side";
      break;
    case message::CalSensor::kMag:
      started = System::GetInstance().SensorCalSvc().StartMag(ctx.now_us);
      hint = ": turn on each side";
      break;
    case message::CalSensor::kLevel:
      started = System::GetInstance().SensorCalSvc().StartLevel(ctx.now_us);
      hint = ": hold level and still";
      break;
    case message::CalSensor::kCount:
      break;
  }
  if (started) {
    System::GetInstance().FcLinkSvc().SendLog("%s calibration started%s",
                                 message::kCalSensorNames[sensor], hint);
    return;
  }
  // 🖕 if asked outside Standby, return the finger.
  System::GetInstance().FcLinkSvc().SendLog(
      "%s calibration refused: armed or busy",
      message::kCalSensorNames[sensor]);
  System::GetInstance().FcLinkSvc().SendPacket(
      message::MsgId::kTone,
      message::ToneMsg{.tone = static_cast<uint8_t>(message::Tone::kWarning)});
}

static void OnCancelCalibration(const StateMachineContext &,
                                const message::Packet &) {
  System::GetInstance().SensorCalSvc().Cancel();
}

// The gyro and the accel are one chip, so they answer with one id.
static void OnReqCalibrationId(const StateMachineContext &,
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
      cfg.id = System::GetInstance().Imu().GetDeviceId();
      break;
    case message::CalSensor::kMag:
      cfg.id = System::GetInstance().SensorCalSvc().MagCalibrationId();
      break;
    case message::CalSensor::kLevel:
    case message::CalSensor::kCount:
      break;
  }
  if (!message::IsCalibrationIdConfigValid(cfg)) {
    Panic(ErrorCode::Common::kFcLinkInvalidCalibrationIdConfig);
  }
  System::GetInstance().FcLinkSvc().SendPacket(
      message::MsgId::kCalibrationIdConfig, cfg);
}

static void OnReqBoardTrim(const StateMachineContext &,
                           const message::Packet &) {
  System::GetInstance().FcLinkSvc().SendPacket(
      message::MsgId::kBoardTrimConfig,
      System::GetInstance().SensorCalSvc().BoardTrim());
}

// Standby only, for the reasons in OnSetRcMapConfig; the echo is what says
// whether the write took.
static void OnSetBoardTrim(const StateMachineContext &ctx,
                           const message::Packet &pkt) {
  const auto &req = message::PayloadAs<message::BoardTrimConfigMsg>(pkt);
  if (ctx.sm->CurrentState() == &ctx.standby_state &&
      message::IsBoardTrimConfigValid(req)) {
    (void)System::GetInstance().SensorCalSvc().SetBoardTrim(req);
  }
  const message::BoardTrimConfigMsg trim =
      System::GetInstance().SensorCalSvc().BoardTrim();
  if (!message::IsBoardTrimConfigValid(trim)) {
    Panic(ErrorCode::Common::kFcLinkInvalidBoardTrimConfig);
  }
  System::GetInstance().FcLinkSvc().SendPacket(message::MsgId::kBoardTrimConfig,
                                               trim);
}

// Privileged only in that FcLink access is the whole gate, as for kReboot and
// kBootload -- it bypasses nothing. Sentinel answers this request against the
// same interlocks the arm switch faces, and owns the refusal, so this end just
// forwards it. The bench has no transmitter to arm with, and a ground station
// sends it too.
static void OnPrivilegedArm(const StateMachineContext &,
                            const message::Packet &pkt) {
  const auto &req = message::PayloadAs<message::PrivilegedArmMsg>(pkt);
  System::GetInstance().SentinelSvc().RequestArm(req.armed != 0u);
}

// Revoke before grant: UsbCdc refuses to swap class descriptors while
// attached, so granting first leaves the new dialect on the old descriptors.
static void OnSetUsbMode(const StateMachineContext &,
                         const message::Packet &pkt) {
  const auto mode = static_cast<message::UsbMode>(
      message::PayloadAs<message::SetUsbModeMsg>(pkt).mode);

  if (mode != message::UsbMode::kEscConfig) {
    System::GetInstance().MspSvc().SetEscConfigMode(false);
  }
  if (mode != message::UsbMode::kMsc) {
    System::GetInstance().MscSvc().SetMscMode(false);
  }
  if (mode == message::UsbMode::kEscConfig) {
    System::GetInstance().MspSvc().SetEscConfigMode(true);
  } else if (mode == message::UsbMode::kMsc) {
    System::GetInstance().MscSvc().SetMscMode(true);
  }
}

// Outside Standby the card belongs to the flight (armed) or to the host (MSC).
static void OnLogList(const StateMachineContext &ctx,
                      const message::Packet &pkt) {
  const auto &req = message::PayloadAs<message::LogListMsg>(pkt);
  message::LogListReplyMsg reply{};
  if (ctx.sm->CurrentState() != &ctx.standby_state) {
    reply.status = static_cast<uint8_t>(message::LogStatus::kBusy);
  } else {
    System::GetInstance().LogSvc().ListLogs(req.first, reply);
  }
  System::GetInstance().FcLinkSvc().SendPacket(message::MsgId::kLogListReply,
                                               reply);
}

static void OnLogRead(const StateMachineContext &ctx,
                      const message::Packet &pkt) {
  const auto &req = message::PayloadAs<message::LogReadMsg>(pkt);
  message::LogDataMsg reply{};
  if (ctx.sm->CurrentState() != &ctx.standby_state) {
    reply.offset = req.offset;
    reply.status = static_cast<uint8_t>(message::LogStatus::kBusy);
  } else {
    System::GetInstance().LogSvc().ReadLog(req, reply);
  }
  System::GetInstance().FcLinkSvc().SendPacket(message::MsgId::kLogData, reply);
}

static const Dispatcher<const StateMachineContext>::Entry kHandlers[] = {
    {message::MsgId::kHandshake, OnHandshake},
    {message::MsgId::kReqRcMap, OnReqRcMap},
    {message::MsgId::kSetRcMapConfig, OnSetRcMapConfig},
    {message::MsgId::kReqReceiverBind, OnReqReceiverBind},
    {message::MsgId::kReqReceiverCancelBind, OnReqReceiverCancelBind},
    {message::MsgId::kCalibrate, OnCalibrate},
    {message::MsgId::kCancelCalibration, OnCancelCalibration},
    {message::MsgId::kReqCalibrationId, OnReqCalibrationId},
    {message::MsgId::kReqBoardTrim, OnReqBoardTrim},
    {message::MsgId::kSetBoardTrimConfig, OnSetBoardTrim},
    {message::MsgId::kRcChannels, OnRcChannels},
    {message::MsgId::kPrivilegedArm, OnPrivilegedArm},
    {message::MsgId::kSetUsbMode, OnSetUsbMode},
    {message::MsgId::kLogList, OnLogList},
    {message::MsgId::kLogRead, OnLogRead},
};

static const Dispatcher<const StateMachineContext> kDispatcher(kHandlers);

void CommandHandler::Init() {
  if (initialized_) {
    Panic(ErrorCode::Stm32::kCommandHandlerReinit);
  }
  initialized_ = true;
}

bool CommandHandler::Dispatch(const StateMachineContext &ctx,
                              const message::Packet &pkt) {
  if (!message::IsPacketValid(pkt.header.id, pkt.payload, pkt.header.len)) {
    return false;
  }

  return kDispatcher.Dispatch(ctx, pkt);
}
