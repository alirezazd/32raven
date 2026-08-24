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

static void OnPing(const AppContext &ctx, const message::Packet &pkt) {
  (void)pkt;
  message::Packet tx_pkt;
  tx_pkt.header.id = (uint8_t)message::MsgId::kPong;
  tx_pkt.header.len = 0;
  ctx.sys->FcLinkSvc().Send(tx_pkt);
}

static void OnRcChannels(const AppContext &ctx, const message::Packet &pkt) {
  if (!message::IsPayloadLengthValid(message::MsgId::kRcChannels,
                                     pkt.header.len)) {
    return;
  }

  const auto &rc = message::PayloadAs<message::RcChannelsMsg>(pkt);
  ctx.sys->RcRx().ProcessRawState(rc, ctx.sys->Time().Micros());
}

static void OnReqRcMap(const AppContext &ctx, const message::Packet &pkt) {
  if (!message::IsPayloadLengthValid(message::MsgId::kReqRcMap,
                                     pkt.header.len)) {
    return;
  }

  const message::RcMapConfigMsg rc_map = ctx.sys->RcRx().GetRcMapConfig();
  if (!message::IsRcMapConfigValid(rc_map)) {
    Panic(ErrorCode::Stm32::kRcReceiverInvalidConfig);
  }
  ctx.sys->FcLinkSvc().SendRcMapConfig(rc_map);
}

static void OnReqRcCalibration(const AppContext &ctx,
                               const message::Packet &pkt) {
  if (!message::IsPayloadLengthValid(message::MsgId::kReqRcCalibration,
                                     pkt.header.len)) {
    return;
  }

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
  if (!message::IsPayloadLengthValid(message::MsgId::kSetRcMapConfig,
                                     pkt.header.len)) {
    return;
  }

  // Outside Idle the config belongs to the flight or to the host: the write is
  // a blocking EEPROM transfer on SPI1 from the main tick, and it would retune
  // the channel map the aircraft is being flown by. Echoing the unchanged map
  // below is what tells the sender the write did not take.
  const auto &req = message::PayloadAs<message::RcMapConfigMsg>(pkt);
  if (ctx.sm->CurrentState() == ctx.idle_state &&
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
  if (!message::IsPayloadLengthValid(message::MsgId::kSetRcCalibrationConfig,
                                     pkt.header.len)) {
    return;
  }

  // Idle only, for the reasons in OnSetRcMapConfig.
  const auto &req = message::PayloadAs<message::RcCalibrationConfigMsg>(pkt);
  if (ctx.sm->CurrentState() == ctx.idle_state &&
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

static void OnReqGyroCalibrationId(const AppContext &ctx,
                                   const message::Packet &pkt) {
  if (!message::IsPayloadLengthValid(message::MsgId::kReqGyroCalibrationId,
                                     pkt.header.len)) {
    return;
  }

  const message::GyroCalibrationIdConfigMsg cfg = {
      .cal_gyro0_id = ctx.sys->Imu().GetDeviceId(),
  };
  if (!message::IsGyroCalibrationIdConfigValid(cfg)) {
    Panic(ErrorCode::Common::kFcLinkInvalidGyroCalibrationIdConfig);
  }
  ctx.sys->FcLinkSvc().SendGyroCalibrationIdConfig(cfg);
}

static void OnReqReceiverBind(const AppContext &ctx,
                              const message::Packet &pkt) {
  if (!message::IsPayloadLengthValid(message::MsgId::kReqReceiverBind,
                                     pkt.header.len)) {
    return;
  }

  ctx.sys->CrsfLinkSvc().RequestReceiverBind();
  ctx.sys->FcLinkSvc().SendLog("CRSF RX bind requested");
}

static void OnReqReceiverCancelBind(const AppContext &ctx,
                                    const message::Packet &pkt) {
  if (!message::IsPayloadLengthValid(message::MsgId::kReqReceiverCancelBind,
                                     pkt.header.len)) {
    return;
  }

  ctx.sys->CrsfLinkSvc().RequestReceiverCancelBind();
  ctx.sys->FcLinkSvc().SendLog("CRSF RX bind cancelled");
}

// No arm check here: SensorCalService owns that and the busy test both. The
// outcome is a tone, not a reply -- the run outlasts this packet.
static void OnCalibrateGyro(const AppContext &ctx, const message::Packet &pkt) {
  if (!message::IsPayloadLengthValid(message::MsgId::kCalibrateGyro,
                                     pkt.header.len)) {
    return;
  }

  if (ctx.sys->SensorCalSvc().Gyro().Start(ctx.now_us)) {
    ctx.sys->FcLinkSvc().SendLog("Gyro calibration started");
    return;
  }
  // 🖕 if asked while not idle, return the finger.
  ctx.sys->FcLinkSvc().SendLog("Gyro calibration refused: armed or busy");
  ctx.sys->FcLinkSvc().SendPacket(
      message::MsgId::kTone,
      message::ToneMsg{.tone = static_cast<uint8_t>(message::Tone::kWarning)});
}

// Privileged: directly toggle ESC + Mixer arm state, bypassing any future
// arming state machine. Trust model matches kReboot/kBootload (gated only by
// FCLink access). Used by test fixtures to drive non-zero mixer outputs
// without physical RC stick gestures.
static void OnPrivilegedArm(const AppContext &ctx, const message::Packet &pkt) {
  if (!message::IsPayloadLengthValid(message::MsgId::kPrivilegedArm,
                                     pkt.header.len)) {
    return;
  }
  const auto &req = message::PayloadAs<message::PrivilegedArmMsg>(pkt);
  // Sentinel owns the interlocks and the transition; this end only turns a
  // refusal back into something the operator can hear.
  if (!ctx.sys->SentinelSvc().RequestArm(ctx, req.armed != 0u)) {
    ctx.sys->FcLinkSvc().SendPacket(
        message::MsgId::kTone, message::ToneMsg{.tone = static_cast<uint8_t>(
                                                    message::Tone::kWarning)});
  }
}

// Revoke before grant: UsbCdc refuses to swap class descriptors while
// attached, so granting first leaves the new dialect on the old descriptors.
static void OnSetUsbMode(const AppContext &ctx, const message::Packet &pkt) {
  if (!message::IsPayloadLengthValid(message::MsgId::kSetUsbMode,
                                     pkt.header.len)) {
    return;
  }
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

// Outside Idle the card belongs to the flight (armed) or to the host (MSC).
static void OnLogList(const AppContext &ctx, const message::Packet &pkt) {
  if (!message::IsPayloadLengthValid(message::MsgId::kLogList,
                                     pkt.header.len)) {
    return;
  }
  const auto &req = message::PayloadAs<message::LogListMsg>(pkt);
  message::LogListReplyMsg reply{};
  if (ctx.sm->CurrentState() != ctx.idle_state) {
    reply.status = static_cast<uint8_t>(message::LogStatus::kBusy);
  } else {
    ctx.sys->LogSvc().ListLogs(req.first, reply);
  }
  ctx.sys->FcLinkSvc().SendPacket(message::MsgId::kLogListReply, reply);
}

static void OnLogRead(const AppContext &ctx, const message::Packet &pkt) {
  if (!message::IsPayloadLengthValid(message::MsgId::kLogRead,
                                     pkt.header.len)) {
    return;
  }
  const auto &req = message::PayloadAs<message::LogReadMsg>(pkt);
  message::LogDataMsg reply{};
  if (ctx.sm->CurrentState() != ctx.idle_state) {
    reply.offset = req.offset;
    reply.status = static_cast<uint8_t>(message::LogStatus::kBusy);
  } else {
    ctx.sys->LogSvc().ReadLog(req, reply);
  }
  ctx.sys->FcLinkSvc().SendPacket(message::MsgId::kLogData, reply);
}

static const Dispatcher<const AppContext>::Entry kHandlers[] = {
    {message::MsgId::kPing, OnPing},
    {message::MsgId::kReqRcMap, OnReqRcMap},
    {message::MsgId::kReqRcCalibration, OnReqRcCalibration},
    {message::MsgId::kSetRcMapConfig, OnSetRcMapConfig},
    {message::MsgId::kSetRcCalibrationConfig, OnSetRcCalibration},
    {message::MsgId::kReqGyroCalibrationId, OnReqGyroCalibrationId},
    {message::MsgId::kReqReceiverBind, OnReqReceiverBind},
    {message::MsgId::kReqReceiverCancelBind, OnReqReceiverCancelBind},
    {message::MsgId::kCalibrateGyro, OnCalibrateGyro},
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
