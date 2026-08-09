// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "command_handler.hpp"

#include <cstring>

#include "ctx.hpp"
#include "error_code.hpp"
#include "message.hpp"
#include "panic.hpp"
#include "rc_receiver.hpp"
#include "system.hpp"

CommandHandler &CommandHandler::GetInstance() {
  static CommandHandler instance;
  return instance;
}

static void OnPing(AppContext &ctx, const message::Packet &pkt) {
  (void)pkt;
  message::Packet tx_pkt;
  tx_pkt.header.id = (uint8_t)message::MsgId::kPong;
  tx_pkt.header.len = 0;
  ctx.sys->FcLinkSvc().Send(tx_pkt);
}

static void OnRcChannels(AppContext &ctx, const message::Packet &pkt) {
  if (!message::IsPayloadLengthValid(message::MsgId::kRcChannels,
                                     pkt.header.len)) {
    return;
  }

  const auto &rc = message::PayloadAs<message::RcChannelsMsg>(pkt);
  ctx.sys->RcRx().ProcessRawState(rc, ctx.sys->Time().Micros());
}

static void OnReqRcMap(AppContext &ctx, const message::Packet &pkt) {
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

static void OnReqRcCalibration(AppContext &ctx, const message::Packet &pkt) {
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

static void OnSetRcMapConfig(AppContext &ctx, const message::Packet &pkt) {
  if (!message::IsPayloadLengthValid(message::MsgId::kSetRcMapConfig,
                                     pkt.header.len)) {
    return;
  }

  const auto &req = message::PayloadAs<message::RcMapConfigMsg>(pkt);
  if (message::IsRcMapConfigValid(req)) {
    (void)ctx.sys->RcRx().SetRcMapConfig(req);
  }

  const message::RcMapConfigMsg rc_map = ctx.sys->RcRx().GetRcMapConfig();
  if (!message::IsRcMapConfigValid(rc_map)) {
    Panic(ErrorCode::Stm32::kRcReceiverInvalidConfig);
  }
  ctx.sys->FcLinkSvc().SendRcMapConfig(rc_map);
}

static void OnSetRcCalibration(AppContext &ctx, const message::Packet &pkt) {
  if (!message::IsPayloadLengthValid(message::MsgId::kSetRcCalibrationConfig,
                                     pkt.header.len)) {
    return;
  }

  const auto &req = message::PayloadAs<message::RcCalibrationConfigMsg>(pkt);
  if (message::IsRcCalibrationConfigValid(req)) {
    (void)ctx.sys->RcRx().SetCalibrationConfig(req);
  }

  const message::RcCalibrationConfigMsg rc_cal =
      GetRcCalibrationConfigMsg(ctx.sys->RcRx());
  if (!message::IsRcCalibrationConfigValid(rc_cal)) {
    Panic(ErrorCode::Common::kFcLinkInvalidRcCalibrationConfig);
  }
  ctx.sys->FcLinkSvc().SendRcCalibrationConfig(rc_cal);
}

static void OnReqGyroCalibrationId(AppContext &ctx,
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

static void OnReqReceiverBind(AppContext &ctx, const message::Packet &pkt) {
  if (!message::IsPayloadLengthValid(message::MsgId::kReqReceiverBind,
                                     pkt.header.len)) {
    return;
  }

  ctx.sys->CrsfLinkSvc().RequestReceiverBind();
  ctx.sys->FcLinkSvc().SendLog("CRSF RX bind requested");
}

// Privileged: directly toggle ESC + Mixer arm state, bypassing any future
// arming state machine. Trust model matches kReboot/kBootload (gated only by
// FCLink access). Used by test fixtures to drive non-zero mixer outputs
// without physical RC stick gestures.
static void OnPrivilegedArm(AppContext &ctx, const message::Packet &pkt) {
  if (!message::IsPayloadLengthValid(message::MsgId::kPrivilegedArm,
                                     pkt.header.len)) {
    return;
  }
  const auto &req = message::PayloadAs<message::PrivilegedArmMsg>(pkt);
  const bool armed = req.armed != 0u;
  // Tearing the port down mid-write is how an ESC gets bricked, so an arm
  // during a configuration session is refused rather than allowed to revoke.
  if (armed && ctx.sys->MspSvc().EscConfigGranted()) {
    ctx.sys->FcLinkSvc().SendPacket(
        message::MsgId::kTone, message::ToneMsg{.tone = static_cast<uint8_t>(
                                                    message::Tone::kWarning)});
    return;
  }
  // Reset on every arm transition: a wound-up controller from a prior
  // session must not kick the next arm.
  ctx.sys->RateControllerSvc().Reset();
  ctx.sys->EscSvc().SetArmed(armed);
  ctx.sys->MixerSvc().SetArmed(armed);
}

static void OnSetEscConfigMode(AppContext &ctx, const message::Packet &pkt) {
  if (!message::IsPayloadLengthValid(message::MsgId::kSetEscConfigMode,
                                     pkt.header.len)) {
    return;
  }
  const auto &req = message::PayloadAs<message::SetEscConfigModeMsg>(pkt);
  ctx.sys->MspSvc().SetEscConfigMode(req.enabled != 0u);
}

static const Dispatcher<AppContext>::Entry kHandlers[] = {
    {message::MsgId::kPing, OnPing},
    {message::MsgId::kReqRcMap, OnReqRcMap},
    {message::MsgId::kReqRcCalibration, OnReqRcCalibration},
    {message::MsgId::kSetRcMapConfig, OnSetRcMapConfig},
    {message::MsgId::kSetRcCalibrationConfig, OnSetRcCalibration},
    {message::MsgId::kReqGyroCalibrationId, OnReqGyroCalibrationId},
    {message::MsgId::kReqReceiverBind, OnReqReceiverBind},
    {message::MsgId::kRcChannels, OnRcChannels},
    {message::MsgId::kPrivilegedArm, OnPrivilegedArm},
    {message::MsgId::kSetEscConfigMode, OnSetEscConfigMode},
};

static const Dispatcher<AppContext> kDispatcher(kHandlers);

void CommandHandler::Init() {}

bool CommandHandler::Dispatch(AppContext &ctx, const message::Packet &pkt) {
  if (!message::IsPacketValid(pkt.header.id, pkt.payload, pkt.header.len)) {
    return false;
  }

  return kDispatcher.Dispatch(ctx, pkt);
}
