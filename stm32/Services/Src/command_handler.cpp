#include "command_handler.hpp"

#include <cstring>

#include "ctx.hpp"
#include "message.hpp"
#include "panic.hpp"
#include "rc_receiver.hpp"
#include "system.hpp"

static void OnPing(AppContext &ctx, const message::Packet &pkt) {
  (void)pkt;
  // Send Pong
  message::Packet tx_pkt;
  tx_pkt.header.id = (uint8_t)message::MsgId::kPong;
  tx_pkt.header.len = 0;
  ctx.sys->GetFcLink().Send(tx_pkt);
}

static void OnRcChannels(AppContext &ctx, const message::Packet &pkt) {
  if (!message::IsPayloadLengthValid(message::MsgId::kRcChannels,
                                     pkt.header.len)) {
    return;
  }

  const auto *rc = (const message::RcChannelsMsg *)pkt.payload;
  ctx.sys->GetRcReceiver().ProcessRawState(*rc, ctx.sys->Time().Micros());
}

static void OnReqRcMap(AppContext &ctx, const message::Packet &pkt) {
  if (!message::IsPayloadLengthValid(message::MsgId::kReqRcMap,
                                     pkt.header.len)) {
    return;
  }

  const message::RcMapConfigMsg rc_map =
      ctx.sys->GetRcReceiver().GetRcMapConfig();
  if (!message::IsRcMapConfigValid(rc_map)) {
    Panic(ErrorCode::kRcReceiverInvalidConfig);
  }
  ctx.sys->GetFcLink().SendRcMapConfig(rc_map);
}

static void OnReqRcCalibration(AppContext &ctx, const message::Packet &pkt) {
  if (!message::IsPayloadLengthValid(message::MsgId::kReqRcCalibration,
                                     pkt.header.len)) {
    return;
  }

  const auto &cal = ctx.sys->GetRcReceiver().GetCalibration();
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
    Panic(ErrorCode::kFcLinkInvalidRcCalibrationConfig);
  }
  ctx.sys->GetFcLink().SendRcCalibrationConfig(rc_cal);
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

  const auto *req = (const message::RcMapConfigMsg *)pkt.payload;
  if (message::IsRcMapConfigValid(*req)) {
    (void)ctx.sys->GetRcReceiver().SetRcMapConfig(*req);
  }

  const message::RcMapConfigMsg rc_map =
      ctx.sys->GetRcReceiver().GetRcMapConfig();
  if (!message::IsRcMapConfigValid(rc_map)) {
    Panic(ErrorCode::kRcReceiverInvalidConfig);
  }
  ctx.sys->GetFcLink().SendRcMapConfig(rc_map);
}

static void OnSetRcCalibration(AppContext &ctx, const message::Packet &pkt) {
  if (!message::IsPayloadLengthValid(message::MsgId::kSetRcCalibrationConfig,
                                     pkt.header.len)) {
    return;
  }

  const auto *req = (const message::RcCalibrationConfigMsg *)pkt.payload;
  if (message::IsRcCalibrationConfigValid(*req)) {
    (void)ctx.sys->GetRcReceiver().SetCalibrationConfig(*req);
  }

  const message::RcCalibrationConfigMsg rc_cal =
      GetRcCalibrationConfigMsg(ctx.sys->GetRcReceiver());
  if (!message::IsRcCalibrationConfigValid(rc_cal)) {
    Panic(ErrorCode::kFcLinkInvalidRcCalibrationConfig);
  }
  ctx.sys->GetFcLink().SendRcCalibrationConfig(rc_cal);
}

static void OnReqGyroCalibrationId(AppContext &ctx,
                                   const message::Packet &pkt) {
  if (!message::IsPayloadLengthValid(message::MsgId::kReqGyroCalibrationId,
                                     pkt.header.len)) {
    return;
  }

  const message::GyroCalibrationIdConfigMsg cfg = {
      .cal_gyro0_id = ctx.sys->GetImu42688p().GetDeviceId(),
  };
  if (!message::IsGyroCalibrationIdConfigValid(cfg)) {
    Panic(ErrorCode::kFcLinkInvalidGyroCalibrationIdConfig);
  }
  ctx.sys->GetFcLink().SendGyroCalibrationIdConfig(cfg);
}

static void OnReqReceiverBind(AppContext &ctx, const message::Packet &pkt) {
  if (!message::IsPayloadLengthValid(message::MsgId::kReqReceiverBind,
                                     pkt.header.len)) {
    return;
  }

  ctx.sys->GetCrsfLinkService().RequestReceiverBind();
  ctx.sys->GetFcLink().SendLog("CRSF RX bind requested");
}

static const Epistole::Dispatcher<AppContext>::Entry kHandlers[] = {
    {message::MsgId::kPing, OnPing},
    {message::MsgId::kReqRcMap, OnReqRcMap},
    {message::MsgId::kReqRcCalibration, OnReqRcCalibration},
    {message::MsgId::kSetRcMapConfig, OnSetRcMapConfig},
    {message::MsgId::kSetRcCalibrationConfig, OnSetRcCalibration},
    {message::MsgId::kReqGyroCalibrationId, OnReqGyroCalibrationId},
    {message::MsgId::kReqReceiverBind, OnReqReceiverBind},
    {message::MsgId::kRcChannels, OnRcChannels},
};

static const Epistole::Dispatcher<AppContext> kDispatcher(
    kHandlers, sizeof(kHandlers) / sizeof(kHandlers[0]));

void CommandHandler::Init() {}

bool CommandHandler::Dispatch(AppContext &ctx, const message::Packet &pkt) {
  if (!message::IsPacketValid(pkt.header.id, pkt.payload, pkt.header.len)) {
    return false;
  }

  return kDispatcher.Dispatch(ctx, pkt);
}
