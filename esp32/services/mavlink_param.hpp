// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include <mavlink.h>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <variant>

#include "fc_config_cache.hpp"
#include "mavlink_config.hpp"
#include "ring_buffer.hpp"

// The parameter protocol: a fixed table of what this vehicle serves, the RC
// calibration set derived per channel, and the PARAM_VALUE frames owed in
// reply. Values held by the flight computer come and go through
// FcConfigCache; a reply that needs one not yet held waits on it.
class MavlinkParamServer {
 public:
  enum class SetResult : uint8_t {
    kAccepted,
    kUnknownParam,
    kUnsupported,
    kMissingBaseConfig,
    kInvalidValue,
    kInvalidResultingConfig,
  };
  static const char *SetResultName(SetResult result);

  void Init(const MavlinkConfig &cfg, FcConfigCache &fc_config);
  // The stream and the replies queued are dropped: the link they were for is
  // gone.
  void Reset();

  // PARAM_REQUEST_LIST.
  void StartStream();
  // PARAM_REQUEST_READ. False when neither the index nor the id names one.
  bool QueueRead(int16_t param_index, const char *param_id);
  // PARAM_SET. A PARAM_VALUE goes back whatever the result, carrying what the
  // parameter holds now -- which is how a ground station sees a rejection.
  SetResult Set(const char *param_id, float param_value, uint8_t param_type);
  // The next PARAM_VALUE owed, queued replies before the stream. Absent when
  // nothing is owed, or the one owed waits on the flight computer.
  std::optional<mavlink_message_t> NextMessage(uint32_t now_ms);

 private:
  static constexpr std::size_t kParamIdCStringLen =
      MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1u;
  static constexpr uint8_t kReplyQueueDepth = 64;

  struct FixedParamRef {
    uint16_t mavlink_index = 0;
  };

  // Values are the per-channel parameter offsets; mavlink_index arithmetic and
  // the index->field modulo both depend on them.
  enum class RcCalField : uint8_t { kMin = 0, kMax = 1, kTrim = 2, kRev = 3 };

  struct RcCalibrationParamRef {
    uint16_t mavlink_index = 0;
    uint8_t channel_index = 0;
    RcCalField field = RcCalField::kMin;
  };

  using ParamRef = std::variant<FixedParamRef, RcCalibrationParamRef>;

  struct EncodedParam {
    char id[kParamIdCStringLen]{};
    uint8_t type = 0;
    float value = 0.0f;
  };

  struct StreamIdle {};
  struct StreamActive {
    uint16_t next_param_index = 0;
  };

  static uint16_t ParamMavlinkIndex(const ParamRef &param);
  static std::optional<ParamRef> TryResolveRcCalibrationParam(
      const char *param_id);
  std::optional<ParamRef> TryResolveParam(int16_t requested_index,
                                          const char *requested_id) const;
  std::optional<ParamRef> TryResolveParamByIndex(uint16_t param_index) const;

  std::optional<EncodedParam> TryEncodeParam(const ParamRef &param) const;
  std::optional<EncodedParam> TryEncodeFixedParam(
      const FixedParamRef &param) const;
  std::optional<float> TryEncodeGyroCalibrationIdParam() const;
  std::optional<float> TryEncodeMagCalibrationIdParam() const;
  std::optional<float> TryEncodeRcMapParam(const FixedParamRef &param) const;
  std::optional<EncodedParam> TryEncodeRcCalibrationParam(
      const RcCalibrationParamRef &param) const;

  SetResult TrySetParam(const ParamRef &param, float param_value);
  SetResult TrySetFixedParam(const FixedParamRef &param, float param_value);
  SetResult TrySetRcMapParam(const FixedParamRef &param, float param_value);
  SetResult TrySetRcCalibrationParam(const RcCalibrationParamRef &param,
                                     float param_value);

  // The flight-computer record a parameter is served from, if any.
  std::optional<FcConfigCache::Record> RecordFor(const ParamRef &param) const;
  uint32_t ComputeParamHash() const;

  void QueueReply(uint16_t param_index);
  std::optional<mavlink_message_t> NextQueuedMessage(uint32_t now_ms);
  std::optional<mavlink_message_t> NextStreamMessage(uint32_t now_ms);
  std::optional<mavlink_message_t> PackParamValue(const ParamRef &param) const;

  const MavlinkConfig *cfg_ = nullptr;
  FcConfigCache *fc_config_ = nullptr;
  RingBuffer<uint16_t, kReplyQueueDepth + 1> reply_queue_{};
  std::variant<StreamIdle, StreamActive> stream_{};
};
