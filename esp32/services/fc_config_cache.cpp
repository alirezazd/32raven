// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "fc_config_cache.hpp"

#include <cstring>

#include "esp_log.h"
#include "fc_link.hpp"
#include "panic.hpp"
#include "timebase.hpp"

namespace {

constexpr const char *kTag = "fc_config";
constexpr uint16_t kWriteAttempts = 100;
constexpr uint16_t kRetryPeriodMs = 50;

}  // namespace

template <typename T>
void FcConfigCache::Slot<T>::Settle(const T &cfg) {
  value = cfg;
  request = {};
  if (write.has_value() && std::memcmp(&write->desired, &cfg, sizeof(T)) == 0) {
    write.reset();
  }
}

template <typename T>
std::optional<T> FcConfigCache::Slot<T>::Retry(uint32_t now_ms,
                                               ErrorCode::Esp32 failed) {
  if (!write.has_value() || (write->next_retry_ms.has_value() &&
                             !TimeReached(now_ms, *write->next_retry_ms))) {
    return std::nullopt;
  }
  if (write->attempts_remaining == 0) {
    Panic(failed);
  }
  write->attempts_remaining--;
  write->next_retry_ms = TimeAfter(now_ms, kRetryPeriodMs);
  return write->desired;
}

void FcConfigCache::Init(FcLink &fc_link) { fc_link_ = &fc_link; }

void FcConfigCache::AbandonWrites() {
  rc_map_.write.reset();
  rc_calibration_.write.reset();
}

void FcConfigCache::Adopt(const message::RcMapConfigMsg &cfg) {
  if (!message::IsRcMapConfigValid(cfg)) {
    Panic(ErrorCode::Esp32::kFcLinkInvalidRcMapConfig);
  }
  rc_map_.Settle(cfg);
}

void FcConfigCache::Adopt(const message::RcCalibrationConfigMsg &cfg) {
  if (!message::IsRcCalibrationConfigValid(cfg)) {
    Panic(ErrorCode::Common::kFcLinkInvalidRcCalibrationConfig);
  }
  rc_calibration_.Settle(cfg);
}

void FcConfigCache::Adopt(const message::CalibrationIdConfigMsg &cfg) {
  if (!message::IsCalibrationIdConfigValid(cfg)) {
    Panic(ErrorCode::Common::kFcLinkInvalidCalibrationIdConfig);
  }
  switch (static_cast<message::CalSensor>(cfg.sensor)) {
    case message::CalSensor::kGyro:
      gyro_calibration_id_.Settle(cfg);
      break;
    case message::CalSensor::kAccel:
      // One chip with the gyro: CAL_ACC0_ID reads the gyro's, nothing asks.
      break;
    case message::CalSensor::kMag:
      mag_calibration_id_.Settle(cfg);
      break;
    case message::CalSensor::kCount:
      break;
  }
}

void FcConfigCache::WriteRcMap(const message::RcMapConfigMsg &value) {
  rc_map_.StartWrite(value, kWriteAttempts);
}

void FcConfigCache::WriteRcCalibration(
    const message::RcCalibrationConfigMsg &value) {
  rc_calibration_.StartWrite(value, kWriteAttempts);
}

bool FcConfigCache::Held(Record record) const {
  switch (record) {
    case Record::kRcMap:
      return rc_map_.value.has_value();
    case Record::kRcCalibration:
      return rc_calibration_.value.has_value();
    case Record::kGyroCalibrationId:
      return gyro_calibration_id_.value.has_value();
    case Record::kMagCalibrationId:
      return mag_calibration_id_.value.has_value();
  }
  return false;
}

bool FcConfigCache::WritePending(Record record) const {
  switch (record) {
    case Record::kRcMap:
      return rc_map_.write.has_value();
    case Record::kRcCalibration:
      return rc_calibration_.write.has_value();
    case Record::kGyroCalibrationId:
      return gyro_calibration_id_.write.has_value();
    case Record::kMagCalibrationId:
      return mag_calibration_id_.write.has_value();
  }
  return false;
}

bool FcConfigCache::Available(Record record, uint32_t now_ms) {
  if (WritePending(record)) {
    return false;
  }
  if (Held(record)) {
    return true;
  }
  Request(record, now_ms);
  return false;
}

void FcConfigCache::Request(Record record, uint32_t now_ms) {
  switch (record) {
    case Record::kRcMap:
      SendRequest(rc_map_.request,
                  message::MakePacket(message::MsgId::kReqRcMap), "RC map",
                  now_ms);
      break;
    case Record::kRcCalibration:
      SendRequest(rc_calibration_.request,
                  message::MakePacket(message::MsgId::kReqRcCalibration),
                  "RC calibration", now_ms);
      break;
    case Record::kGyroCalibrationId:
      SendRequest(gyro_calibration_id_.request,
                  message::MakePacket(
                      message::MsgId::kReqCalibrationId,
                      message::ReqCalibrationIdMsg{
                          .sensor = static_cast<uint8_t>(
                              message::CalSensor::kGyro)}),
                  "gyro calibration ID", now_ms);
      break;
    case Record::kMagCalibrationId:
      SendRequest(mag_calibration_id_.request,
                  message::MakePacket(
                      message::MsgId::kReqCalibrationId,
                      message::ReqCalibrationIdMsg{
                          .sensor = static_cast<uint8_t>(
                              message::CalSensor::kMag)}),
                  "mag calibration ID", now_ms);
      break;
  }
}

void FcConfigCache::SendRequest(RequestState &state,
                                const message::Packet &request,
                                const char *description, uint32_t now_ms) {
  if (state.waiting && !TimeReached(now_ms, state.next_request_ms)) {
    return;
  }
  state.waiting = true;
  state.next_request_ms = TimeAfter(now_ms, kRetryPeriodMs);
  ESP_LOGI(kTag, "Requesting STM32 %s on demand...", description);
  fc_link_->SendPacket(request);
}

void FcConfigCache::Poll(uint32_t now_ms) {
  if (const std::optional<message::RcMapConfigMsg> send =
          rc_map_.Retry(now_ms, ErrorCode::Esp32::kFcLinkRcMapSetFailed)) {
    fc_link_->SendPacket(message::MsgId::kSetRcMapConfig, *send);
  }
  if (const std::optional<message::RcCalibrationConfigMsg> send =
          rc_calibration_.Retry(
              now_ms, ErrorCode::Esp32::kFcLinkRcCalibrationSetFailed)) {
    fc_link_->SendPacket(message::MsgId::kSetRcCalibrationConfig, *send);
  }
}
