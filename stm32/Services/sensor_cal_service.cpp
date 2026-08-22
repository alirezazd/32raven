// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "sensor_cal_service.hpp"

#include <cstdint>

#include "error_code.hpp"
#include "fc_link.hpp"
#include "icm42688p.hpp"
#include "message.hpp"
#include "panic.hpp"
#include "shared_state.hpp"
#include "time_base.hpp"

void GyroCal::Init(const Config &cfg, SharedState &blackboard, Icm42688p &imu) {
  cfg_ = cfg;
  blackboard_ = &blackboard;
  imu_ = &imu;
}

uint32_t GyroCal::SamplePathFaults() const {
  const ImuHealth &health = blackboard_->GetImuHealth();
  return health.overruns + health.dropped_records + health.invalid_samples +
         health.parse_fails + health.missed_samples;
}

bool GyroCal::Start(uint32_t now_us) {
  // Refused while armed: a run ends in a register write that stops the sample
  // path for ~50 ms, well past Sentinel's stall window.
  if (blackboard_ == nullptr || blackboard_->IsArmed() ||
      collecting_.load(std::memory_order_relaxed)) {
    return false;
  }
  deadline_us_ =
      now_us + static_cast<uint32_t>(SecondsToMicros(cfg_.timeout_s));
  fault_mark_ = SamplePathFaults();
  samples_needed_ = 0;
  state_ = State::kCollecting;
  collected_.store(false, std::memory_order_relaxed);
  ResetRun();
  // A compiler fence, not a barrier: the control tick preempts this thread on
  // the same core, so only the compiler could move the setup above past the
  // store that publishes it.
  std::atomic_signal_fence(std::memory_order_release);
  collecting_.store(true, std::memory_order_relaxed);
  return true;
}

void GyroCal::ResetRun() {
  for (int axis = 0; axis < 3; ++axis) {
    sum_[axis] = 0;
    min_[axis] = INT32_MAX;
    max_[axis] = INT32_MIN;
  }
  samples_ = 0;
  span_us_ = 0;
}

void GyroCal::Feed(const ImuBurst &burst) {
  std::atomic_signal_fence(std::memory_order_acquire);

  // The chip's own dt is what says how many samples an ODR owes us, so the
  // target is set from the first burst rather than from a configured rate.
  if (samples_needed_ == 0u) {
    if (burst.dt_us <= 0.0f) {
      return;
    }
    const float needed =
        static_cast<float>(SecondsToMicros(cfg_.duration_s)) / burst.dt_us;
    samples_needed_ = static_cast<uint32_t>(needed) + 1u;
    gyro_scale_ = burst.gyro_scale;
  }

  const uint32_t faults = SamplePathFaults();
  if (faults != fault_mark_) {
    fault_mark_ = faults;
    ResetRun();
    return;
  }

  for (uint8_t i = 0; i < burst.count; ++i) {
    for (int axis = 0; axis < 3; ++axis) {
      const int32_t g = burst.gyro[axis][i];
      if (g < min_[axis]) {
        min_[axis] = g;
      }
      if (g > max_[axis]) {
        max_[axis] = g;
      }
      if (static_cast<uint32_t>(max_[axis] - min_[axis]) >
          cfg_.still_threshold_raw) {
        ResetRun();
        return;
      }
      sum_[axis] += g;
    }
    ++samples_;
  }
  span_us_ += static_cast<uint32_t>(burst.dt_us * burst.count);

  if (samples_ >= samples_needed_ &&
      span_us_ >= SecondsToMicros(cfg_.duration_s)) {
    collecting_.store(false, std::memory_order_relaxed);
    std::atomic_signal_fence(std::memory_order_release);
    collected_.store(true, std::memory_order_relaxed);
  }
}

GyroCal::State GyroCal::Poll(uint32_t now_us) {
  if (state_ != State::kCollecting) {
    return state_;
  }

  // Latched here rather than on the control tick, so the tick's gate stays one
  // load. Up to a main tick of samples lands after the arm, and goes out with
  // the run that abandons them.
  if (blackboard_->IsArmed()) {
    collecting_.store(false, std::memory_order_relaxed);
    state_ = State::kFailed;
    return state_;
  }

  if (collected_.load(std::memory_order_relaxed)) {
    std::atomic_signal_fence(std::memory_order_acquire);
    float bias_body[3] = {0.0f, 0.0f, 0.0f};
    for (int axis = 0; axis < 3; ++axis) {
      bias_body[axis] =
          (static_cast<float>(sum_[axis]) / static_cast<float>(samples_)) *
          gyro_scale_;
    }
    imu_->ApplyGyroOffsets(bias_body);
    state_ = State::kApplied;
    return state_;
  }

  // The only exit for a run the airframe will not hold still for: every other
  // failure restarts collection, so this is what stops it retrying forever.
  if (static_cast<int32_t>(now_us - deadline_us_) >= 0) {
    collecting_.store(false, std::memory_order_relaxed);
    state_ = State::kFailed;
  }
  return state_;
}

SensorCalService &SensorCalService::GetInstance() {
  static SensorCalService instance;
  return instance;
}

void SensorCalService::Init(const Config &cfg, SharedState &blackboard,
                            Icm42688p &imu, FcLink &fclink) {
  if (initialized_) {
    Panic(ErrorCode::Stm32::kSensorCalServiceReinit);
  }
  blackboard_ = &blackboard;
  fclink_ = &fclink;
  gyro_.Init(cfg.gyro, blackboard, imu);
  initialized_ = true;
}

void SensorCalService::MaybeCollectBurst() {
  if (!gyro_.Collecting()) {
    return;
  }
  const ImuBurstSlot &slot = blackboard_->GetImuBurstSlot();

  // Runs after the AHRS has cleared `fresh`, so there is no flag to hold and
  // the interrupt may overwrite the slot at any point below. See ImuBurstSlot.
  const uint32_t seq = slot.seq;
  if ((seq & 1u) != 0u || seq == last_seq_) {
    return;
  }
  const ImuBurst burst = slot.burst;
  if (slot.seq != seq) {
    return;
  }
  last_seq_ = seq;

  if (burst.count == 0u) {
    return;
  }
  gyro_.Feed(burst);
}

void SensorCalService::Poll(uint32_t now_us) {
  // Reported on the edge, not the value: a run sits in kApplied or kFailed
  // until the next Start, and the operator wants one tone, not one per tick.
  const GyroCal::State before = gyro_.Status();
  const GyroCal::State after = gyro_.Poll(now_us);
  if (after != before) {
    Report(after);
  }
}

void SensorCalService::Report(GyroCal::State outcome) {
  if (outcome != GyroCal::State::kApplied &&
      outcome != GyroCal::State::kFailed) {
    return;
  }
  const message::Tone tone = (outcome == GyroCal::State::kApplied)
                                 ? message::Tone::kConfirm
                                 : message::Tone::kError;
  fclink_->SendPacket(message::MsgId::kTone,
                      message::ToneMsg{.tone = static_cast<uint8_t>(tone)});
}
