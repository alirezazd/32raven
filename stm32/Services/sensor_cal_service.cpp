// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "sensor_cal_service.hpp"

#include <cstdint>

#include "ee_config_storage.hpp"
#include "error_code.hpp"
#include "fc_link.hpp"
#include "icm42688p.hpp"
#include "message.hpp"
#include "panic.hpp"
#include "shared_state.hpp"
#include "time_base.hpp"

namespace {

// A change-detector, not a metric: five unrelated counters summed, so the
// value means nothing and only its movement does. Any of them moving means
// samples went missing between two the stillness check did see, and a gap can
// hide the very motion that check exists to catch -- so a run that sees one
// throws away the window rather than averaging across it.
uint32_t SamplePathFaults(const SharedState &blackboard) {
  const ImuHealth &health = blackboard.GetImuHealth();
  return health.overruns + health.dropped_records + health.invalid_samples +
         health.parse_fails + health.missed_samples;
}

}  // namespace

void GyroCal::Init(const Config &cfg, SharedState &blackboard, Icm42688p &imu) {
  cfg_ = cfg;
  blackboard_ = &blackboard;
  imu_ = &imu;
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
  fault_mark_ = SamplePathFaults(*blackboard_);
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

  const uint32_t faults = SamplePathFaults(*blackboard_);
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

namespace {

// A pose is accepted only when one axis carries nearly all of gravity. The
// bands are wide because the operator is holding a drone against a bench, not
// a jig: 0.85 g admits about 32 degrees of tilt, and the 0.30 g ceiling on the
// other two is what keeps two poses from ever matching at once.
constexpr float kDominantMinG = 0.85f;
constexpr float kOffAxisMaxG = 0.30f;
constexpr float kGravityMps2 = 9.80665f;

}  // namespace

void AccelCal::Init(const Config &cfg, SharedState &blackboard, EE &ee) {
  cfg_ = cfg;
  blackboard_ = &blackboard;
  ee_ = &ee;
  record_ = EeConfigStorage::LoadOrInitImuAccelCalibration(ee);
  Publish();
}

bool AccelCal::IsPlausible(const ee_schema::ImuAccelCalibration &cal) {
  constexpr float kMaxOffsetMps2 = kGravityMps2;
  for (int axis = 0; axis < 3; ++axis) {
    const float gain = cal.gains[axis];
    if (!(gain > 0.5f) || !(gain < 2.0f)) {
      return false;
    }
    const float offset = cal.offsets[axis];
    if (!(offset > -kMaxOffsetMps2) || !(offset < kMaxOffsetMps2)) {
      return false;
    }
  }
  return true;
}

void AccelCal::Publish() {
  AccelCalibration published{};
  if (IsPlausible(record_)) {
    for (int axis = 0; axis < 3; ++axis) {
      published.offsets_mps2[axis] = record_.offsets[axis];
      published.gains[axis] = record_.gains[axis];
    }
  }
  // Anything that failed the test publishes as identity rather than not at
  // all: the estimator has no way to ask again, and untouched samples are the
  // honest answer to a calibration that cannot be trusted.
  blackboard_->UpdateAccelCalibration(published);
}

bool AccelCal::Store(const ee_schema::ImuAccelCalibration &cal) {
  if (!IsPlausible(cal) || ee_ == nullptr) {
    return false;
  }
  const ee_schema::ImuAccelCalibration previous = record_;
  record_ = cal;
  if (EeConfigStorage::SaveImuAccelCalibration(*ee_, record_)) {
    Publish();
    return true;
  }
  // The write is what makes a fit real. Rolling back leaves the board on the
  // calibration it booted with rather than one only RAM knows about.
  record_ = previous;
  return false;
}

bool AccelCal::Start(uint32_t now_us) {
  // Same refusal as the gyro run, for the same reason: this one ends in an
  // EEPROM write rather than a register write, but neither belongs in flight.
  if (blackboard_ == nullptr || blackboard_->IsArmed() ||
      collecting_.load(std::memory_order_relaxed)) {
    return false;
  }
  deadline_us_ =
      now_us + static_cast<uint32_t>(SecondsToMicros(cfg_.timeout_s));
  fault_mark_ = SamplePathFaults(*blackboard_);
  state_ = State::kDetecting;
  sides_done_.store(0, std::memory_order_relaxed);
  collected_.store(false, std::memory_order_relaxed);
  for (int axis = 0; axis < 3; ++axis) {
    up_mps2_[axis] = 0.0f;
    down_mps2_[axis] = 0.0f;
  }
  ResetWindow();
  std::atomic_signal_fence(std::memory_order_release);
  collecting_.store(true, std::memory_order_relaxed);
  return true;
}

void AccelCal::ResetWindow() {
  for (int axis = 0; axis < 3; ++axis) {
    sum_[axis] = 0;
    min_[axis] = INT32_MAX;
    max_[axis] = INT32_MIN;
  }
  samples_ = 0;
  span_us_ = 0;
  still_us_ = 0;
  side_ = AccelSide::kCount;
}

AccelSide AccelCal::Classify(float scale) const {
  if (samples_ == 0u || scale <= 0.0f) {
    return AccelSide::kCount;
  }
  float mean_g[3];
  for (int axis = 0; axis < 3; ++axis) {
    mean_g[axis] =
        (static_cast<float>(sum_[axis]) / static_cast<float>(samples_)) *
        scale / kGravityMps2;
  }

  for (int axis = 0; axis < 3; ++axis) {
    const float own = mean_g[axis] < 0.0f ? -mean_g[axis] : mean_g[axis];
    if (own < kDominantMinG) {
      continue;
    }
    bool clean = true;
    for (int other = 0; other < 3; ++other) {
      if (other == axis) {
        continue;
      }
      const float off = mean_g[other] < 0.0f ? -mean_g[other] : mean_g[other];
      if (off > kOffAxisMaxG) {
        clean = false;
      }
    }
    if (!clean) {
      return AccelSide::kCount;
    }
    // Up and down differ only in the sign the dominant axis carries, which is
    // also why the sign check PX4 runs after collection is not repeated here:
    // the sign is what chose the side.
    const uint8_t base = static_cast<uint8_t>(axis) * 2u;
    return static_cast<AccelSide>(mean_g[axis] > 0.0f ? base : base + 1u);
  }
  return AccelSide::kCount;
}

void AccelCal::Feed(const ImuBurst &burst) {
  std::atomic_signal_fence(std::memory_order_acquire);

  if (burst.dt_us <= 0.0f) {
    return;
  }
  accel_scale_ = burst.accel_scale;

  // A gap in the samples can hide the motion the dispersion check exists to
  // catch, exactly as it can for the gyro run.
  const uint32_t faults = SamplePathFaults(*blackboard_);
  if (faults != fault_mark_) {
    fault_mark_ = faults;
    ResetWindow();
    return;
  }

  for (uint8_t i = 0; i < burst.count; ++i) {
    for (int axis = 0; axis < 3; ++axis) {
      const int32_t a = burst.accel[axis][i];
      if (a < min_[axis]) {
        min_[axis] = a;
      }
      if (a > max_[axis]) {
        max_[axis] = a;
      }
      if (static_cast<uint32_t>(max_[axis] - min_[axis]) >
          cfg_.still_threshold_raw) {
        // Moving. A pose half-collected is discarded rather than kept: the
        // board has left the orientation those samples described.
        ResetWindow();
        return;
      }
      sum_[axis] += a;
    }
    ++samples_;
  }
  const uint32_t elapsed_us = static_cast<uint32_t>(burst.dt_us * burst.count);
  span_us_ += elapsed_us;

  if (state_ == State::kDetecting) {
    still_us_ += elapsed_us;
    if (still_us_ < (cfg_.still_duration_ms * 1000u)) {
      return;
    }
    const AccelSide side = Classify(accel_scale_);
    if (side == AccelSide::kCount) {
      // Still, but in no pose the fit can use -- a corner. Held there it would
      // spin forever, so the window restarts and the operator sees no
      // progress, which is the only signal a corner can give.
      ResetWindow();
      return;
    }
    const uint8_t bit = static_cast<uint8_t>(1u << static_cast<uint8_t>(side));
    if ((sides_done_.load(std::memory_order_relaxed) & bit) != 0u) {
      // Already captured. Left as-is rather than restarted: the operator has
      // not moved yet, and re-averaging a pose would only overwrite it with a
      // noisier copy.
      return;
    }
    side_ = side;
    state_ = State::kCollecting;
    // Detection already averaged a still window of this very pose; the
    // accumulation below wants its own, so the counters start over.
    for (int axis = 0; axis < 3; ++axis) {
      sum_[axis] = 0;
      min_[axis] = INT32_MAX;
      max_[axis] = INT32_MIN;
    }
    samples_ = 0;
    span_us_ = 0;
    return;
  }

  if (span_us_ < (cfg_.side_duration_ms * 1000u) || samples_ == 0u) {
    return;
  }

  const uint8_t index = static_cast<uint8_t>(side_);
  const int axis = index / 2;
  const float mean_mps2 =
      (static_cast<float>(sum_[axis]) / static_cast<float>(samples_)) *
      accel_scale_;
  if ((index % 2u) == 0u) {
    up_mps2_[axis] = mean_mps2;
  } else {
    down_mps2_[axis] = mean_mps2;
  }

  const uint8_t done = static_cast<uint8_t>(
      sides_done_.load(std::memory_order_relaxed) | (1u << index));
  sides_done_.store(done, std::memory_order_relaxed);

  ResetWindow();
  if (done != message::kAccelSideAllMask) {
    state_ = State::kDetecting;
    return;
  }
  collecting_.store(false, std::memory_order_relaxed);
  std::atomic_signal_fence(std::memory_order_release);
  collected_.store(true, std::memory_order_relaxed);
}

AccelCal::State AccelCal::Poll(uint32_t now_us) {
  if (state_ != State::kDetecting && state_ != State::kCollecting) {
    return state_;
  }

  if (blackboard_->IsArmed()) {
    collecting_.store(false, std::memory_order_relaxed);
    state_ = State::kFailed;
    return state_;
  }

  if (collected_.load(std::memory_order_relaxed)) {
    std::atomic_signal_fence(std::memory_order_acquire);
    ee_schema::ImuAccelCalibration cal{};
    for (int axis = 0; axis < 3; ++axis) {
      // The two poses of an axis straddle its zero, so their midpoint is the
      // offset and their separation is two gravities' worth of span. This is
      // PX4's result without PX4's matrix: it builds a 3x3 from the positive
      // poses, inverts it against g and then keeps only the diagonal, which
      // for axis-aligned poses is exactly the pair below.
      const float span = up_mps2_[axis] - down_mps2_[axis];
      if (span <= 0.0f) {
        state_ = State::kFailed;
        return state_;
      }
      cal.offsets[axis] = (up_mps2_[axis] + down_mps2_[axis]) * 0.5f;
      cal.gains[axis] = (2.0f * kGravityMps2) / span;
    }
    state_ = Store(cal) ? State::kApplied : State::kFailed;
    return state_;
  }

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
                            Icm42688p &imu, EE &ee, FcLink &fclink) {
  if (initialized_) {
    Panic(ErrorCode::Stm32::kSensorCalServiceReinit);
  }
  blackboard_ = &blackboard;
  fclink_ = &fclink;
  gyro_.Init(cfg.gyro, blackboard, imu);
  accel_.Init(cfg.accel, blackboard, ee);
  initialized_ = true;
}

bool SensorCalService::StartGyro(uint32_t now_us) {
  // Each calibrator already refuses while it is running, so only the other one
  // has to be tested here.
  if (accel_.Collecting()) {
    return false;
  }
  return gyro_.Start(now_us);
}

bool SensorCalService::StartAccel(uint32_t now_us) {
  if (gyro_.Collecting()) {
    return false;
  }
  return accel_.Start(now_us);
}

void SensorCalService::MaybeCollectBurst() {
  const bool gyro_wants = gyro_.Collecting();
  const bool accel_wants = accel_.Collecting();
  if (!gyro_wants && !accel_wants) {
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
  // One read of the slot, offered to each: the runs are mutually exclusive in
  // practice, and nothing here has to know that.
  if (gyro_wants) {
    gyro_.Feed(burst);
  }
  if (accel_wants) {
    accel_.Feed(burst);
  }
}

void SensorCalService::Poll(uint32_t now_us) {
  // Reported on the edge, not the value: a run sits in kApplied or kFailed
  // until the next Start, and the operator wants one tone, not one per tick.
  const GyroCal::State before = gyro_.Status();
  const GyroCal::State after = gyro_.Poll(now_us);
  if (after != before) {
    ReportGyro(after);
  }

  // The accel run reports on two edges, not one: the outcome as the gyro does,
  // and every captured side, because an operator holding a drone needs to know
  // a pose landed before moving to the next.
  const AccelCal::State accel_before = accel_.Status();
  const uint8_t sides_before = accel_.SidesDone();
  const AccelCal::State accel_after = accel_.Poll(now_us);
  const uint8_t sides_after = accel_.SidesDone();
  if (accel_after != accel_before || sides_after != sides_before) {
    ReportAccel(accel_after, sides_after);
  }
}

void SensorCalService::ReportGyro(GyroCal::State outcome) {
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

void SensorCalService::ReportAccel(AccelCal::State outcome,
                                   uint8_t sides_done) {
  // The state travels as its own wire enum rather than this class's: the two
  // agree today and the link is not the place to assume they always will.
  message::AccelCalState wire = message::AccelCalState::kIdle;
  switch (outcome) {
    case AccelCal::State::kIdle:
      wire = message::AccelCalState::kIdle;
      break;
    case AccelCal::State::kDetecting:
      wire = message::AccelCalState::kDetecting;
      break;
    case AccelCal::State::kCollecting:
      wire = message::AccelCalState::kCollecting;
      break;
    case AccelCal::State::kApplied:
      wire = message::AccelCalState::kApplied;
      break;
    case AccelCal::State::kFailed:
      wire = message::AccelCalState::kFailed;
      break;
  }
  fclink_->SendPacket(
      message::MsgId::kAccelCalStatus,
      message::AccelCalStatusMsg{.state = static_cast<uint8_t>(wire),
                                 .sides_done = sides_done});

  // A tone as well, on the edges an operator holding an airframe cannot watch
  // a screen for: one per captured pose, and one for the outcome.
  message::Tone tone = message::Tone::kBeep;
  if (outcome == AccelCal::State::kApplied) {
    tone = message::Tone::kConfirm;
  } else if (outcome == AccelCal::State::kFailed) {
    tone = message::Tone::kError;
  }
  fclink_->SendPacket(message::MsgId::kTone,
                      message::ToneMsg{.tone = static_cast<uint8_t>(tone)});
}
