// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "sensor_cal_service.hpp"

#include <bit>
#include <cmath>
#include <cstdint>
#include <span>

#include <Eigen/Geometry>

#include "ee_config_storage.hpp"
#include "error_code.hpp"
#include "fc_link.hpp"
#include "math/attitude_euler.hpp"
#include "message.hpp"
#include "panic.hpp"
#include "shared_state.hpp"
#include "stm32_config.hpp"
#include "time_base.hpp"

// The sample buffer is sized for the range's ceiling, and a run needs its
// six rest holds, six turn waits and six poses inside the deadline.
static_assert(kSensorCalConfig.mag.points_per_side <= MagCal::kMaxPointsPerSide,
              "STM32_SENSOR_CAL_MAG_POINTS_PER_SIDE exceeds the sample buffer");
static_assert(kSensorCalConfig.mag.timeout_s >
                  message::kAccelSideCount *
                      ((kSensorCalConfig.mag.still_duration_ms / 1000u) +
                       MagCal::kRotateTimeoutS +
                       kSensorCalConfig.mag.side_duration_s),
              "compass calibration deadline cannot cover six poses -- raise "
              "STM32_SENSOR_CAL_MAG_TIMEOUT_S");

namespace {

// A change-detector, not a metric: five unrelated counters summed, so the
// value means nothing and only its movement does. Any of them moving means
// samples went missing between two the stillness check did see, and a gap can
// hide the very motion that check exists to catch -- so a run that sees one
// throws away the window rather than averaging across it.
constexpr float kDegToRad = 0.01745329252f;
constexpr float kRadToMdps = 1000.0f / kDegToRad;

uint32_t SamplePathFaults(const SharedState &blackboard) {
  const ImuHealth &health = blackboard.GetImuHealth();
  return health.overruns + health.dropped_records + health.invalid_samples +
         health.parse_fails + health.missed_samples;
}

// Zero until the scale is known, which holds the run rather than passing a
// window nothing was checked against.
uint32_t StillThresholdCounts(uint32_t threshold_si_milli, float unit_si,
                              float count_to_si) {
  if (count_to_si <= 0.0f) {
    return 0;
  }
  const float si = static_cast<float>(threshold_si_milli) * 0.001f * unit_si;
  return static_cast<uint32_t>(si / count_to_si);
}

}  // namespace

void GyroCal::Init(const Config &cfg, SharedState &blackboard, EE &ee) {
  cfg_ = cfg;
  blackboard_ = &blackboard;
  ee_ = &ee;
  const ee_schema::ImuGyroCalibration stored =
      EeConfigStorage::LoadOrInitImuGyroCalibration(ee);
  if (stored.calibrated != 0u && IsPlausible(stored.offsets_rad_s)) {
    Publish(stored.offsets_rad_s, GyroCalSource::kStored);
  }
}

bool GyroCal::IsPlausible(const float offsets_rad_s[3]) const {
  const float max_rad_s =
      static_cast<float>(cfg_.max_offset_mdps) * 0.001f * kDegToRad;
  for (int axis = 0; axis < 3; ++axis) {
    const float offset = offsets_rad_s[axis];
    if (!(offset > -max_rad_s) || !(offset < max_rad_s)) {
      return false;
    }
  }
  return true;
}

void GyroCal::Publish(const float offsets_rad_s[3], GyroCalSource source) {
  GyroCalibration cal{};
  for (int axis = 0; axis < 3; ++axis) {
    cal.offsets_rad_s[axis] = offsets_rad_s[axis];
  }
  cal.source = source;
  blackboard_->UpdateGyroCalibration(cal);
}

bool GyroCal::Store() {
  if (ee_ == nullptr) {
    return false;
  }
  ee_schema::ImuGyroCalibration record{};
  for (int axis = 0; axis < 3; ++axis) {
    record.offsets_rad_s[axis] = result_rad_s_[axis];
  }
  record.calibrated = 1u;
  return EeConfigStorage::SaveImuGyroCalibration(*ee_, record);
}

bool GyroCal::Start(uint32_t now_us) {
  // Refused while armed: the offsets step the rate loop the moment they land.
  if (blackboard_ == nullptr || blackboard_->IsArmed() ||
      collecting_.load(std::memory_order_relaxed)) {
    return false;
  }
  deadline_us_ =
      now_us + static_cast<uint32_t>(SecondsToMicros(cfg_.timeout_s));
  fault_mark_ = SamplePathFaults(*blackboard_);
  samples_needed_ = 0;
  failure_ = Failure::kNone;
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

void GyroCal::Cancel() {
  if (state_ != State::kCollecting) {
    return;
  }
  collecting_.store(false, std::memory_order_relaxed);
  ResetRun();
  state_ = State::kIdle;
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

  const uint32_t still_counts =
      StillThresholdCounts(cfg_.still_threshold_mdps, kDegToRad, gyro_scale_);

  for (uint8_t i = 0; i < burst.count; ++i) {
    for (int axis = 0; axis < 3; ++axis) {
      const int32_t g = burst.gyro[axis][i];
      if (g < min_[axis]) {
        min_[axis] = g;
      }
      if (g > max_[axis]) {
        max_[axis] = g;
      }
      if (static_cast<uint32_t>(max_[axis] - min_[axis]) > still_counts) {
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
    failure_ = Failure::kArmed;
    state_ = State::kFailed;
    return state_;
  }

  if (collected_.load(std::memory_order_relaxed)) {
    std::atomic_signal_fence(std::memory_order_acquire);
    for (int axis = 0; axis < 3; ++axis) {
      result_rad_s_[axis] =
          (static_cast<float>(sum_[axis]) / static_cast<float>(samples_)) *
          gyro_scale_;
    }
    if (!IsPlausible(result_rad_s_)) {
      failure_ = Failure::kBias;
      state_ = State::kFailed;
      return state_;
    }
    Publish(result_rad_s_, GyroCalSource::kSession);
    state_ = State::kApplied;
    return state_;
  }

  // The only exit for a run the airframe will not hold still for: every other
  // failure restarts collection, so this is what stops it retrying forever.
  if (static_cast<int32_t>(now_us - deadline_us_) >= 0) {
    collecting_.store(false, std::memory_order_relaxed);
    failure_ = Failure::kNeverStill;
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
  // Same refusal as the gyro run: neither result belongs landing in flight.
  if (blackboard_ == nullptr || blackboard_->IsArmed() ||
      collecting_.load(std::memory_order_relaxed)) {
    return false;
  }
  deadline_us_ =
      now_us + static_cast<uint32_t>(SecondsToMicros(cfg_.timeout_s));
  fault_mark_ = SamplePathFaults(*blackboard_);
  failure_ = Failure::kNone;
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

void AccelCal::Cancel() {
  if (state_ != State::kDetecting && state_ != State::kCollecting) {
    return;
  }
  collecting_.store(false, std::memory_order_relaxed);
  state_ = State::kCancelled;
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

  const uint32_t still_counts =
      StillThresholdCounts(cfg_.still_threshold_mg, kGravityMps2, accel_scale_);

  for (uint8_t i = 0; i < burst.count; ++i) {
    for (int axis = 0; axis < 3; ++axis) {
      const int32_t a = burst.accel[axis][i];
      if (a < min_[axis]) {
        min_[axis] = a;
      }
      if (a > max_[axis]) {
        max_[axis] = a;
      }
      if (static_cast<uint32_t>(max_[axis] - min_[axis]) > still_counts) {
        // Moving. A pose half-collected is discarded rather than kept: the
        // board has left the orientation those samples described. Detection
        // has to run again with it, because the window it cleared held which
        // pose was being averaged -- resuming without one would index the
        // solve's arrays by kCount.
        ResetWindow();
        state_ = State::kDetecting;
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
    failure_ = Failure::kArmed;
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
        failure_ = Failure::kGeometry;
        state_ = State::kFailed;
        return state_;
      }
      cal.offsets[axis] = (up_mps2_[axis] + down_mps2_[axis]) * 0.5f;
      cal.gains[axis] = (2.0f * kGravityMps2) / span;
    }
    if (!Store(cal)) {
      failure_ = Failure::kStore;
      state_ = State::kFailed;
      return state_;
    }
    state_ = State::kApplied;
    return state_;
  }

  if (static_cast<int32_t>(now_us - deadline_us_) >= 0) {
    collecting_.store(false, std::memory_order_relaxed);
    failure_ = Failure::kTimeout;
    state_ = State::kFailed;
  }
  return state_;
}

namespace {

constexpr float kGaussPerMicrotesla = 0.01f;
constexpr float kMicroteslaPerGauss = 100.0f;
// PX4's rest detector and pose test, lenient mode.
constexpr float kRestEmaTauS = 0.5f;
constexpr float kPoseErrorMps2 = 5.0f;
// PX4's "did the operator turn it": any one axis through this much.
constexpr float kTurnRad = 0.5f;
// A poll that came late integrates as if it had not: a torn timestamp or a
// stalled loop must not read as a turn.
constexpr uint32_t kMaxPollDtUs = 5000;

}  // namespace

void MagCal::Init(const Config &cfg, SharedState &blackboard, EE &ee) {
  cfg_ = cfg;
  blackboard_ = &blackboard;
  ee_ = &ee;
  record_ = EeConfigStorage::LoadOrInitMagnetometerCalibration(ee);
  Publish();
}

bool MagCal::IsPlausible(const ee_schema::MagnetometerCalibration &cal) {
  constexpr float kMaxOffsetUt = 200.0f;
  for (int axis = 0; axis < 3; ++axis) {
    if (!(cal.diag[axis] > 0.5f) || !(cal.diag[axis] < 2.0f)) {
      return false;
    }
    if (!(cal.offdiag[axis] > -0.5f) || !(cal.offdiag[axis] < 0.5f)) {
      return false;
    }
    if (!(cal.offsets_ut[axis] > -kMaxOffsetUt) ||
        !(cal.offsets_ut[axis] < kMaxOffsetUt)) {
      return false;
    }
  }
  return true;
}

void MagCal::Publish() {
  MagCalibration published{};
  if (record_.calibrated != 0u && IsPlausible(record_)) {
    for (int axis = 0; axis < 3; ++axis) {
      published.offsets_ut[axis] = record_.offsets_ut[axis];
      published.soft_iron[axis][axis] = record_.diag[axis];
    }
    published.soft_iron[0][1] = record_.offdiag[0];
    published.soft_iron[1][0] = record_.offdiag[0];
    published.soft_iron[0][2] = record_.offdiag[1];
    published.soft_iron[2][0] = record_.offdiag[1];
    published.soft_iron[1][2] = record_.offdiag[2];
    published.soft_iron[2][1] = record_.offdiag[2];
    published.calibrated = true;
  }
  blackboard_->UpdateMagCalibration(published);
}

bool MagCal::Store(const ee_schema::MagnetometerCalibration &cal) {
  if (!IsPlausible(cal) || ee_ == nullptr) {
    return false;
  }
  const ee_schema::MagnetometerCalibration previous = record_;
  record_ = cal;
  if (EeConfigStorage::SaveMagnetometerCalibration(*ee_, record_)) {
    Publish();
    return true;
  }
  record_ = previous;
  return false;
}

bool MagCal::Start(uint32_t now_us) {
  if (blackboard_ == nullptr || blackboard_->IsArmed() || Running() ||
      !blackboard_->IsControlLoopRunning() ||
      blackboard_->GetMagnetometer().timestamp_us == 0u) {
    return false;
  }
  deadline_us_ =
      now_us + static_cast<uint32_t>(SecondsToMicros(cfg_.timeout_s));
  last_poll_us_ = now_us;
  failure_ = Failure::kNone;
  sides_done_ = 0;
  count_ = 0;
  side_count_ = 0;
  sphere_radius_ = MagFit::kMinRadius;
  const MagnetometerData &mag = blackboard_->GetMagnetometer();
  last_sample_count_ = mag.sample_count;
  last_overflow_count_ = mag.overflow_count;
  BeginDetecting();
  return true;
}

void MagCal::Cancel() {
  if (Running()) {
    state_ = State::kCancelled;
  }
}

void MagCal::BeginDetecting() {
  state_ = State::kDetecting;
  side_ = AccelSide::kCount;
  for (int axis = 0; axis < 3; ++axis) {
    accel_ema_[axis] = 0.0f;
    accel_disp_[axis] = 0.0f;
  }
  still_ = false;
  still_since_us_ = 0;
}

MagCal::State MagCal::Poll(uint32_t now_us) {
  if (!Running()) {
    return state_;
  }
  if (blackboard_->IsArmed()) {
    failure_ = Failure::kArmed;
    state_ = State::kFailed;
    return state_;
  }
  if (static_cast<int32_t>(now_us - deadline_us_) >= 0) {
    failure_ = Failure::kTimeout;
    state_ = State::kFailed;
    return state_;
  }
  uint32_t dt_us = now_us - last_poll_us_;
  if (dt_us > kMaxPollDtUs) {
    dt_us = kMaxPollDtUs;
  }
  last_poll_us_ = now_us;
  const float dt_s = static_cast<float>(dt_us) * 1e-6f;

  switch (state_) {
    case State::kDetecting:
      PollDetecting(now_us, dt_s);
      break;
    case State::kRotating:
      PollRotating(now_us, dt_s);
      break;
    case State::kCollecting:
      PollCollecting(now_us);
      break;
    case State::kFitting:
      PollFitting();
      break;
    case State::kIdle:
    case State::kApplied:
    case State::kFailed:
    case State::kCancelled:
      break;
  }
  return state_;
}

void MagCal::PollDetecting(uint32_t now_us, float dt_s) {
  const Eigen::Vector3f accel = blackboard_->GetEstimate().accel_body_mps2;
  const float thr =
      static_cast<float>(cfg_.still_threshold_mg) * 0.001f * kGravityMps2;
  const float thr2 = thr * thr;
  const float w = dt_s / kRestEmaTauS;

  bool still = true;
  bool moving = false;
  for (int axis = 0; axis < 3; ++axis) {
    float d = accel(axis) - accel_ema_[axis];
    accel_ema_[axis] += d * w;
    d = d * d;
    accel_disp_[axis] *= 1.0f - w;
    if (d > thr2 * 8.0f) {
      d = thr2 * 8.0f;
    }
    if (d > accel_disp_[axis]) {
      accel_disp_[axis] = d;
    }
    still = still && accel_disp_[axis] < thr2;
    moving = moving || accel_disp_[axis] > thr2 * 4.0f;
  }

  if (!still) {
    if (moving) {
      still_ = false;
    }
    return;
  }
  if (!still_) {
    still_ = true;
    still_since_us_ = now_us;
    return;
  }
  if (now_us - still_since_us_ < cfg_.still_duration_ms * 1000u) {
    return;
  }

  const AccelSide side = Classify();
  if (side == AccelSide::kCount ||
      (sides_done_ & (1u << static_cast<uint8_t>(side))) != 0u) {
    // A corner, or a pose already taken: hold again rather than re-test
    // every tick, and let the operator move on.
    still_ = false;
    return;
  }
  side_ = side;
  state_ = State::kRotating;
  for (int axis = 0; axis < 3; ++axis) {
    gyro_integral_[axis] = 0.0f;
  }
  stage_deadline_us_ =
      now_us + static_cast<uint32_t>(SecondsToMicros(kRotateTimeoutS));
}

AccelSide MagCal::Classify() const {
  for (int axis = 0; axis < 3; ++axis) {
    bool others_level = true;
    for (int other = 0; other < 3; ++other) {
      if (other != axis && !(std::fabs(accel_ema_[other]) < kPoseErrorMps2)) {
        others_level = false;
      }
    }
    if (!others_level) {
      continue;
    }
    if (std::fabs(accel_ema_[axis] - kGravityMps2) < kPoseErrorMps2) {
      return static_cast<AccelSide>(axis * 2);
    }
    if (std::fabs(accel_ema_[axis] + kGravityMps2) < kPoseErrorMps2) {
      return static_cast<AccelSide>((axis * 2) + 1);
    }
  }
  return AccelSide::kCount;
}

void MagCal::PollRotating(uint32_t now_us, float dt_s) {
  const Eigen::Vector3f gyro = blackboard_->GetEstimate().gyro_body_rad_s;
  bool turned = false;
  for (int axis = 0; axis < 3; ++axis) {
    gyro_integral_[axis] += gyro(axis) * dt_s;
    turned = turned || std::fabs(gyro_integral_[axis]) >= kTurnRad;
  }
  if (turned) {
    state_ = State::kCollecting;
    side_count_ = 0;
    stage_deadline_us_ =
        now_us + static_cast<uint32_t>(SecondsToMicros(cfg_.side_duration_s));
    return;
  }
  if (static_cast<int32_t>(now_us - stage_deadline_us_) >= 0) {
    failure_ = Failure::kNoTurn;
    state_ = State::kFailed;
  }
}

void MagCal::PollCollecting(uint32_t now_us) {
  (void)TakeSample();
  // Either ends the pose, as PX4 has it: a pose the operator turned slowly
  // on gives fewer points, not a longer wait.
  if (side_count_ >= cfg_.points_per_side ||
      static_cast<int32_t>(now_us - stage_deadline_us_) >= 0) {
    EndSide();
  }
}

bool MagCal::TakeSample() {
  const MagnetometerData &mag = blackboard_->GetMagnetometer();
  if (mag.sample_count == last_sample_count_) {
    return false;
  }
  const bool overflowed = mag.overflow_count != last_overflow_count_;
  last_sample_count_ = mag.sample_count;
  last_overflow_count_ = mag.overflow_count;
  if (overflowed || count_ >= kMaxPoints) {
    return false;
  }

  const float x = mag.x * kGaussPerMicrotesla;
  const float y = mag.y * kGaussPerMicrotesla;
  const float z = mag.z * kGaussPerMicrotesla;
  if (count_ == 0u) {
    const float norm = std::sqrt((x * x) + (y * y) + (z * z));
    sphere_radius_ = norm < MagFit::kMinRadius   ? MagFit::kMinRadius
                     : norm > MagFit::kMaxRadius ? MagFit::kMaxRadius
                                                 : norm;
  }

  // PX4's spacing: samples closer than a share of the sphere are the same
  // point seen twice, and a pose turned slowly would otherwise fill its quota
  // from one patch of it.
  const float total =
      static_cast<float>(cfg_.points_per_side * message::kAccelSideCount);
  const float min_dist =
      std::fabs(5.4f * sphere_radius_ / std::sqrt(total)) / 3.0f;
  const float min_dist2 = min_dist * min_dist;
  for (uint32_t i = 0; i < count_; ++i) {
    const float dx = x - x_[i];
    const float dy = y - y_[i];
    const float dz = z - z_[i];
    if ((dx * dx) + (dy * dy) + (dz * dz) < min_dist2) {
      return false;
    }
  }
  x_[count_] = x;
  y_[count_] = y;
  z_[count_] = z;
  ++count_;
  ++side_count_;
  return true;
}

void MagCal::EndSide() {
  sides_done_ |= static_cast<uint8_t>(1u << static_cast<uint8_t>(side_));
  side_ = AccelSide::kCount;
  side_count_ = 0;
  if (sides_done_ != message::kAccelSideAllMask) {
    BeginDetecting();
    return;
  }
  state_ = State::kFitting;
  fitting_ellipsoid_ = false;
  MagFitParams seed{};
  seed.radius = sphere_radius_;
  fit_.Start(MagFit::Stage::kSphere, seed);
}

void MagCal::PollFitting() {
  const std::span<const float> x(x_, count_);
  const std::span<const float> y(y_, count_);
  const std::span<const float> z(z_, count_);
  const MagFit::Status status = fit_.Step(x, y, z);
  if (status == MagFit::Status::kRunning) {
    return;
  }

  if (!fitting_ellipsoid_) {
    if (status == MagFit::Status::kFailed) {
      failure_ = Failure::kSphereFit;
      state_ = State::kFailed;
      return;
    }
    sphere_result_ = fit_.Params();
    result_cost_ = fit_.Cost();
    fitting_ellipsoid_ = true;
    fit_.Start(MagFit::Stage::kEllipsoid, sphere_result_);
    return;
  }

  // An ellipsoid that would not converge leaves the sphere standing: offsets
  // alone are most of the correction.
  if (status == MagFit::Status::kConverged) {
    result_ = fit_.Params();
    result_cost_ = fit_.Cost();
  } else {
    result_ = sphere_result_;
  }
  if (!IsSane(result_)) {
    failure_ = Failure::kBounds;
    state_ = State::kFailed;
    return;
  }

  ee_schema::MagnetometerCalibration cal{};
  for (int axis = 0; axis < 3; ++axis) {
    cal.offsets_ut[axis] = result_.offset(axis) * kMicroteslaPerGauss;
    cal.diag[axis] = result_.diag(axis);
    cal.offdiag[axis] = result_.offdiag(axis);
  }
  cal.calibrated = 1u;
  if (!Store(cal)) {
    failure_ = Failure::kStore;
    state_ = State::kFailed;
    return;
  }
  state_ = State::kApplied;
}

bool MagCal::IsSane(const MagFitParams &p) {
  if (!std::isfinite(p.radius) || !p.offset.allFinite() ||
      !p.diag.allFinite() || !p.offdiag.allFinite()) {
    return false;
  }
  if (p.radius < MagFit::kMinRadius || p.radius >= MagFit::kMaxRadius) {
    return false;
  }
  return p.diag(0) > 0.0f && p.diag(1) > 0.0f && p.diag(2) > 0.0f;
}

uint8_t MagCal::Progress() const {
  if (state_ == State::kFitting || state_ == State::kApplied) {
    return 100;
  }
  const uint32_t per = cfg_.points_per_side;
  const uint32_t done = static_cast<uint32_t>(std::popcount(sides_done_));
  const uint32_t taken = (done * per) + (side_count_ < per ? side_count_ : per);
  return static_cast<uint8_t>((100u * taken) /
                              (per * message::kAccelSideCount));
}


namespace {

// PX4's motion bound and its excess bound: half a degree of spread within a
// window is rest; a mean past 0.8 rad is a mount, not a trim.
constexpr float kLevelStillRad = 0.5f * kDegToRad;
constexpr float kLevelExcessRad = 0.8f;
// The estimator stopping for this long mid-run ends it.
constexpr uint32_t kLevelEstimateTimeoutUs = 100000;

// v_body = R * v_board, PX4's order: yaw about Z, then pitch, then roll.
Eigen::Matrix3f TrimRotation(const ee_schema::BoardTrim &trim) {
  return (Eigen::AngleAxisf(trim.yaw_deg * kDegToRad, Eigen::Vector3f::UnitZ()) *
          Eigen::AngleAxisf(trim.pitch_deg * kDegToRad,
                            Eigen::Vector3f::UnitY()) *
          Eigen::AngleAxisf(trim.roll_deg * kDegToRad, Eigen::Vector3f::UnitX()))
      .toRotationMatrix();
}

}  // namespace

void LevelCal::Init(SharedState &blackboard, EE &ee) {
  blackboard_ = &blackboard;
  ee_ = &ee;
  record_ = EeConfigStorage::LoadOrInitBoardTrim(ee);
  Publish();
}

bool LevelCal::IsPlausible(const ee_schema::BoardTrim &trim) {
  return message::IsBoardTrimConfigValid(message::BoardTrimConfigMsg{
      .roll_deg = trim.roll_deg,
      .pitch_deg = trim.pitch_deg,
      .yaw_deg = trim.yaw_deg});
}

void LevelCal::Publish() {
  BoardTrim published{};
  if (IsPlausible(record_)) {
    published.rotation = TrimRotation(record_);
  }
  blackboard_->UpdateBoardTrim(published);
}

bool LevelCal::Store(const ee_schema::BoardTrim &trim) {
  if (!IsPlausible(trim) || ee_ == nullptr) {
    return false;
  }
  const ee_schema::BoardTrim previous = record_;
  record_ = trim;
  if (EeConfigStorage::SaveBoardTrim(*ee_, record_)) {
    Publish();
    return true;
  }
  record_ = previous;
  return false;
}

message::BoardTrimConfigMsg LevelCal::Trim() const {
  return message::BoardTrimConfigMsg{.roll_deg = record_.roll_deg,
                                     .pitch_deg = record_.pitch_deg,
                                     .yaw_deg = record_.yaw_deg};
}

bool LevelCal::SetTrim(const message::BoardTrimConfigMsg &trim) {
  if (Running() || !message::IsBoardTrimConfigValid(trim)) {
    return false;
  }
  ee_schema::BoardTrim record = record_;
  record.roll_deg = trim.roll_deg;
  record.pitch_deg = trim.pitch_deg;
  record.yaw_deg = trim.yaw_deg;
  return Store(record);
}

bool LevelCal::Start(uint32_t now_us) {
  if (blackboard_ == nullptr || blackboard_->IsArmed() || Running() ||
      !blackboard_->IsControlLoopRunning()) {
    return false;
  }
  failure_ = Failure::kNone;
  windows_ = 0;
  last_estimate_us_ = blackboard_->GetEstimate().timestamp_us;
  last_sample_us_ = now_us;
  state_ = State::kCollecting;
  BeginWindow(now_us);
  return true;
}

void LevelCal::Cancel() {
  if (Running()) {
    state_ = State::kCancelled;
  }
}

void LevelCal::BeginWindow(uint32_t now_us) {
  window_start_us_ = now_us;
  roll_sum_ = 0.0f;
  pitch_sum_ = 0.0f;
  count_ = 0;
  roll_min_ = 100.0f;
  roll_max_ = -100.0f;
  pitch_min_ = 100.0f;
  pitch_max_ = -100.0f;
}

LevelCal::State LevelCal::Poll(uint32_t now_us) {
  if (!Running()) {
    return state_;
  }
  if (blackboard_->IsArmed()) {
    failure_ = Failure::kArmed;
    state_ = State::kFailed;
    return state_;
  }

  const EstimatorState &estimate = blackboard_->GetEstimate();
  if (estimate.timestamp_us != last_estimate_us_) {
    last_estimate_us_ = estimate.timestamp_us;
    last_sample_us_ = now_us;
    // The board's own attitude: the estimate is of the trimmed frame, so
    // the stored trim is composed back in before the angles are read.
    const Eigen::Quaternionf board =
        estimate.attitude_world_to_body *
        Eigen::Quaternionf(TrimRotation(record_));
    const math::EulerZyx angles = math::EulerZyxFromQuaternion(board);
    roll_sum_ += angles.roll;
    pitch_sum_ += angles.pitch;
    ++count_;
    roll_min_ = angles.roll < roll_min_ ? angles.roll : roll_min_;
    roll_max_ = angles.roll > roll_max_ ? angles.roll : roll_max_;
    pitch_min_ = angles.pitch < pitch_min_ ? angles.pitch : pitch_min_;
    pitch_max_ = angles.pitch > pitch_max_ ? angles.pitch : pitch_max_;
  } else if (now_us - last_sample_us_ >= kLevelEstimateTimeoutUs) {
    failure_ = Failure::kNoEstimate;
    state_ = State::kFailed;
    return state_;
  }

  if (now_us - window_start_us_ >= kWindowUs) {
    EndWindow(now_us);
  }
  return state_;
}

void LevelCal::EndWindow(uint32_t now_us) {
  const bool still = count_ > 0u && (roll_max_ - roll_min_) < kLevelStillRad &&
                     (pitch_max_ - pitch_min_) < kLevelStillRad;
  if (!still) {
    if (++windows_ >= kMaxWindows) {
      failure_ = Failure::kMotion;
      state_ = State::kFailed;
      return;
    }
    BeginWindow(now_us);
    return;
  }

  const float roll = roll_sum_ / static_cast<float>(count_);
  const float pitch = pitch_sum_ / static_cast<float>(count_);
  result_roll_deg_ = roll / kDegToRad;
  result_pitch_deg_ = pitch / kDegToRad;
  if (std::fabs(roll) > kLevelExcessRad || std::fabs(pitch) > kLevelExcessRad) {
    failure_ = Failure::kExcess;
    state_ = State::kFailed;
    return;
  }

  // Yaw is the operator's to set; no rest measures it.
  ee_schema::BoardTrim trim = record_;
  trim.roll_deg = result_roll_deg_;
  trim.pitch_deg = result_pitch_deg_;
  if (!Store(trim)) {
    failure_ = Failure::kStore;
    state_ = State::kFailed;
    return;
  }
  state_ = State::kApplied;
}

namespace {

// The state travels as the wire's enum rather than a class's: they agree
// today and the link is not the place to assume they always will.
message::CalState ToWire(AccelCal::State state) {
  switch (state) {
    case AccelCal::State::kIdle:
      return message::CalState::kIdle;
    case AccelCal::State::kDetecting:
      return message::CalState::kDetecting;
    case AccelCal::State::kCollecting:
      return message::CalState::kCollecting;
    case AccelCal::State::kApplied:
      return message::CalState::kApplied;
    case AccelCal::State::kFailed:
      return message::CalState::kFailed;
    case AccelCal::State::kCancelled:
      return message::CalState::kCancelled;
  }
  return message::CalState::kIdle;
}

message::CalState ToWire(MagCal::State state) {
  switch (state) {
    case MagCal::State::kIdle:
      return message::CalState::kIdle;
    case MagCal::State::kDetecting:
      return message::CalState::kDetecting;
    case MagCal::State::kRotating:
      return message::CalState::kRotating;
    case MagCal::State::kCollecting:
      return message::CalState::kCollecting;
    case MagCal::State::kFitting:
      return message::CalState::kFitting;
    case MagCal::State::kApplied:
      return message::CalState::kApplied;
    case MagCal::State::kFailed:
      return message::CalState::kFailed;
    case MagCal::State::kCancelled:
      return message::CalState::kCancelled;
  }
  return message::CalState::kIdle;
}

message::CalState ToWire(LevelCal::State state) {
  switch (state) {
    case LevelCal::State::kIdle:
      return message::CalState::kIdle;
    case LevelCal::State::kCollecting:
      return message::CalState::kCollecting;
    case LevelCal::State::kApplied:
      return message::CalState::kApplied;
    case LevelCal::State::kFailed:
      return message::CalState::kFailed;
    case LevelCal::State::kCancelled:
      return message::CalState::kCancelled;
  }
  return message::CalState::kIdle;
}

}  // namespace

SensorCalService &SensorCalService::GetInstance() {
  static SensorCalService instance;
  return instance;
}

void SensorCalService::Init(const Config &cfg, SharedState &blackboard, EE &ee,
                            FcLink &fclink) {
  if (initialized_) {
    Panic(ErrorCode::Stm32::kSensorCalServiceReinit);
  }
  blackboard_ = &blackboard;
  fclink_ = &fclink;
  gyro_.Init(cfg.gyro, blackboard, ee);
  accel_.Init(cfg.accel, blackboard, ee);
  mag_.Init(cfg.mag, blackboard, ee);
  level_.Init(blackboard, ee);
  initialized_ = true;
}

bool SensorCalService::StartGyro(uint32_t now_us) {
  // Each calibrator already refuses while it is running, so only the others
  // have to be tested here.
  if (accel_.Collecting() || mag_.Running() || level_.Running()) {
    return false;
  }
  if (gyro_auto_) {
    gyro_.Cancel();
  }
  const bool started = gyro_.Start(now_us);
  if (started) {
    gyro_auto_ = false;
    // A host run has a page waiting on it. Poll compares the run's state
    // before and after its own step, so the edges made here and in Cancel are
    // reported where they happen.
    ReportCal(message::CalSensor::kGyro, message::CalState::kCollecting, 0,
              AccelSide::kCount, 0, false);
  }
  return started;
}

bool SensorCalService::StartAccel(uint32_t now_us) {
  if (mag_.Running() || level_.Running()) {
    return false;
  }
  if (gyro_.Collecting()) {
    if (!gyro_auto_) {
      return false;
    }
    gyro_.Cancel();
  }
  return accel_.Start(now_us);
}

bool SensorCalService::StartMag(uint32_t now_us) {
  if (accel_.Collecting() || level_.Running()) {
    return false;
  }
  if (gyro_.Collecting()) {
    if (!gyro_auto_) {
      return false;
    }
    gyro_.Cancel();
  }
  return mag_.Start(now_us);
}

bool SensorCalService::StartLevel(uint32_t now_us) {
  if (accel_.Collecting() || mag_.Running()) {
    return false;
  }
  if (gyro_.Collecting()) {
    if (!gyro_auto_) {
      return false;
    }
    gyro_.Cancel();
  }
  return level_.Start(now_us);
}

uint32_t SensorCalService::MagCalibrationId() const {
  return mag_.Calibrated() ? blackboard_->GetMagnetometer().device_id : 0u;
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

void SensorCalService::Cancel() {
  if (gyro_.Collecting() && !gyro_auto_) {
    ReportCal(message::CalSensor::kGyro, message::CalState::kCancelled, 0,
              AccelSide::kCount, 0, false);
  }
  gyro_.Cancel();
  accel_.Cancel();
  mag_.Cancel();
  level_.Cancel();
}

void SensorCalService::Poll(uint32_t now_us) {
  // Reported on the edge, not the value: a run sits in kApplied or kFailed
  // until the next Start, and the operator wants one tone, not one per tick.
  const GyroCal::State before = gyro_.Status();
  const GyroCal::State after = gyro_.Poll(now_us);
  if (after != before) {
    ReportGyro(after);
  }
  ScheduleGyro(now_us);

  // The accel run reports on three edges, not one: the outcome as the gyro
  // does, every captured side, and the pose being held, because an operator
  // holding a drone needs to know a pose registered before moving to the next.
  // Two of the three are Feed's to make, so the comparison is against what was
  // last reported -- a before/after pair taken around Poll would already carry
  // Feed's work in both halves and never differ.
  const AccelCal::State accel_state = accel_.Poll(now_us);
  const uint8_t sides = accel_.SidesDone();
  const AccelSide side = accel_.CurrentSide();
  if (accel_state != reported_accel_state_ || sides != reported_accel_sides_ ||
      side != reported_accel_side_) {
    const bool captured = sides != reported_accel_sides_;
    reported_accel_state_ = accel_state;
    reported_accel_sides_ = sides;
    reported_accel_side_ = side;
    ReportAccel(accel_state, sides, side, captured);
  }

  const MagCal::State mag_state = mag_.Poll(now_us);
  const uint8_t mag_sides = mag_.SidesDone();
  const AccelSide mag_side = mag_.CurrentSide();
  const uint8_t progress = mag_.Progress();
  const bool edge = mag_state != reported_mag_state_ ||
                    mag_sides != reported_mag_sides_ ||
                    mag_side != reported_mag_side_;
  constexpr uint32_t kProgressPeriodUs = 250000;
  const bool progressed = progress != reported_mag_progress_ &&
                          (now_us - mag_progress_sent_us_) >= kProgressPeriodUs;
  if (edge || progressed) {
    const bool captured = mag_sides != reported_mag_sides_;
    const bool turn = mag_state == MagCal::State::kRotating &&
                      reported_mag_state_ != MagCal::State::kRotating;
    reported_mag_state_ = mag_state;
    reported_mag_sides_ = mag_sides;
    reported_mag_side_ = mag_side;
    reported_mag_progress_ = progress;
    mag_progress_sent_us_ = now_us;
    ReportMag(mag_state, mag_sides, mag_side, progress, captured, turn);
  }

  const LevelCal::State level_state = level_.Poll(now_us);
  if (level_state != reported_level_state_) {
    reported_level_state_ = level_state;
    ReportLevel(level_state);
  }
}

void SensorCalService::ScheduleGyro(uint32_t now_us) {
  if (gyro_.Collecting() || accel_.Collecting() || mag_.Running() ||
      level_.Running() || blackboard_->IsArmed()) {
    return;
  }
  gyro_auto_ = gyro_.Start(now_us);
}

namespace {

// Bounded for the line: a diverged fit prints as its cap, not as garbage.
long Milli(float value) {
  const float milli = value * 1000.0f;
  return milli > 9999.0f ? 9999L : milli < -9999.0f ? -9999L
                                                     : static_cast<long>(milli);
}

long Micro(float value) {
  const float micro = value * 1e6f;
  return micro > 999999999.0f ? 999999999L : static_cast<long>(micro);
}

// A positive scale as "1.010": three decimals, no float formatting.
struct Fixed3 {
  long whole;
  long frac;
};

Fixed3 ToFixed3(float value) {
  const long milli = Milli(value);
  return Fixed3{milli / 1000L, milli % 1000L};
}

// A signed angle as "-1.2": one decimal, the sign kept off the fraction.
struct Fixed1 {
  const char *sign;
  long whole;
  long frac;
};

Fixed1 ToFixed1(float value) {
  const long deci = Milli(value) / 100L;
  const long magnitude = deci < 0 ? -deci : deci;
  return Fixed1{deci < 0 ? "-" : "", magnitude / 10L, magnitude % 10L};
}

}  // namespace

void SensorCalService::ReportGyro(GyroCal::State outcome) {
  if (outcome == GyroCal::State::kFailed) {
    if (!gyro_auto_) {
      LogGyroOutcome(outcome, false);
      ReportCal(message::CalSensor::kGyro, message::CalState::kFailed, 0,
                AccelSide::kCount, 0, false);
    }
    return;
  }
  if (outcome != GyroCal::State::kApplied) {
    return;
  }
  const bool first = !gyro_session_;
  gyro_session_ = true;
  if (gyro_auto_ && !first) {
    return;
  }
  LogGyroOutcome(outcome, gyro_.Store());
  // The session's first result is sounded whoever started the run; only a
  // host's has a page to tell.
  if (gyro_auto_) {
    fclink_->SendPacket(
        message::MsgId::kTone,
        message::ToneMsg{
            .tone = static_cast<uint8_t>(message::Tone::kConfirm)});
    return;
  }
  ReportCal(message::CalSensor::kGyro, message::CalState::kApplied, 0,
            AccelSide::kCount, 100, false);
}

void SensorCalService::LogGyroOutcome(GyroCal::State outcome, bool stored) {
  if (outcome == GyroCal::State::kApplied) {
    const GyroCalibration &cal = blackboard_->GetGyroCalibration();
    fclink_->SendLog("[cal] gyro off %ld %ld %ld mdps%s",
                     static_cast<long>(cal.offsets_rad_s[0] * kRadToMdps),
                     static_cast<long>(cal.offsets_rad_s[1] * kRadToMdps),
                     static_cast<long>(cal.offsets_rad_s[2] * kRadToMdps),
                     stored ? "" : ", not stored");
    return;
  }
  switch (gyro_.Reason()) {
    case GyroCal::Failure::kNone:
      break;
    case GyroCal::Failure::kArmed:
      fclink_->SendLog("[cal] gyro failed: armed");
      break;
    case GyroCal::Failure::kBias: {
      const float *bias = gyro_.ResultRadS();
      fclink_->SendLog("[cal] gyro failed: bias %ld %ld %ld dps",
                       static_cast<long>(bias[0] / kDegToRad),
                       static_cast<long>(bias[1] / kDegToRad),
                       static_cast<long>(bias[2] / kDegToRad));
      break;
    }
    case GyroCal::Failure::kNeverStill:
      fclink_->SendLog("[cal] gyro failed: not still for %lu s",
                       static_cast<unsigned long>(
                           kSensorCalConfig.gyro.duration_s));
      break;
  }
}

void SensorCalService::ReportAccel(AccelCal::State outcome, uint8_t sides_done,
                                   AccelSide side, bool captured) {
  LogAccelOutcome(outcome);
  // Recognising a pose is silent -- it is the edge that asks the operator to
  // keep holding, and a beep there would be the same sound as the one meaning
  // "move on".
  const auto done = static_cast<uint8_t>(std::popcount(sides_done));
  ReportCal(message::CalSensor::kAccel, ToWire(outcome), sides_done, side,
            static_cast<uint8_t>(100u * done / message::kAccelSideCount),
            captured);
}

void SensorCalService::LogAccelOutcome(AccelCal::State outcome) {
  if (outcome == AccelCal::State::kApplied) {
    const AccelCalibration &cal = blackboard_->GetAccelCalibration();
    fclink_->SendLog("[cal] accel off %ld %ld %ld mm/s2",
                     Milli(cal.offsets_mps2[0]), Milli(cal.offsets_mps2[1]),
                     Milli(cal.offsets_mps2[2]));
    const Fixed3 x = ToFixed3(cal.gains[0]);
    const Fixed3 y = ToFixed3(cal.gains[1]);
    const Fixed3 z = ToFixed3(cal.gains[2]);
    fclink_->SendLog("[cal] accel scale %ld.%03ld %ld.%03ld %ld.%03ld",
                     x.whole, x.frac, y.whole, y.frac, z.whole, z.frac);
    return;
  }
  if (outcome != AccelCal::State::kFailed) {
    return;
  }
  switch (accel_.Reason()) {
    case AccelCal::Failure::kNone:
      break;
    case AccelCal::Failure::kArmed:
      fclink_->SendLog("[cal] accel failed: armed");
      break;
    case AccelCal::Failure::kTimeout:
      fclink_->SendLog("[cal] accel failed: timed out, %u of 6 poses",
                       static_cast<unsigned>(std::popcount(accel_.SidesDone())));
      break;
    case AccelCal::Failure::kGeometry:
      fclink_->SendLog("[cal] accel failed: opposite poses read alike");
      break;
    case AccelCal::Failure::kStore:
      fclink_->SendLog("[cal] accel failed: not stored");
      break;
  }
}

void SensorCalService::ReportMag(MagCal::State outcome, uint8_t sides_done,
                                 AccelSide side, uint8_t progress,
                                 bool captured, bool turn) {
  if (outcome == MagCal::State::kApplied) {
    // The id first, so a ground station that re-reads it on "done" finds the
    // calibration already there.
    fclink_->SendPacket(
        message::MsgId::kCalibrationIdConfig,
        message::CalibrationIdConfigMsg{
            .sensor = static_cast<uint8_t>(message::CalSensor::kMag),
            .id = MagCalibrationId()});
  }
  LogMagOutcome(outcome);
  // Start turning, move to the next pose: the two edges the operator acts on.
  ReportCal(message::CalSensor::kMag, ToWire(outcome), sides_done, side,
            progress, captured || turn);
}

void SensorCalService::LogMagOutcome(MagCal::State outcome) {
  if (outcome == MagCal::State::kApplied) {
    const MagFitParams &fit = mag_.Result();
    fclink_->SendLog("[cal] mag off %ld %ld %ld uT, field %ld mG",
                     static_cast<long>(fit.offset(0) * kMicroteslaPerGauss),
                     static_cast<long>(fit.offset(1) * kMicroteslaPerGauss),
                     static_cast<long>(fit.offset(2) * kMicroteslaPerGauss),
                     Milli(fit.radius));
    const Fixed3 x = ToFixed3(fit.diag(0));
    const Fixed3 y = ToFixed3(fit.diag(1));
    const Fixed3 z = ToFixed3(fit.diag(2));
    fclink_->SendLog("[cal] mag scale %ld.%03ld %ld.%03ld %ld.%03ld cost %ld e-6",
                     x.whole, x.frac, y.whole, y.frac, z.whole, z.frac,
                     Micro(mag_.ResultCost()));
    return;
  }
  if (outcome != MagCal::State::kFailed) {
    return;
  }
  const MagFit &fit = mag_.Fit();
  switch (mag_.Reason()) {
    case MagCal::Failure::kNone:
      break;
    case MagCal::Failure::kArmed:
      fclink_->SendLog("[cal] mag failed: armed");
      break;
    case MagCal::Failure::kTimeout:
      fclink_->SendLog("[cal] mag failed: run timed out");
      break;
    case MagCal::Failure::kNoTurn:
      fclink_->SendLog("[cal] mag failed: pose named but never turned on");
      break;
    case MagCal::Failure::kSphereFit:
      fclink_->SendLog("[cal] mag failed: no fit, %lu pts, %u iter",
                       static_cast<unsigned long>(mag_.Points()),
                       static_cast<unsigned>(fit.Iteration()));
      fclink_->SendLog("[cal] mag fit cost %ld e-6, field %ld mG",
                       Micro(fit.Cost()), Milli(fit.Params().radius));
      break;
    case MagCal::Failure::kBounds:
      fclink_->SendLog("[cal] mag failed: out of bounds, field %ld mG",
                       Milli(mag_.Result().radius));
      break;
    case MagCal::Failure::kStore:
      fclink_->SendLog("[cal] mag failed: not stored");
      break;
  }
}

void SensorCalService::ReportLevel(LevelCal::State outcome) {
  if (outcome == LevelCal::State::kApplied) {
    // The trim first, so a ground station that re-reads its parameters on
    // "done" finds the new one in the bridge's cache.
    fclink_->SendPacket(message::MsgId::kBoardTrimConfig, level_.Trim());
  }
  LogLevelOutcome(outcome);
  ReportCal(message::CalSensor::kLevel, ToWire(outcome), 0, AccelSide::kCount,
            outcome == LevelCal::State::kApplied ? 100 : 0, false);
}

void SensorCalService::LogLevelOutcome(LevelCal::State outcome) {
  if (outcome == LevelCal::State::kApplied) {
    const message::BoardTrimConfigMsg trim = level_.Trim();
    const Fixed1 roll = ToFixed1(trim.roll_deg);
    const Fixed1 pitch = ToFixed1(trim.pitch_deg);
    fclink_->SendLog("[cal] level trim roll %s%ld.%ld pitch %s%ld.%ld deg",
                     roll.sign, roll.whole, roll.frac, pitch.sign, pitch.whole,
                     pitch.frac);
    return;
  }
  if (outcome != LevelCal::State::kFailed) {
    return;
  }
  switch (level_.Reason()) {
    case LevelCal::Failure::kNone:
      break;
    case LevelCal::Failure::kArmed:
      fclink_->SendLog("[cal] level failed: armed");
      break;
    case LevelCal::Failure::kNoEstimate:
      fclink_->SendLog("[cal] level failed: no attitude estimate");
      break;
    case LevelCal::Failure::kMotion:
      fclink_->SendLog("[cal] level failed: motion, never still for 0.5 s");
      break;
    case LevelCal::Failure::kExcess:
      fclink_->SendLog("[cal] level failed: tilt over 45 deg, fix the mount");
      break;
    case LevelCal::Failure::kStore:
      fclink_->SendLog("[cal] level failed: not stored");
      break;
  }
}

void SensorCalService::ReportCal(message::CalSensor sensor,
                                 message::CalState state, uint8_t sides_done,
                                 AccelSide side, uint8_t progress, bool act) {
  fclink_->SendPacket(
      message::MsgId::kCalStatus,
      message::CalStatusMsg{.sensor = static_cast<uint8_t>(sensor),
                            .state = static_cast<uint8_t>(state),
                            .sides_done = sides_done,
                            .side = static_cast<uint8_t>(side),
                            .progress = progress});

  message::Tone tone = message::Tone::kBeep;
  if (state == message::CalState::kApplied) {
    tone = message::Tone::kConfirm;
  } else if (state == message::CalState::kFailed) {
    tone = message::Tone::kError;
  } else if (!act) {
    return;
  }
  fclink_->SendPacket(message::MsgId::kTone,
                      message::ToneMsg{.tone = static_cast<uint8_t>(tone)});
}
