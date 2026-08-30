// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include <atomic>
#include <cstdint>

#include "ee_schema.hpp"
#include "shared_state.hpp"

class EE;
class FcLink;
class Icm42688p;
class SensorCalService;
class SharedState;

// Averages the gyro's zero-rate bias out of bursts SensorCalService feeds it.
// Motion or a gap in the samples restarts the run rather than failing it, so
// only the deadline can end one badly -- a bump on the bench costs time.
class GyroCal {
 public:
  struct Config {
    // A floor on sample count *and* elapsed sensor time: a count says nothing
    // about the window it came from, a window nothing about its density.
    uint32_t duration_s;
    uint32_t timeout_s;
    // Peak-to-peak, not magnitude, and in the chip's own counts.
    uint32_t still_threshold_raw;
  };

  enum class State : uint8_t { kIdle, kCollecting, kApplied, kFailed };

  State Status() const { return state_; }
  bool Collecting() const {
    return collecting_.load(std::memory_order_relaxed);
  }

  // Control tick. The caller has already validated the burst.
  void Feed(const ImuBurst &burst);
  // Slow loop: the register write that ends a run stops the sample path for
  // ~50 ms, which no tick could afford.
  State Poll(uint32_t now_us);

 private:
  friend class SensorCalService;
  void Init(const Config &cfg, SharedState &blackboard, Icm42688p &imu);
  // False when the run was refused -- armed, or one already going.
  bool Start(uint32_t now_us);

  void ResetRun();

  Config cfg_{};
  SharedState *blackboard_ = nullptr;
  Icm42688p *imu_ = nullptr;

  State state_ = State::kIdle;
  uint32_t deadline_us_ = 0;
  uint32_t fault_mark_ = 0;
  uint32_t samples_needed_ = 0;

  // Written by Feed on the control tick, read by Poll on the slow loop;
  // `collected_` is the handoff, and neither side touches the accumulators on
  // the other's side of it. Kept in counts so the sum stays exact however long
  // the run runs -- the scale applies once, at the end.
  std::atomic<bool> collected_{false};
  std::atomic<bool> collecting_{false};
  int64_t sum_[3]{};
  int32_t min_[3]{};
  int32_t max_[3]{};
  uint32_t samples_ = 0;
  uint32_t span_us_ = 0;
  float gyro_scale_ = 0.0f;
};

// The pose set is message::AccelSide: the captured-side mask crosses the link,
// so the order is wire format and only one definition of it may exist.
using AccelSide = message::AccelSide;

// Six-pose accelerometer fit, PX4's model: per-axis offset and scale, no
// cross-axis terms. ArduPilot implements the full ellipsoid and calls the
// axis-aligned one anyway -- six axis-aligned poses do not determine nine
// parameters, so the extra ones fit noise. `ImuAccelCalibration` stores
// exactly what this produces.
//
// Unlike GyroCal this is a session rather than a capture: the run alternates
// between waiting for the board to be still in a new pose and accumulating
// that pose, and only the last side completes it.
//
// It also cannot end the way GyroCal does, by handing the driver a number that
// goes into the chip. OFFSET_USER holds an accel offset -- +-1 g at 0.5 mg, in
// the same nine bytes as the gyro's -- but the part has no gain register at
// all. A gain that must live in software needs something to hold it across
// boots and something to hand it to whoever applies it, which is what the
// EEPROM record and the blackboard field are for; and once a record exists,
// splitting the offset into silicon while the gain sits in RAM is worse than
// keeping the pair together. Every difference from GyroCal below follows from
// that one missing register rather than from the fit being harder.
class AccelCal {
 public:
  struct Config {
    // Per pose, once the board has been still long enough to classify it.
    uint32_t side_duration_ms;
    // How long "still" has to hold before a pose counts. PX4 waits 1.3 s for
    // the same reason: a board passing through an orientation on its way
    // somewhere else is briefly indistinguishable from one resting in it.
    uint32_t still_duration_ms;
    // The whole session, not one pose -- the operator is the slow part.
    uint32_t timeout_s;
    // Peak-to-peak in the chip's own counts, as GyroCal does it.
    uint32_t still_threshold_raw;
  };

  enum class State : uint8_t {
    kIdle,
    kDetecting,   // waiting for the board to settle into an unvisited pose
    kCollecting,  // averaging the pose it settled into
    kApplied,
    kFailed,
  };

  State Status() const { return state_; }
  bool Collecting() const {
    return collecting_.load(std::memory_order_relaxed);
  }
  // Which sides are already captured, one bit per AccelSide. Read by the
  // reporter to tell the operator what is left.
  uint8_t SidesDone() const {
    return sides_done_.load(std::memory_order_relaxed);
  }

  void Feed(const ImuBurst &burst);
  State Poll(uint32_t now_us);

 private:
  friend class SensorCalService;
  void Init(const Config &cfg, SharedState &blackboard, EE &ee);
  bool Start(uint32_t now_us);

  // A gain multiplies every sample the estimator will ever see, so an
  // implausible one is worse than none: a run that ended badly, or a record
  // that passed its schema check with nonsense inside it, must not reach the
  // sample path. Both bands reject broken rather than merely poor -- the
  // part's zero-g offset is +-20 mg typical and its gain within a percent or
  // two, so a whole gravity of offset is fifty times the worst a working
  // sensor produces.
  static bool IsPlausible(const ee_schema::ImuAccelCalibration &cal);
  // Puts the active fit where the estimator reads it. Called wherever the
  // record changes, which is Init and a completed run.
  void Publish();
  // Adopt a freshly solved fit, persist it and hand it to the estimator, or do
  // none of the three: a fit the EEPROM did not take is one the board would
  // lose at the next boot without ever being told.
  bool Store(const ee_schema::ImuAccelCalibration &cal);

  void ResetWindow();
  // The dominant axis has to be near a whole g and the other two near none of
  // it, or the pose is a corner and the fit would take a projection for an
  // axis. Returns kCount when the board is not in any of the six.
  AccelSide Classify(float scale) const;

  Config cfg_{};
  SharedState *blackboard_ = nullptr;
  EE *ee_ = nullptr;
  // The fit as the EEPROM holds it. Owned here rather than by the driver: the
  // driver never applies it, so passing through one would make it a courier
  // for a record it has no use for.
  ee_schema::ImuAccelCalibration record_{};

  State state_ = State::kIdle;
  uint32_t deadline_us_ = 0;
  uint32_t fault_mark_ = 0;

  std::atomic<bool> collected_{false};
  std::atomic<bool> collecting_{false};
  std::atomic<uint8_t> sides_done_{0};

  // Window state, all owned by Feed on the control tick.
  int64_t sum_[3]{};
  int32_t min_[3]{};
  int32_t max_[3]{};
  uint32_t samples_ = 0;
  uint32_t span_us_ = 0;
  uint32_t still_us_ = 0;
  AccelSide side_ = AccelSide::kCount;
  float accel_scale_ = 0.0f;

  // Only the dominant axis of each pose survives: the solve takes the
  // diagonal, so the other two components are read and dropped. PX4 keeps the
  // full vector because it builds a matrix first and then discards the
  // off-diagonal terms anyway.
  float up_mps2_[3]{};
  float down_mps2_[3]{};
};

// Owns the calibrators, not the calibrations: it reads the burst once, hands it
// to whoever is collecting, and turns an outcome into a tone. The compass joins
// as a sibling later, with its own run.
class SensorCalService {
 public:
  struct Config {
    GyroCal::Config gyro;
    AccelCal::Config accel;
  };

  static SensorCalService &GetInstance();

  // One run at a time, and the exclusion lives here because neither
  // calibrator can see the other. An accel session needs the airframe turned
  // between poses, and every turn trips the gyro run's stillness check: it
  // would restart until it timed out and fired a failure tone in the middle of
  // a calibration that was going fine.
  bool StartGyro(uint32_t now_us);
  bool StartAccel(uint32_t now_us);
  // The MMC5983MA brings its own feed and an ellipsoid fit (#45). The DPS310
  // does not -- a baro's zero is a ground reference the estimator
  // re-establishes at every arm (#46).

  // Control tick, after the AHRS. A no-op unless a run is in progress, so the
  // caller offers every burst rather than deciding.
  void MaybeCollectBurst();
  void Poll(uint32_t now_us);

 private:
  friend class System;
  void Init(const Config &cfg, SharedState &blackboard, Icm42688p &imu, EE &ee,
            FcLink &fclink);

  SensorCalService() = default;
  ~SensorCalService() = default;
  SensorCalService(const SensorCalService &) = delete;
  SensorCalService &operator=(const SensorCalService &) = delete;

  void ReportGyro(GyroCal::State outcome);
  void ReportAccel(AccelCal::State outcome, uint8_t sides_done);

  GyroCal gyro_;
  AccelCal accel_;
  SharedState *blackboard_ = nullptr;
  FcLink *fclink_ = nullptr;
  bool initialized_ = false;
  // Doubles as the novelty test: `fresh` is already cleared by the time the
  // probe runs and cannot say what is new. Shared, because one read of the slot
  // serves every calibrator.
  uint32_t last_seq_ = 0;
};
