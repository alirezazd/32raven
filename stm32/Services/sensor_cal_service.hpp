// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include <atomic>
#include <cstdint>

#include "ee_schema.hpp"
#include "mag_fit.hpp"
#include "message.hpp"
#include "shared_state.hpp"

class EE;
class FcLink;
class SensorCalService;
class SharedState;

// Averages the gyro's zero-rate bias out of bursts SensorCalService feeds it.
// Motion or a gap in the samples restarts the run rather than failing it, so
// only the deadline and an implausible mean end one badly -- a bump on the
// bench costs time.
//
// The result goes to the blackboard, where the estimator subtracts it, and
// the last one stored is what a boot flies on until this session has its own.
class GyroCal {
 public:
  struct Config {
    // A floor on sample count *and* elapsed sensor time: a count says nothing
    // about the window it came from, a window nothing about its density.
    uint32_t duration_s;
    uint32_t timeout_s;
    // Peak-to-peak, not magnitude, and physical: a count means nothing
    // without the full-scale range and bit depth behind it.
    uint32_t still_threshold_mdps;
    // A mean past this is motion the stillness gate could not see, or a part
    // that is broken; either way not a bias to subtract.
    uint32_t max_offset_mdps;
  };

  enum class State : uint8_t { kIdle, kCollecting, kApplied, kFailed };
  // Why a run ended in kFailed, for the line the operator reads.
  enum class Failure : uint8_t {
    kNone,
    kArmed,
    kBias,       // a mean past the bound: motion, or a part that is broken
    kNeverStill,
  };

  State Status() const { return state_; }
  bool Collecting() const {
    return collecting_.load(std::memory_order_relaxed);
  }
  Failure Reason() const { return failure_; }
  // The mean a failed run measured, for the line that says so.
  const float *ResultRadS() const { return result_rad_s_; }

  // Control tick. The caller has already validated the burst.
  void Feed(const ImuBurst &burst);
  // Slow loop, where the outcome is reported from.
  State Poll(uint32_t now_us);

 private:
  friend class SensorCalService;
  void Init(const Config &cfg, SharedState &blackboard, EE &ee);
  // False when the run was refused -- armed, or one already going.
  bool Start(uint32_t now_us);
  void Cancel();
  // Persist the last result. Which results deserve the write is the
  // service's policy, not this class's.
  bool Store();

  bool IsPlausible(const float offsets_rad_s[3]) const;
  void Publish(const float offsets_rad_s[3], GyroCalSource source);
  void ResetRun();

  Config cfg_{};
  SharedState *blackboard_ = nullptr;
  EE *ee_ = nullptr;
  float result_rad_s_[3]{};

  State state_ = State::kIdle;
  Failure failure_ = Failure::kNone;
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
// that pose, and only the last side completes it. And unlike the gyro's, the
// result is stored: an accel fit holds across boots, a zero-rate bias does not.
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
    // Peak-to-peak in milli-g, converted against the burst's own scale as
    // GyroCal does it.
    uint32_t still_threshold_mg;
  };

  enum class State : uint8_t {
    kIdle,
    kDetecting,   // waiting for the board to settle into an unvisited pose
    kCollecting,  // averaging the pose it settled into
    kApplied,
    kFailed,
    kCancelled,
  };
  enum class Failure : uint8_t {
    kNone,
    kArmed,
    kTimeout,
    kGeometry,  // opposite poses of an axis did not straddle its zero
    kStore,
  };

  State Status() const { return state_; }
  bool Collecting() const {
    return collecting_.load(std::memory_order_relaxed);
  }
  Failure Reason() const { return failure_; }
  // Which sides are already captured, one bit per AccelSide. Read by the
  // reporter to tell the operator what is left.
  uint8_t SidesDone() const {
    return sides_done_.load(std::memory_order_relaxed);
  }
  // The pose being averaged, kCount whenever none is -- which is every state
  // but kCollecting, and kCollecting the instant the board is disturbed.
  AccelSide CurrentSide() const { return side_; }

  void Feed(const ImuBurst &burst);
  State Poll(uint32_t now_us);

 private:
  friend class SensorCalService;
  void Init(const Config &cfg, SharedState &blackboard, EE &ee);
  bool Start(uint32_t now_us);
  void Cancel();

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
  Failure failure_ = Failure::kNone;
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

// Six-pose compass fit, PX4's routine: the accel names the pose the board is
// resting on, the gyro confirms the operator has started turning it, and the
// field is sampled while it turns, spread over the sphere rather than dense
// where the turn was slow. Six poses feed one sphere-then-ellipsoid fit whose
// centre is the hard iron and whose scale is the soft iron; `MagFit` is the
// fit, this is everything around it.
//
// Runs entirely on the main loop, the tick the compass is read on: the pose
// and the turn come from the estimator's calibrated burst averages, so unlike
// AccelCal there is no control-tick feed and nothing to hand across.
class MagCal {
 public:
  struct Config {
    uint32_t points_per_side;
    uint32_t side_duration_s;
    uint32_t still_duration_ms;
    // Deviation from the running mean, not the peak-to-peak spread the accel
    // run gates on: PX4's lenient detector, whose 0.75 m/s^2 is 76 mg.
    uint32_t still_threshold_mg;
    uint32_t timeout_s;
  };

  enum class State : uint8_t {
    kIdle,
    kDetecting,   // waiting for the board to rest on an unvisited pose
    kRotating,    // pose named, waiting for the operator to start turning
    kCollecting,  // sampling the field while the board turns on that pose
    kFitting,     // one fit iteration per poll, sphere then ellipsoid
    kApplied,
    kFailed,
    kCancelled,
  };

  // Why a run ended in kFailed: what the operator is told, since the page
  // only says "failed".
  enum class Failure : uint8_t {
    kNone,
    kArmed,
    kTimeout,    // the whole run's deadline
    kNoTurn,     // a pose named, never turned on
    kSphereFit,  // did not converge on the points
    kBounds,     // converged on something the Earth's field is not
    kStore,
  };

  static constexpr uint32_t kMaxPointsPerSide = 40;
  static constexpr uint32_t kMaxPoints =
      kMaxPointsPerSide * message::kAccelSideCount;
  // How long a named pose waits for the operator to start turning.
  static constexpr uint32_t kRotateTimeoutS = 35;

  State Status() const { return state_; }
  bool Running() const {
    return state_ != State::kIdle && state_ != State::kApplied &&
           state_ != State::kFailed && state_ != State::kCancelled;
  }
  uint8_t SidesDone() const { return sides_done_; }
  AccelSide CurrentSide() const { return side_; }
  // Of the whole run, 0 to 100.
  uint8_t Progress() const;
  bool Calibrated() const { return record_.calibrated != 0u; }
  // The last fit, for the report that lands with kApplied.
  const MagFitParams &Result() const { return result_; }
  float ResultCost() const { return result_cost_; }
  // For the report that lands with kFailed: the fit as it stood.
  Failure Reason() const { return failure_; }
  uint32_t Points() const { return count_; }
  const MagFit &Fit() const { return fit_; }

  State Poll(uint32_t now_us);

 private:
  friend class SensorCalService;
  void Init(const Config &cfg, SharedState &blackboard, EE &ee);
  // False when the run was refused -- armed, going, or nothing to read: in the
  // bench states the estimator is stale and a run would sit detecting until
  // its deadline.
  bool Start(uint32_t now_us);
  void Cancel();

  // Bands reject broken rather than merely poor: a scale of two or an offset
  // of two gauss is not a fit of the Earth's field.
  static bool IsPlausible(const ee_schema::MagnetometerCalibration &cal);
  void Publish();
  bool Store(const ee_schema::MagnetometerCalibration &cal);

  void BeginDetecting();
  void PollDetecting(uint32_t now_us, float dt_s);
  void PollRotating(uint32_t now_us, float dt_s);
  void PollCollecting(uint32_t now_us);
  void PollFitting();
  bool TakeSample();
  AccelSide Classify() const;
  void EndSide();
  // PX4's check on a finished fit: finite, a radius the Earth's field could
  // be, positive scale.
  static bool IsSane(const MagFitParams &p);

  Config cfg_{};
  SharedState *blackboard_ = nullptr;
  EE *ee_ = nullptr;
  ee_schema::MagnetometerCalibration record_{};

  State state_ = State::kIdle;
  Failure failure_ = Failure::kNone;
  uint32_t deadline_us_ = 0;
  // The rotate deadline while kRotating, the side deadline while kCollecting.
  uint32_t stage_deadline_us_ = 0;
  uint32_t last_poll_us_ = 0;
  uint8_t sides_done_ = 0;
  AccelSide side_ = AccelSide::kCount;

  // PX4's rest detector: a running mean and a leaky max-hold of the squared
  // deviation from it, per axis.
  float accel_ema_[3]{};
  float accel_disp_[3]{};
  uint32_t still_since_us_ = 0;
  bool still_ = false;

  float gyro_integral_[3]{};

  uint32_t last_sample_count_ = 0;
  uint32_t last_overflow_count_ = 0;
  // Seeds the fit and spaces the samples: the first accepted sample's
  // magnitude, clamped to the band a fit may land in.
  float sphere_radius_ = MagFit::kMinRadius;
  float x_[kMaxPoints]{};
  float y_[kMaxPoints]{};
  float z_[kMaxPoints]{};
  uint32_t count_ = 0;
  uint32_t side_count_ = 0;

  MagFit fit_;
  bool fitting_ellipsoid_ = false;
  MagFitParams sphere_result_{};
  MagFitParams result_{};
  float result_cost_ = 0.0f;
};


// PX4's level-horizon routine: with the airframe resting in its level flight
// attitude, the estimate's roll and pitch are the board's tilt inside the
// frame, and become the trim. Not a sensor calibration -- the sensors are
// already corrected by the time the estimate exists -- so it reads the
// estimate, not a burst, and nothing runs on the control tick.
//
// The trim is also what a ground station reads and writes as
// SENS_BOARD_{X,Y,Z}_OFF, so the stored record is owned here and both paths
// go through the same store.
class LevelCal {
 public:
  enum class State : uint8_t {
    kIdle,
    kCollecting,  // a window of attitude samples, restarted on motion
    kApplied,
    kFailed,
    kCancelled,
  };
  enum class Failure : uint8_t {
    kNone,
    kArmed,
    kNoEstimate,  // the estimator stopped moving mid-run
    kMotion,      // every window saw the frame move
    kExcess,      // a tilt the coarse mount should have taken
    kStore,
  };

  // PX4's numbers: a half-second window is still when both angles stayed
  // within half a degree of themselves, and fifty windows is the patience.
  static constexpr uint32_t kWindowUs = 500000;
  static constexpr uint32_t kMaxWindows = 50;

  State Status() const { return state_; }
  bool Running() const { return state_ == State::kCollecting; }
  Failure Reason() const { return failure_; }
  // The stored trim, in the degrees the wire carries.
  message::BoardTrimConfigMsg Trim() const;
  // A ground station's write. False when refused: out of bounds, mid-run,
  // or the store failed.
  bool SetTrim(const message::BoardTrimConfigMsg &trim);

  State Poll(uint32_t now_us);

 private:
  friend class SensorCalService;
  void Init(SharedState &blackboard, EE &ee);
  bool Start(uint32_t now_us);
  void Cancel();

  static bool IsPlausible(const ee_schema::BoardTrim &trim);
  void Publish();
  bool Store(const ee_schema::BoardTrim &trim);
  void BeginWindow(uint32_t now_us);
  void EndWindow(uint32_t now_us);

  SharedState *blackboard_ = nullptr;
  EE *ee_ = nullptr;
  ee_schema::BoardTrim record_{};

  State state_ = State::kIdle;
  Failure failure_ = Failure::kNone;
  uint32_t window_start_us_ = 0;
  uint32_t windows_ = 0;
  uint64_t last_estimate_us_ = 0;
  uint32_t last_sample_us_ = 0;
  // The board's angles with the stored trim taken back out, so a repeat run
  // measures the same tilt rather than the residual.
  float roll_sum_ = 0.0f;
  float pitch_sum_ = 0.0f;
  uint32_t count_ = 0;
  float roll_min_ = 0.0f;
  float roll_max_ = 0.0f;
  float pitch_min_ = 0.0f;
  float pitch_max_ = 0.0f;
  // The measure the run ended on, degrees, for the line that reports it.
  float result_roll_deg_ = 0.0f;
  float result_pitch_deg_ = 0.0f;
};

// Owns the calibrators, not the calibrations: it reads the burst once, hands it
// to whoever is collecting, and turns an outcome into a tone.
class SensorCalService {
 public:
  struct Config {
    GyroCal::Config gyro;
    AccelCal::Config accel;
    MagCal::Config mag;
  };

  static SensorCalService &GetInstance();

  // One run at a time, and the exclusion lives here because neither
  // calibrator can see the other. An accel session needs the airframe turned
  // between poses, and every turn trips the gyro run's stillness check: it
  // would restart until it timed out and fired a failure tone in the middle of
  // a calibration that was going fine. A run this service started on its own
  // yields to either request rather than refusing it.
  bool StartGyro(uint32_t now_us);
  bool StartAccel(uint32_t now_us);
  bool StartMag(uint32_t now_us);
  bool StartLevel(uint32_t now_us);
  // Stops whichever run is going. The accel and compass runs have a page
  // waiting on the outcome, and report the cancel so that page can close.
  void Cancel();
  // The DPS310 gets no run -- a baro's zero is a ground reference the
  // estimator re-establishes at every arm (#46).

  // What a ground station reads as CAL_MAG0_ID: the part once a calibration
  // is stored, zero before, which is how the page knows one is needed.
  uint32_t MagCalibrationId() const;
  // The board trim a ground station reads and writes.
  message::BoardTrimConfigMsg BoardTrim() const { return level_.Trim(); }
  bool SetBoardTrim(const message::BoardTrimConfigMsg &trim) {
    return level_.SetTrim(trim);
  }

  // Control tick, after the AHRS. A no-op unless a run is in progress, so the
  // caller offers every burst rather than deciding.
  void MaybeCollectBurst();
  void Poll(uint32_t now_us);

 private:
  friend class System;
  void Init(const Config &cfg, SharedState &blackboard, EE &ee, FcLink &fclink);

  SensorCalService() = default;
  ~SensorCalService() = default;
  SensorCalService(const SensorCalService &) = delete;
  SensorCalService &operator=(const SensorCalService &) = delete;

  // Starts a gyro run whenever nothing else is using the airframe: the bias
  // moves with temperature, so every landing is followed by a fresh measure
  // at the temperature the next flight starts at. The stillness gate is what
  // makes an unwatched run safe, and its deadline is not a failure here -- a
  // vehicle being carried is simply tried again.
  void ScheduleGyro(uint32_t now_us);
  void ReportGyro(GyroCal::State outcome);
  // `captured` separates the two edges a tone cannot: a pose recognised and
  // a pose averaged both leave the run detecting again.
  void ReportAccel(AccelCal::State outcome, uint8_t sides_done, AccelSide side,
                   bool captured);
  // `turn` is the edge into kRotating, the one the operator has to act on.
  void ReportMag(MagCal::State outcome, uint8_t sides_done, AccelSide side,
                 uint8_t progress, bool captured, bool turn);
  // The result or the reason, as lines the ground station's page collects:
  // each begins "[cal] " and fits a STATUSTEXT, and goes out before the
  // status that closes the page's log.
  void LogGyroOutcome(GyroCal::State outcome, bool stored);
  void LogAccelOutcome(AccelCal::State outcome);
  void LogMagOutcome(MagCal::State outcome);
  void ReportLevel(LevelCal::State outcome);
  void LogLevelOutcome(LevelCal::State outcome);
  // The status line both pose runs send, and the tone beside it: one for the
  // outcome, and a beep where `act` says the operator has to do something an
  // airframe in their hands cannot show them.
  void ReportCal(message::CalSensor sensor, message::CalState state,
                 uint8_t sides_done, AccelSide side, uint8_t progress,
                 bool act);

  GyroCal gyro_;
  AccelCal accel_;
  MagCal mag_;
  LevelCal level_;
  SharedState *blackboard_ = nullptr;
  FcLink *fclink_ = nullptr;
  bool initialized_ = false;
  // The gyro run going was ScheduleGyro's, not a host's: it yields to a host
  // and reports nothing a host did not ask for.
  bool gyro_auto_ = false;
  // Only the session's first result is stored and sounded. The board keeps
  // re-measuring while it sits, and the first is the cold one the next boot
  // most resembles.
  bool gyro_session_ = false;
  // Doubles as the novelty test: `fresh` is already cleared by the time the
  // probe runs and cannot say what is new. Shared, because one read of the slot
  // serves every calibrator.
  uint32_t last_seq_ = 0;

  // What the last accel report said. The run's interesting edges -- a pose
  // detected, a pose captured -- are made by Feed on the control tick, so a
  // before/after pair taken around Poll on this side would already carry them
  // and compare equal. Only what was last sent can say what is new.
  AccelCal::State reported_accel_state_ = AccelCal::State::kIdle;
  uint8_t reported_accel_sides_ = 0;
  AccelSide reported_accel_side_ = AccelSide::kCount;

  // The compass run is reported the same way, plus its progress: edges go out
  // at once, a progress change no more than every quarter second, because
  // each report becomes a line on the bridge's short status queue.
  MagCal::State reported_mag_state_ = MagCal::State::kIdle;
  uint8_t reported_mag_sides_ = 0;
  AccelSide reported_mag_side_ = AccelSide::kCount;
  uint8_t reported_mag_progress_ = 0;
  uint32_t mag_progress_sent_us_ = 0;

  LevelCal::State reported_level_state_ = LevelCal::State::kIdle;
};
