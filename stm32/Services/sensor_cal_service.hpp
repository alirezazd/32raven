// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include <atomic>
#include <cstdint>

#include "shared_state.hpp"

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

  // False when the run was refused -- armed, or one already going.
  bool Start(uint32_t now_us);
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

  void ResetRun();
  // Summed because any of them moving means samples went missing between two
  // the stillness check did see, and a gap can hide the very motion it exists
  // to catch.
  uint32_t SamplePathFaults() const;

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

// Owns the calibrators, not the calibrations: it reads the burst once, hands it
// to whoever is collecting, and turns an outcome into a tone. Accel and, later,
// a compass join as siblings, each with its own run.
class SensorCalService {
 public:
  struct Config {
    GyroCal::Config gyro;
  };

  static SensorCalService &GetInstance();

  GyroCal &Gyro() { return gyro_; }
  // Accel joins here off the same burst (#25); the MMC5983MA brings its own
  // feed and an ellipsoid fit (#45). The DPS310 does not -- a baro's zero is a
  // ground reference the estimator re-establishes at every arm (#46).

  // Control tick, after the AHRS. A no-op unless a run is in progress, so the
  // caller offers every burst rather than deciding.
  void MaybeCollectBurst();
  void Poll(uint32_t now_us);

 private:
  friend class System;
  void Init(const Config &cfg, SharedState &blackboard, Icm42688p &imu,
            FcLink &fclink);

  SensorCalService() = default;
  ~SensorCalService() = default;
  SensorCalService(const SensorCalService &) = delete;
  SensorCalService &operator=(const SensorCalService &) = delete;

  void Report(GyroCal::State outcome);

  GyroCal gyro_;
  SharedState *blackboard_ = nullptr;
  FcLink *fclink_ = nullptr;
  bool initialized_ = false;
  // Doubles as the novelty test: `fresh` is already cleared by the time the
  // probe runs and cannot say what is new. Shared, because one read of the slot
  // serves every calibrator.
  uint32_t last_seq_ = 0;
};
