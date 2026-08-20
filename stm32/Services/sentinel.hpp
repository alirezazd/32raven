// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include <cstdint>

#include "ctx.hpp"
#include "error_code.hpp"
#include "esc_service.hpp"
#include "fc_link.hpp"
#include "icm42688p.hpp"
#include "rate_controller.hpp"
#include "shared_state.hpp"

// The board's safety authority. It owns the arm state end to end: the sources
// that ask for it, the interlocks that refuse it, the effects that make a
// transition safe, and the one variable that records the answer.
//
// Its writes are deliberately narrow -- SharedState::armed_ and nothing else.
// It never drives motors and never sequences flight modes: it decides, the
// state machine sequences, and EscService enforces at the wire. Anything it
// gains later (link-loss and sensor-staleness watchdogs) must arrive as
// another reason to call the same transition, not as another thing it writes.
class Sentinel {
 public:
  // Thresholds only. Every one is a policy question the components that raise
  // the conditions are deliberately not asked -- a driver counts, this decides
  // how much counts as too much.
  struct Config {
    // Resolved by the generator from a share of the record rate, so a rate
    // change moves it rather than silently meaning something else.
    uint32_t imu_loss_threshold_samples;
    uint32_t imu_loss_window_us;
    // Windows in a row that must exceed the threshold. 1 removes the
    // hysteresis.
    uint32_t imu_loss_consecutive;
    uint32_t imu_fault_threshold;
    uint32_t imu_fault_window_us;
    uint32_t imu_stall_timeout_us;
    // Silence from the bench host before the deadman cuts the throttle.
    uint32_t test_throttle_silence_us;
  };

  // Every arm request on the board lands here -- the privileged command today,
  // an RC switch when one exists. Returns false when an interlock refuses;
  // the caller owns whatever the refusal should sound like.
  bool RequestArm(const AppContext &ctx, bool armed);

  // Called from the main tick, never the control loop: PendSV is pended by the
  // sample interrupt, so a watchdog living there would fall silent in exactly
  // the failure it exists to catch.
  void Supervise(uint32_t now_us);

 private:
  friend class System;

  void Init(const Config &cfg, SharedState &blackboard, EscService &esc,
            RateController &rate_controller, Icm42688p &imu,
            FcLink &fc_link);

  // Weighs the driver's counters against the thresholds it deliberately does
  // not know, and answers per the arm state.
  void SuperviseImu(uint32_t now_us);
  void SuperviseTestThrottle(uint32_t now_us);
  void RecoverStalledImu(uint32_t now_us);
  // Halts when disarmed; armed, raises the failsafe flag and remembers the
  // code so the halt still happens once the aircraft is down.
  void RaiseImuFault(ErrorCode::Stm32 code);

  Config cfg_{};
  SharedState *blackboard_ = nullptr;
  EscService *esc_ = nullptr;
  RateController *rate_controller_ = nullptr;
  Icm42688p *imu_ = nullptr;
  FcLink *fc_link_ = nullptr;
  // Rate-limits recovery to one attempt per stall window, rather than every
  // main tick while the heartbeat stays frozen.
  uint32_t last_imu_recovery_us_ = 0;
  uint32_t last_loss_window_us_ = 0;
  uint32_t last_missed_samples_ = 0;
  uint32_t last_fault_window_us_ = 0;
  uint32_t last_path_faults_ = 0;
  uint32_t high_loss_consec_ = 0;
  // 0 means no throttle is standing, so a stale stamp from a previous
  // session can never cut the next one on its first pass.
  uint32_t last_host_us_ = 0;
  uint32_t last_msp_requests_ = 0;
  bool imu_fault_latched_ = false;
  // Only read while latched, so its initial value never reaches a Panic.
  ErrorCode::Stm32 imu_fault_code_{};
  bool initialized_ = false;
};
