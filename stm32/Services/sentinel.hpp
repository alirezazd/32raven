// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include <cstdint>

#include "ctx.hpp"
#include "esc_service.hpp"
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
  // Every arm request on the board lands here -- the privileged command today,
  // an RC switch when one exists. Returns false when an interlock refuses;
  // the caller owns whatever the refusal should sound like.
  bool RequestArm(const AppContext &ctx, bool armed);

  // Called from the main tick, never the control loop: PendSV is pended by the
  // sample interrupt, so a watchdog living there would fall silent in exactly
  // the failure it exists to catch.
  void Supervise(const AppContext &ctx, uint32_t now_us);

 private:
  friend class System;

  void Init(SharedState &blackboard, EscService &esc,
            RateController &rate_controller);

  SharedState *blackboard_ = nullptr;
  EscService *esc_ = nullptr;
  RateController *rate_controller_ = nullptr;
  // Rate-limits recovery to one attempt per stall window, rather than every
  // main tick while the heartbeat stays frozen.
  uint32_t last_imu_recovery_us_ = 0;
  bool initialized_ = false;
};
