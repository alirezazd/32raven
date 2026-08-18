// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "sentinel.hpp"

#include "error_code.hpp"
#include "panic.hpp"
#include "state_machine.hpp"
#include "system.hpp"
// IdleState has to be complete to compare against the state machine's
// current state; ctx.hpp only forward-declares it.
#include "states.hpp"  // IWYU pragma: keep

namespace {

// The sample interrupt stamps ImuHealth every burst, so silence this long is
// roughly twenty missed bursts -- far outside jitter, and nothing else can
// produce it: the interrupt runs independently of the loop that checks it.
// ESC configuration silences the sensor deliberately, but that state runs its
// own step body and never reaches Supervise, so it needs no exemption here.
constexpr uint32_t kImuStallTimeoutUs = 20000u;

}  // namespace

void Sentinel::Init(SharedState &blackboard, EscService &esc,
                    RateController &rate_controller) {
  if (initialized_) {
    Panic(ErrorCode::Stm32::kSentinelReinit);
  }

  blackboard_ = &blackboard;
  esc_ = &esc;
  rate_controller_ = &rate_controller;
  initialized_ = true;
}

bool Sentinel::RequestArm(const AppContext &ctx, bool armed) {
  if (!initialized_) {
    Panic(ErrorCode::Stm32::kSentinelReinit);
  }

  // Answered before the interlock so a redundant request is not a refusal --
  // it would otherwise sound the warning for asking twice.
  if (armed == blackboard_->IsArmed()) {
    return true;
  }

  // Idle is the only state this board may arm from. Naming the state rather
  // than the reason means every future non-flyable mode inherits the refusal
  // instead of needing its own interlock; ESC configuration, which holds the
  // motor lines, is only the first. Disarming stays available from anywhere --
  // a refusal must never trap the board armed.
  if (armed && ctx.sm->CurrentState() != ctx.idle_state) {
    return false;
  }

  // A wound-up controller from a prior session must not kick the next arm.
  rate_controller_->Reset();

  // The flag moves to its safe end of the transition first: cleared before the
  // stop frames go out, set only once the ESC path is clean. The control loop
  // can preempt anywhere in this window, and either order leaves it reading
  // disarmed -- the reverse would let it re-command motors mid-disarm.
  if (!armed) {
    blackboard_->SetArmed(false);
    esc_->OnArmedChanged(false);
    return true;
  }

  esc_->OnArmedChanged(true);
  blackboard_->SetArmed(true);
  return true;
}

void Sentinel::Supervise(const AppContext &ctx, uint32_t now_us) {
  if (!initialized_) {
    Panic(ErrorCode::Stm32::kSentinelReinit);
  }

  // TODO(fc): detect the RC-loss, battery, IMU and GPS conditions here and
  // publish them through SetFailsafeFlags. Roadmap #15 owns which conditions
  // count and what each one does; #20 owns moving the two Panic sites here.

  const auto &imu_health_us = blackboard_->GetImuHealth().timestamp_us;
  // No burst yet, so a cold start would otherwise read as a stall.
  if (imu_health_us == 0u) {
    return;
  }

  if ((now_us - imu_health_us) < kImuStallTimeoutUs) {
    return;
  }

  // Give the previous attempt a full window to show whether it took.
  if ((now_us - last_imu_recovery_us_) < kImuStallTimeoutUs) {
    return;
  }
  last_imu_recovery_us_ = now_us;
  ctx.sys->Imu().RestartSampling();
}
