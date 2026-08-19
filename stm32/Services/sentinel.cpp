// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "sentinel.hpp"

#include "error_code.hpp"
#include "message.hpp"
#include "panic.hpp"
#include "state_machine.hpp"
#include "system.hpp"
// IdleState has to be complete to compare against the state machine's
// current state; ctx.hpp only forward-declares it.
#include "states.hpp"  // IWYU pragma: keep

void Sentinel::Init(const Config &cfg, SharedState &blackboard,
                    EscService &esc, RateController &rate_controller) {
  if (initialized_) {
    Panic(ErrorCode::Stm32::kSentinelReinit);
  }

  // A zero threshold is met by an empty window, so it would raise on every
  // pass. The generator refuses one; this refuses a config that reached the
  // board another way.
  if (cfg.imu_loss_threshold_samples == 0u || cfg.imu_loss_consecutive == 0u ||
      cfg.imu_fault_threshold == 0u || cfg.imu_loss_window_us == 0u ||
      cfg.imu_fault_window_us == 0u || cfg.imu_stall_timeout_us == 0u) {
    Panic(ErrorCode::Stm32::kSentinelInvalidConfig);
  }

  cfg_ = cfg;
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

  // TODO(fc): detect the RC-loss, battery and GPS conditions here and publish
  // them through SetFailsafeFlags. Roadmap #15 owns which conditions count and
  // what each one does.

  SuperviseImu(now_us);
  RecoverStalledImu(ctx, now_us);
}

void Sentinel::SuperviseImu(uint32_t now_us) {
  // Tested every pass rather than on the window boundary below, so a deferred
  // fault lands on the disarm edge instead of up to a second after it.
  if (imu_fault_latched_ && !blackboard_->IsArmed()) {
    Panic(imu_fault_code_);
  }

  const ImuHealth &health = blackboard_->GetImuHealth();

  // No rate to clear: the chip only writes its sentinel when a read outran
  // the ODR, which is a configuration that cannot come right on its own.
  if (health.invalid_samples != 0u) {
    RaiseImuFault(ErrorCode::Stm32::kImuInvalidSampleDetected);
  }

  if ((now_us - last_fault_window_us_) >= cfg_.imu_fault_window_us) {
    last_fault_window_us_ = now_us;
    const uint32_t faults = health.path_faults - last_path_faults_;
    last_path_faults_ = health.path_faults;
    if (faults >= cfg_.imu_fault_threshold) {
      RaiseImuFault(ErrorCode::Stm32::kImuOverrun);
    }
  }

  if ((now_us - last_loss_window_us_) < cfg_.imu_loss_window_us) {
    return;
  }
  last_loss_window_us_ = now_us;

  const uint32_t lost = health.missed_samples - last_missed_samples_;
  last_missed_samples_ = health.missed_samples;

  if (lost < cfg_.imu_loss_threshold_samples) {
    high_loss_consec_ = 0;
    return;
  }

  high_loss_consec_++;
  if (high_loss_consec_ >= cfg_.imu_loss_consecutive) {
    RaiseImuFault(ErrorCode::Stm32::kImuDroppedFrame);
  }
}

// Armed, a degraded gyro is still the only gyro: halting answers a fault the
// aircraft can fly through with one it cannot. ArduPilot draws the same line
// from the other side, refusing to demote a sensor unless a healthy peer can
// take over -- on a single-IMU board, never.
void Sentinel::RaiseImuFault(ErrorCode::Stm32 code) {
  if (!blackboard_->IsArmed()) {
    Panic(code);
  }

  blackboard_->SetFailsafeFlags(blackboard_->FailsafeFlags() |
                                message::kVehicleFailsafeFlagImu);

  // Deferring the halt must not become dropping it. The first cause is kept
  // rather than the last: later faults are as likely to be consequences of it
  // as independent, and one that clears before landing is still owed an answer.
  if (!imu_fault_latched_) {
    imu_fault_latched_ = true;
    imu_fault_code_ = code;
  }
}

// The sample interrupt stamps ImuHealth every burst, so silence past the
// configured timeout is many missed bursts -- far outside jitter, and nothing
// else can produce it: the interrupt runs independently of the loop checking
// it. ESC configuration silences the sensor deliberately, but that state runs
// its own step body and never reaches Supervise, so it needs no exemption.
void Sentinel::RecoverStalledImu(const AppContext &ctx, uint32_t now_us) {
  const auto &imu_health_us = blackboard_->GetImuHealth().timestamp_us;
  // No burst yet, so a cold start would otherwise read as a stall.
  if (imu_health_us == 0u) {
    return;
  }

  if ((now_us - imu_health_us) < cfg_.imu_stall_timeout_us) {
    return;
  }

  // Give the previous attempt a full window to show whether it took.
  if ((now_us - last_imu_recovery_us_) < cfg_.imu_stall_timeout_us) {
    return;
  }
  last_imu_recovery_us_ = now_us;
  ctx.sys->Imu().RestartSampling();
}
