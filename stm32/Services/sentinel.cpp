// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "sentinel.hpp"

#include "error_code.hpp"
#include "message.hpp"
#include "panic.hpp"
#include "time_base.hpp"

namespace {

// Not Kconfig: the path works or is broken. At the 1 kHz idle cadence a
// healthy window sees a handful of collisions at most.
constexpr uint32_t kEscDropWindowUs = 200000;
constexpr uint32_t kEscDropThreshold = 150;

}  // namespace

Sentinel &Sentinel::GetInstance() {
  static Sentinel instance;
  return instance;
}

void Sentinel::Init(const Config &cfg, SharedState &blackboard, EscService &esc,
                    RateController &rate_controller, Icm42688p &imu,
                    FcLink &fc_link) {
  if (initialized_) {
    Panic(ErrorCode::Stm32::kSentinelReinit);
  }

  // A zero threshold is met by an empty window and would fire every pass;
  // this catches a config that bypassed the generator's check.
  if (cfg.imu_loss_threshold_samples == 0u || cfg.imu_loss_consecutive == 0u ||
      cfg.imu_fault_threshold == 0u || cfg.imu_loss_window_us == 0u ||
      cfg.imu_fault_window_us == 0u || cfg.imu_stall_timeout_us == 0u ||
      cfg.test_throttle_silence_us == 0u) {
    Panic(ErrorCode::Stm32::kSentinelInvalidConfig);
  }

  cfg_ = cfg;
  blackboard_ = &blackboard;
  esc_ = &esc;
  rate_controller_ = &rate_controller;
  imu_ = &imu;
  fc_link_ = &fc_link;
  initialized_ = true;
}

void Sentinel::RequestArm(bool armed) {
  // Before the interlock, so asking twice does not sound a refusal.
  if (armed == blackboard_->IsArmed()) {
    return;
  }

  // A GCS must not arm past a switch the pilot holds down. Disarm is never
  // refused: a refusal must never trap the board armed.
  if (armed && arm_blockers_ != 0u) {
    EmitWarning();
    return;
  }

  // A wound-up controller from a prior session must not kick the next arm.
  rate_controller_->Reset();

  // The control loop can preempt anywhere here, so it must read disarmed
  // throughout: the flag clears first and is set last.
  if (!armed) {
    // Every disarm, whoever asked, so a GCS or failsafe disarm cannot be
    // undone on a switch nobody moved.
    arm_switch_blocked_ = true;
    blackboard_->SetArmed(false);
    // After the flag, so no control-loop write races the stop frames; an ESC
    // holds its last throttle until told otherwise.
    (void)esc_->StopAll();
    return;
  }

  blackboard_->SetArmed(true);
}

void Sentinel::EmitWarning() {
  fc_link_->SendPacket(
      message::MsgId::kTone,
      message::ToneMsg{.tone = static_cast<uint8_t>(message::Tone::kWarning)});
}

// Guard, disarm and re-arm block are one sequence kept in one place: split
// across callers, a vehicle ends up half in failsafe.
RcLinkPhase Sentinel::StepRcLink(uint32_t now_us, bool link_ok) {
  // Loops to settle, since a zero guard chains kUp to the disarm in one pass;
  // bounded at one step per phase so a bug here cannot hang the main tick.
  for (uint8_t step = 0; step < 3u; ++step) {
    const RcLinkPhase entry = rc_link_phase_;

    switch (rc_link_phase_) {
      case RcLinkPhase::kUp:
        if (!link_ok) {
          rc_link_phase_ = RcLinkPhase::kGuard;
        }
        break;

      case RcLinkPhase::kGuard:
        // A disarmed vehicle skips the guard: no flight to protect.
        if (link_ok) {
          rc_link_phase_ = RcLinkPhase::kUp;
        } else if (!blackboard_->IsArmed() ||
                   (now_us - rc_link_entered_us_) >= cfg_.failsafe_guard_us) {
          RequestArm(false);
          rc_recovery_since_us_ = 0;
          rc_link_phase_ = RcLinkPhase::kRecovering;
        }
        break;

      case RcLinkPhase::kRecovering:
        if (!link_ok) {
          rc_recovery_since_us_ = 0;
        } else if (rc_recovery_since_us_ == 0u) {
          rc_recovery_since_us_ = (now_us == 0u) ? 1u : now_us;
        } else if ((now_us - rc_recovery_since_us_) >=
                   cfg_.failsafe_recovery_us) {
          rc_link_phase_ = RcLinkPhase::kUp;
        }
        break;
    }

    if (rc_link_phase_ == entry) {
      break;
    }
    rc_link_entered_us_ = now_us;
  }

  return rc_link_phase_;
}

// Link loss first, then the switch: a request made in the same pass as the
// silence that invalidates it must not win.
void Sentinel::SuperviseRc(uint32_t now_us, uint16_t state_blockers) {
  const RcData &rc = blackboard_->GetRc();

  // Same clock as the receiver's stamp. It wraps every 71.6 min, after which
  // a dead link's stamp ages back under the timeout and reads live again.
  const bool ever_seen = rc.timestamp_us != 0u;
  const bool fresh =
      ever_seen && (now_us - rc.timestamp_us) < cfg_.rc_loss_timeout_us;

  // Silence before the first frame is not loss: arming over FcLink with no
  // transmitter powered on has to keep working.
  const bool link_ok = !ever_seen || fresh;
  const RcLinkPhase phase = StepRcLink(now_us, link_ok);

  const bool in_failsafe = phase != RcLinkPhase::kUp;
  uint32_t flags = blackboard_->FailsafeFlags();
  if (in_failsafe) {
    flags |= message::kVehicleFailsafeFlagRcLoss;
  } else {
    flags &= ~message::kVehicleFailsafeFlagRcLoss;
  }
  blackboard_->SetFailsafeFlags(flags);

  const bool mapped = cfg_.arm_rc_channel != 0u;
  const uint16_t pulse_us =
      mapped ? rc.channels_raw[cfg_.arm_rc_channel - 1u] : 0u;
  const bool in_window = mapped && fresh && pulse_us >= cfg_.arm_range_min_us &&
                         pulse_us <= cfg_.arm_range_max_us;

  if (in_window) {
    arm_switch_authority_ = true;
  } else if (mapped && fresh) {
    arm_switch_blocked_ = false;
  }

  uint16_t blockers = state_blockers;
  if (in_failsafe) {
    blockers |= kArmBlockRcLoss;
  }
  if (mapped && fresh && arm_switch_blocked_) {
    blockers |= kArmBlockSwitchNotCycled;
  }
  // Not gated on freshness: stale-high blocks, the safe direction, and
  // stale-low is caught by the link blocker. Never-seen reads zero and passes.
  if (cfg_.arm_throttle_max_us != 0u &&
      rc.throttle_us > cfg_.arm_throttle_max_us) {
    blockers |= kArmBlockThrottleHigh;
  }
  arm_blockers_ = blockers;
  blackboard_->SetArmBlocked(blockers != 0u);

  // Cleared whenever the switch is not asking, so each new ask is announced
  // even when refused for the same reason.
  if (!mapped || !fresh) {
    switch_refusal_announced_ = 0;
    return;
  }

  // Disarm before arm, and only once the switch has proved it is driven.
  if (!in_window) {
    switch_refusal_announced_ = 0;
    if (arm_switch_authority_ && blackboard_->IsArmed()) {
      RequestArm(false);
    }
    return;
  }
  if (blackboard_->IsArmed()) {
    return;
  }
  if (blockers == 0u) {
    RequestArm(true);
    return;
  }
  // A refusal spends the switch: a blocker that clears later must not arm on
  // a switch nobody moved.
  arm_switch_blocked_ = true;
  // A refusal RequestArm never sees. The latch just set is not a new reason,
  // so it alone does not sound again.
  const uint16_t others = blockers & ~kArmBlockSwitchNotCycled;
  const uint16_t reasons = (others != 0u) ? others : blockers;
  if (reasons != switch_refusal_announced_) {
    switch_refusal_announced_ = reasons;
    EmitWarning();
  }
}

void Sentinel::Supervise(uint32_t now_us) {
  // Above the branch below: a throttle set on the bench survives the exit to
  // Standby, and the exit that matters is an arm request.
  SuperviseTestThrottle(now_us);

  // Standby's condition, not its name: asking the state machine points back
  // up at Sentinel's owner, and a flag each state sets is one a state forgets.
  const uint16_t state_blockers =
      (blackboard_->IsControlLoopRunning() && !blackboard_->IsArmed())
          ? 0u
          : kArmBlockNotStandby;
  // No stored value and none measured yet: only a board's first boot, and
  // only until it has sat still for one run.
  const uint16_t gyro_blocker =
      blackboard_->GetGyroCalibration().source == GyroCalSource::kNone
          ? kArmBlockGyroUncalibrated
          : 0u;
  // Interference at rest is the site -- rebar, a parked car -- and a take-off
  // from it is where a yaw error starts.
  const uint16_t mag_blocker = blackboard_->GetEstimate().mag.interference
                                   ? kArmBlockMagInterference
                                   : 0u;
  const uint16_t standing_blockers =
      state_blockers | BatteryBlocker() | gyro_blocker | mag_blocker;

  // The bench states stop the sample interrupt on purpose, and a recovery
  // could restart the sensor mid four-way transfer.
  if (!blackboard_->IsControlLoopRunning()) {
    // Still set on the way out, since a request can arrive here; the RC reasons
    // below do not apply, the receiver being off rather than lost.
    arm_blockers_ = standing_blockers;
    blackboard_->SetArmBlocked(standing_blockers != 0u);
    // Unread here, so a switch already up must cycle once back in Standby.
    arm_switch_blocked_ = true;
    return;
  }

  // Below the branch: SuspendFlightComponents stops the RC UART, so above it
  // every ESC-config session would raise a link failsafe.
  SuperviseRc(now_us, standing_blockers);
  SuperviseEscOutput(now_us);
  SuperviseImu(now_us);
  RecoverStalledImu(now_us);
}

// A never-sampled pack reads zero and is refused. No hysteresis: a resting
// pack does not flicker across the threshold.
uint16_t Sentinel::BatteryBlocker() const {
  if (cfg_.arm_battery_min_mv == 0u) {
    return 0u;
  }
  const float pack_mv = blackboard_->GetBattery().voltage * 1000.0f;
  return pack_mv < static_cast<float>(cfg_.arm_battery_min_mv)
             ? kArmBlockLowBattery
             : 0u;
}

// Bench deadman. Any MSP request proves the host is there: a configurator
// polls telemetry continuously while it sits on the motor page.
void Sentinel::SuperviseTestThrottle(uint32_t now_us) {
  if (!esc_->TestThrottleActive()) {
    last_host_us_ = 0;
    return;
  }

  const uint32_t requests = blackboard_->GetUsbStatus().msp_requests;
  if (requests != last_msp_requests_ || last_host_us_ == 0u) {
    last_msp_requests_ = requests;
    last_host_us_ = (now_us == 0u) ? 1u : now_us;
    return;
  }

  if ((now_us - last_host_us_) >= cfg_.test_throttle_silence_us) {
    esc_->ClearTestThrottle();
    // Tells a deadman cut apart from a wiring fault. Sounds once: the clear
    // makes the next pass return at the top.
    EmitWarning();
  }
}

// Panics rather than disarms: with the burst DMA stuck no stop frame lands,
// but Panic latches TIM1's compares low and the ESCs time out on idle.
void Sentinel::SuperviseEscOutput(uint32_t now_us) {
  // Seeded on the first pass so a count carried from bring-up is not read as
  // a window's worth of failures.
  if (last_esc_window_us_ == 0u) {
    last_esc_window_us_ = (now_us == 0u) ? 1u : now_us;
    last_esc_drops_ = esc_->DroppedWriteCount();
    return;
  }

  if ((now_us - last_esc_window_us_) < kEscDropWindowUs) {
    return;
  }
  last_esc_window_us_ = (now_us == 0u) ? 1u : now_us;

  const uint32_t drops = esc_->DroppedWriteCount();
  const uint32_t delta = drops - last_esc_drops_;
  last_esc_drops_ = drops;

  if (delta >= kEscDropThreshold) {
    Panic(ErrorCode::Stm32::kEscOutputStalled);
  }
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

// Armed, a degraded gyro is still the only gyro: halting would trade a fault
// the aircraft can fly through for one it cannot.
void Sentinel::RaiseImuFault(ErrorCode::Stm32 code) {
  if (!blackboard_->IsArmed()) {
    Panic(code);
  }

  blackboard_->SetFailsafeFlags(blackboard_->FailsafeFlags() |
                                message::kVehicleFailsafeFlagImu);

  // The first cause is kept: later faults are as likely its consequences, and
  // one that clears before landing is still owed the halt.
  if (!imu_fault_latched_) {
    imu_fault_latched_ = true;
    imu_fault_code_ = code;
  }
}

// The sample interrupt stamps ImuHealth every burst, so silence past the
// timeout is a stall, not jitter.
void Sentinel::RecoverStalledImu(uint32_t now_us) {
  const auto &imu_health_us = blackboard_->GetImuHealth().timestamp_us;
  // No burst yet, so a cold start would otherwise read as a stall.
  if (imu_health_us == 0u) {
    return;
  }

  if (ElapsedMicros(now_us, imu_health_us) < cfg_.imu_stall_timeout_us) {
    return;
  }

  // Give the previous attempt a full window to show whether it took.
  if ((now_us - last_imu_recovery_us_) < cfg_.imu_stall_timeout_us) {
    return;
  }
  last_imu_recovery_us_ = now_us;
  imu_->RestartSampling();
}
