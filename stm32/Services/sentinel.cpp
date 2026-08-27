// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "sentinel.hpp"

#include "error_code.hpp"
#include "message.hpp"
#include "panic.hpp"

void Sentinel::Init(const Config &cfg, SharedState &blackboard,
                    EscService &esc, RateController &rate_controller,
                    Icm42688p &imu, FcLink &fc_link) {
  if (initialized_) {
    Panic(ErrorCode::Stm32::kSentinelReinit);
  }

  // A zero threshold is met by an empty window, so it would raise on every
  // pass. The generator refuses one; this refuses a config that reached the
  // board another way.
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
  if (!initialized_) {
    Panic(ErrorCode::Stm32::kSentinelReinit);
  }

  // Answered before the interlock so a redundant request is not a refusal --
  // it would otherwise sound the warning for asking twice.
  if (armed == blackboard_->IsArmed()) {
    return;
  }

  // Every blocker, not just the state the request came from: a GCS must not
  // arm past a switch the pilot is holding down, and the bench path is
  // unaffected because the switch blockers only exist once RC is fresh and the
  // channel is mapped. Disarming stays available from anywhere -- a refusal
  // must never trap the board armed.
  if (armed && arm_blockers_ != 0u) {
    AnnounceArmRefusal();
    return;
  }

  // A wound-up controller from a prior session must not kick the next arm.
  rate_controller_->Reset();

  // The flag moves to its safe end of the transition first: cleared before the
  // stop frames go out, set only once the ESC path is clean. The control loop
  // can preempt anywhere in this window, and either order leaves it reading
  // disarmed -- the reverse would let it re-command motors mid-disarm.
  if (!armed) {
    // Every disarm re-arms the interlock, whatever asked for it. Doing this
    // in Supervise instead would leave a GCS or failsafe disarm to be undone
    // by the next pass, on a switch nobody moved.
    arm_switch_blocked_ = true;
    blackboard_->SetArmed(false);
    esc_->OnArmedChanged(false);
    return;
  }

  esc_->OnArmedChanged(true);
  blackboard_->SetArmed(true);
}

// Every refusal sounds the same for now: the bitmask can say which interlock
// said no, but nothing the pilot can see or hear distinguishes them yet.
void Sentinel::AnnounceArmRefusal() {
  fc_link_->SendPacket(
      message::MsgId::kTone,
      message::ToneMsg{.tone = static_cast<uint8_t>(message::Tone::kWarning)});
}

// Kept as its own function because the phase is the whole response: the guard
// that rides out a dropout, the disarm that ends it and the block on re-arming
// afterwards are one sequence, and splitting them across callers is how a
// vehicle ends up half in failsafe.
RcLinkPhase Sentinel::StepRcLink(uint32_t now_us, bool link_ok) {
  // Runs until the phase settles rather than once: a zero guard chains kUp to
  // kGuard to the disarm inside one pass, and waiting a tick between them
  // would leave the motors running past the decision. One iteration per phase
  // is the most a settling chain can need; bounded rather than while(true) so
  // a bug here cannot hang the main tick.
  for (uint8_t step = 0; step < 3u; ++step) {
    const RcLinkPhase entry = rc_link_phase_;

    switch (rc_link_phase_) {
      case RcLinkPhase::kUp:
        if (!link_ok) {
          rc_link_phase_ = RcLinkPhase::kGuard;
        }
        break;

      case RcLinkPhase::kGuard:
        // The aircraft keeps flying the pilot's last frame, and a link that
        // returns inside the window is simply resumed -- the whole reason the
        // phase exists, and why a 600 ms dropout no longer ends a flight. A
        // vehicle already disarmed skips it: there is no flight to protect.
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
        // Holds arming shut until RC has streamed cleanly for the recovery
        // period. One frame back from an intermittent link is not a link,
        // and the vehicle it would re-arm is on the ground with a switch
        // somebody never moved.
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

  // Aged off a local clock stamped when the published one moved. Comparing
  // now_us against rc.timestamp_us directly is only a staleness test while the
  // true age stays under a wrap: sit disarmed with the transmitter off for
  // 71.6 minutes and the difference comes back under the timeout, reporting a
  // link that has been dead the whole time as live.
  if (rc.timestamp_us != last_rc_stamp_) {
    last_rc_stamp_ = rc.timestamp_us;
    // Zero is the never-stamped sentinel, so the one tick that lands on it
    // borrows the next -- the same guard SuperviseTestThrottle uses.
    rc_change_us_ = (now_us == 0u) ? 1u : now_us;
    rc_ever_seen_ = true;
  }

  const bool fresh =
      rc_ever_seen_ && (now_us - rc_change_us_) < cfg_.rc_loss_timeout_us;

  // Silence before the first frame is not loss: arming over FcLink with no
  // transmitter powered on has to keep working.
  const bool link_ok = !rc_ever_seen_ || fresh;
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

  // Live every pass rather than the outcome of whichever request came last:
  // the question is "would it arm right now", and the answer moves with the
  // switch and the link, not with whoever asked previously.
  uint16_t blockers = state_blockers;
  if (in_failsafe) {
    blockers |= kArmBlockRcLoss;
  }
  if (mapped && fresh && arm_switch_blocked_) {
    blockers |= kArmBlockSwitchNotCycled;
  }
  arm_blockers_ = blockers;

  // Cleared wherever the switch is not asking, so the next time it is counts
  // as a new question. Without this a pilot who cycles the switch and is
  // refused for the same reason twice hears it only once.
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
  // The switch is up and an interlock is holding it -- the one moment on this
  // path where the pilot has asked and been told no. RequestArm never sees it,
  // because the check above is what keeps a blocked request from reaching it.
  if (blockers != switch_refusal_announced_) {
    switch_refusal_announced_ = blockers;
    AnnounceArmRefusal();
  }
}

void Sentinel::Supervise(uint32_t now_us) {
  if (!initialized_) {
    Panic(ErrorCode::Stm32::kSentinelReinit);
  }

  // Above the branch below: a throttle set on the bench survives the exit to
  // Idle, and the exit that matters is an arm request.
  SuperviseTestThrottle(now_us);

  // The condition Idle actually is, rather than the name of it: the cascade
  // is running and no flight has started. Asking the state machine which
  // state is current would point Sentinel back up at the machine that owns
  // it, and a flag the states each set is one every future state can forget
  // -- a forgotten call keeps the previous state's answer, which entered
  // from Idle is yes.
  const uint16_t state_blockers =
      (blackboard_->IsControlLoopRunning() && !blackboard_->IsArmed())
          ? 0u
          : kArmBlockNotIdle;

  // The bench states suspend the sample interrupt on purpose, so a frozen
  // heartbeat there is the state machine's doing rather than a stall -- and
  // the recovery would restart a sensor that was switched off, possibly while
  // a bit-banged four-way transfer holds the board.
  if (!blackboard_->IsControlLoopRunning()) {
    // Still set on the way out, because a request can arrive here and the RC
    // reasons below are the ones that do not apply: the receiver is switched
    // off, so its silence is not a link the pilot lost.
    arm_blockers_ = state_blockers;
    return;
  }

  // Below the branch, with the IMU checks and for the same reason those give:
  // SuspendFlightComponents stops the RC UART in the bench states, so a frozen
  // stamp there is the state machine's doing rather than a dead link. Above,
  // every ESC-config session would raise a link failsafe the GCS then holds.
  SuperviseRc(now_us, state_blockers);
  SuperviseImu(now_us);
  RecoverStalledImu(now_us);
}

// The whole bench deadman. EscService only holds the values a host asked for;
// how long that host may go quiet before they are cut is a policy question,
// and this owns it.
//
// MSP request count rather than a stamp from the parser: a configurator polls
// telemetry continuously while it sits on the motor page, so any request is
// proof the host is there -- not just the ones that move the slider.
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
    // Motors stopping on their own is indistinguishable from a wiring fault at
    // the bench, so the board says which it was. One tone, not a stream: the
    // clear above is what makes the next pass return at the top.
    fc_link_->SendPacket(message::MsgId::kTone,
                         message::ToneMsg{.tone = static_cast<uint8_t>(
                                              message::Tone::kWarning)});
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
void Sentinel::RecoverStalledImu(uint32_t now_us) {
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
  imu_->RestartSampling();
}
