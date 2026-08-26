// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "states.hpp"

#include <cmath>

#include "multirotor_mixer.hpp"
#include "stm32_config.hpp"
#include "system.hpp"

static uint32_t g_fault_led_last_toggle_us = 0;

static uint32_t g_main_tick_counter = 0;
static ControlLoopLoad g_control_loop_load{};

static void MainTick(AppContext &ctx);

static void EnterFlightLoop(AppContext &ctx, IControlTickState *state) {
  ctx.control_tick_state = state;
  ctx.sys->Blackboard().SetControlLoopRunning(true);
  ctx.sys->ResumeFlightComponents();
}

static void StepFlightLoop(AppContext &ctx) {
  if (ctx.sys->Time().ConsumeTim5Ticks() == 0u) {
    return;
  }

  MainTick(ctx);
}

static void ControlTickFlightLoop(AppContext &ctx) {
  const uint32_t tick_start_cycles = TimeBase::Cycles();
  // AHRS: aggregate IMU burst → averaged ω + integrated quaternion →
  // EstimatorState → Blackboard. Acro reads gyro_body_rad_s; Stabilize also
  // reads attitude_world_to_body.
  // Before the AHRS, which clears the mailbox's fresh flag: that flag is what
  // holds the interrupt off the slot, so the raw burst has to be taken while
  // it still stands.
  ctx.sys->LogSvc().PushRawImu();

  const EstimatorState estimate =
      ctx.sys->AhrsSvc().Process(ctx.sys->Blackboard());
  ctx.sys->Blackboard().UpdateEstimate(estimate);

  // After the AHRS on purpose: calibration has no deadline, so it reads the
  // slot on the sequence rather than holding the interrupt out of it longer.
  ctx.sys->SensorCalSvc().MaybeCollectBurst();

  // Cascade: sticks → rate_sp → rate PID → torque → mixer → DShot.
  // Mixer and ESC both read the blackboard's armed flag, which Sentinel is
  // the only writer of: Mix() returns all zeros until armed, and the ESC
  // layer checks again at the wire as defense in depth.
  // No tx_online check yet — disarmed mixer + disarmed ESC means worst
  // case is harmlessly computing zeros from stale RC.
  constexpr float fast_dt_sec = kControlLoopDtSec;
  constexpr float max_rate_roll_pitch = kPilotAcroMaxRateRollPitch;
  constexpr float max_rate_yaw = kPilotAcroMaxRateYaw;

  const RcData &rc = ctx.sys->Blackboard().GetRc();

  // Published before the cascade below reads it back.
  {
    const FlightMode new_mode =
        rc.channels_raw[kFlightModeChannelSlot] >= kFlightModeThresholdUs
            ? FlightMode::kStabilize
            : FlightMode::kAcro;
    ctx.sys->Blackboard().SetFlightMode(new_mode);
  }

  // Linear remap of raw stick [0,1] onto [thr_min,1] (PX4 MPC_MANTHR_MIN).
  // Raw `stick` kept separately: the integrator-freeze threshold compares
  // against pilot intent, not post-mapping thrust — see CommitTorque below.
  const float stick = RcReceiver::NormalizedThrottle(rc.throttle_us);
  const float thr_min = ctx.sys->RcRx().ThrottleMin();
  const float pilot_thrust = thr_min + ((1.0f - thr_min) * stick);

  // Throttle-authority scaling: at low thrust the mixer has little
  // torque headroom above `idle`, so scale rate_sp to what it can track
  // and prevent integrator wind-up + spiral on stick whip during descent.
  //   authority = (pilot_thrust − idle) / (1 − idle)
  // 1 at/above hover; → 0 as pilot_thrust → idle (rate_sp → 0).
  const float mixer_idle = ctx.sys->MixerSvc().GetConfig().idle;
  const float band = 1.0f - mixer_idle;
  float authority = 1.0f;
  if (band > 0.0f) {
    authority = (pilot_thrust - mixer_idle) / band;
    if (authority < 0.0f) authority = 0.0f;
    if (authority > 1.0f) authority = 1.0f;
  }

  // FlightMode gates the setpoint source.
  //   kAcro      : sticks → angular-rate setpoint directly.
  //   kStabilize : sticks → desired-tilt quaternion → attitude
  //                controller → roll/pitch rate setpoint. Yaw stays
  //                rate-from-stick (no heading reference without a mag).
  Eigen::Vector3f rate_sp;
  if (ctx.sys->Blackboard().GetFlightMode() == FlightMode::kStabilize) {
    // Stick → desired tilt: roll about body-X, pitch about body-Y, no
    // yaw (yaw bypasses the attitude loop). Direct quaternion build is
    // cheaper than AngleAxis and avoids template bloat.
    const float roll_des_rad =
        RcReceiver::NormalizedAxis(rc.roll_us) * kPilotStabilizeMaxTiltRad;
    const float pitch_des_rad =
        RcReceiver::NormalizedAxis(rc.pitch_us) * kPilotStabilizeMaxTiltRad;

    const float half_roll = 0.5f * roll_des_rad;
    const float half_pitch = 0.5f * pitch_des_rad;
    const Eigen::Quaternionf q_roll(std::cos(half_roll), std::sin(half_roll),
                                    0.0f, 0.0f);
    const Eigen::Quaternionf q_pitch(std::cos(half_pitch), 0.0f,
                                     std::sin(half_pitch), 0.0f);
    // Tilt geometry from sticks. Composition order matters only at
    // compound tilts; indistinguishable at small angles.
    const Eigen::Quaternionf q_rp = q_roll * q_pitch;

    // Yaw decoupling: q_desired must carry the body's CURRENT yaw, else
    // yaw drift (no mag, rate-bypassed yaw) bleeds into the roll/pitch
    // error and the cascade injects cross-axis torque after any heading
    // change.
    // Swing-twist about world-Z: unit q = (w,x,y,z) factors as
    // q_twist·q_swing with twist = (w,0,0,z)/sqrt(w²+z²) — one sqrt +
    // one div, no atan2/trig. Avoids the ZYX-Euler gimbal-lock at pitch
    // ±90°; well-defined for any attitude short of fully inverted
    // (w²+z² ≈ 0, irrelevant for Stabilize).
    const Eigen::Quaternionf &q_meas = estimate.attitude_world_to_body;
    const float qw = q_meas.w();
    const float qz = q_meas.z();
    const float yaw_norm_sq = (qw * qw) + (qz * qz);
    Eigen::Quaternionf q_yaw = Eigen::Quaternionf::Identity();
    if (yaw_norm_sq > 1e-12f) {
      const float inv_n = 1.0f / std::sqrt(yaw_norm_sq);
      q_yaw = Eigen::Quaternionf(qw * inv_n, 0.0f, 0.0f, qz * inv_n);
    }
    // Rotate tilt geometry into current heading so stick direction
    // tracks heading regardless of yaw drift.
    const Eigen::Quaternionf q_desired = q_yaw * q_rp;

    const Eigen::Vector3f attitude_rate_sp =
        ctx.sys->AttitudeControllerSvc().Step(q_desired,
                                              estimate.attitude_world_to_body);
    rate_sp = attitude_rate_sp;
    // Yaw bypasses the attitude loop — stick = desired yaw rate.
    rate_sp.z() = RcReceiver::NormalizedAxis(rc.yaw_us) * max_rate_yaw;
  } else {
    rate_sp = Eigen::Vector3f{
        RcReceiver::NormalizedAxis(rc.roll_us) * max_rate_roll_pitch,
        RcReceiver::NormalizedAxis(rc.pitch_us) * max_rate_roll_pitch,
        RcReceiver::NormalizedAxis(rc.yaw_us) * max_rate_yaw,
    };
  }

  rate_sp *= authority;  // bound demand to deliverable torque, see above

  // 1) Pre-clip torque demand; PID integrators not yet committed.
  const auto torque = ctx.sys->RateControllerSvc().ComputeTorque(
      rate_sp, estimate.gyro_body_rad_s, fast_dt_sec);

  const multirotor_mixer::Inputs in{
      .roll_torque = torque[0],
      .pitch_torque = torque[1],
      .yaw_torque = torque[2],
      .thrust = pilot_thrust,
  };
  // 2) Mix → motor commands + back-projected applied torque. Both zero
  //    when disarmed; armed, applied_torque is the post-saturation
  //    effective torque (= commanded in linear region, < commanded after
  //    Betaflight motor-mix rescale).
  const auto mix = ctx.sys->MixerSvc().Mix(in);
  (void)ctx.sys->EscSvc().WriteMotorsThrust(mix.motors,
                                            ctx.sys->Time().Micros());

  // 3) Commit integrators with APPLIED torque. Back-calc anti-windup
  //    drains at rate Kt = Ki/Kp whenever applied ≠ commanded — covers
  //    disarm (applied=0) and armed mixer saturation (applied=scale·cmd).
  //    Raw stick (not pilot_thrust) lets RateController freeze integrators
  //    on commanded descent; post-floor thrust never drops below the
  //    freeze threshold, so it would never freeze at min stick.
  ctx.sys->RateControllerSvc().CommitTorque(mix.applied_torque, fast_dt_sec,
                                            stick);

  g_control_loop_load.busy_cycles += TimeBase::Cycles() - tick_start_cycles;
  ctx.sys->Blackboard().UpdateControlLoopLoad(g_control_loop_load);
}

static void MainTick(AppContext &ctx) {
  auto micros = [&]() -> uint32_t { return ctx.sys->Time().Micros(); };
  UsbCdc::GetInstance().Poll(micros());

  // Ahead of System::Poll so a fix parsed here reaches the blackboard before
  // TelemetryPublisher reads it, rather than a pass later.
  ctx.sys->GpsSvc().Poll();

  ctx.sys->Poll(micros());

  auto &btn = ctx.sys->Btn();
  btn.Poll(micros() / 1000u);
  if (btn.ConsumePress()) {
    ctx.sys->Led().Toggle();
  }

  g_main_tick_counter++;
  ctx.sys->Blackboard().UpdateMainTickCount(g_main_tick_counter);

  if (ctx.sys->Blackboard().GetImuHealth().path_faults != 0) {
    const uint32_t current_us = micros();
    const uint32_t fault_led_period_us =
        MillisToMicros(kIcm42688pConfig.recovery.fault_led_period_ms);

    if ((current_us - g_fault_led_last_toggle_us) >= fault_led_period_us) {
      ctx.sys->Led().Toggle();
      g_fault_led_last_toggle_us = current_us;
    }
  }

  ctx.sys->CrsfLinkSvc().PollRx(micros());
  ctx.sys->EscSvc().Poll(micros());

  ctx.sys->LogSvc().Poll(micros());
  ctx.sys->SensorCalSvc().Poll(micros());

  ctx.sys->MspSvc().Poll(micros());

  ctx.sys->CrsfLinkSvc().PollCommands();
}

void IdleState::OnControlTick(AppContext &ctx) { ControlTickFlightLoop(ctx); }

void ArmedState::OnControlTick(AppContext &ctx) { ControlTickFlightLoop(ctx); }

void IdleState::OnEnter(AppContext &ctx) {
  EnterFlightLoop(ctx, this);
  // Idempotent: this entry is the disarm edge when a flight just ended.
  ctx.sys->LogSvc().StopFlight();
  ctx.sys->Led().Set(false);
}

void IdleState::OnStep(AppContext &ctx) {
  StepFlightLoop(ctx);

  // No edge to these from Armed, which is what makes the interlock
  // structural: no session can start on a vehicle whose motors are live.
  if (ctx.sys->MspSvc().EscConfigGranted()) {
    ctx.sm->ReqTransition(*ctx.esc_config_state);
    return;
  }
  if (ctx.sys->MscSvc().MscGranted()) {
    ctx.sm->ReqTransition(*ctx.msc_state);
    return;
  }

  if (ctx.sys->Blackboard().IsArmed()) {
    ctx.sm->ReqTransition(*ctx.armed_state);
  }
}

void ArmedState::OnEnter(AppContext &ctx) {
  EnterFlightLoop(ctx, this);
  ctx.sys->LogSvc().StartFlight(ctx.now_us);
  ctx.sys->Led().Set(true);
}

void ArmedState::OnStep(AppContext &ctx) {
  StepFlightLoop(ctx);

  if (!ctx.sys->Blackboard().IsArmed()) {
    ctx.sm->ReqTransition(*ctx.idle_state);
  }
}

void EscConfigState::OnEnter(AppContext &ctx) {
  // Stop the cascade in both directions. Clearing the hook stops the work --
  // ImuTick returns immediately -- and masking the interrupt stops the
  // thing that would otherwise tear a bit-banged byte apart 520 us at a time.
  ctx.control_tick_state = nullptr;
  ctx.sys->Blackboard().SetControlLoopRunning(false);
  ctx.sys->SuspendFlightComponents();
  ctx.sys->Led().Set(true);
}

void EscConfigState::OnStep(AppContext &ctx) {
  if (ctx.sys->Time().ConsumeTim5Ticks() == 0u) {
    return;
  }

  const uint32_t current_time = ctx.sys->Time().Micros();

  UsbCdc::GetInstance().Poll(current_time);
  ctx.sys->MspSvc().Poll(current_time);
  ctx.sys->Poll(current_time);

  // Gated because passthrough hands the pins to the bit-bang.
  if (!ctx.sys->FourWaySvc().IsActive()) {
    ctx.sys->EscSvc().Poll(current_time);
  }

  if (!ctx.sys->MspSvc().EscConfigGranted()) {
    ctx.sm->ReqTransition(*ctx.idle_state);
  }
}

void MscState::OnEnter(AppContext &ctx) {
  // The card changed hands in SetMscMode, before attach.
  ctx.control_tick_state = nullptr;
  ctx.sys->Blackboard().SetControlLoopRunning(false);
  ctx.sys->SuspendFlightComponents();
  ctx.sys->Led().Set(true);
}

void MscState::OnStep(AppContext &ctx) {
  if (ctx.sys->Time().ConsumeTim5Ticks() == 0u) {
    return;
  }

  const uint32_t current_time = ctx.sys->Time().Micros();

  UsbCdc::GetInstance().Poll(current_time);
  ctx.sys->MscSvc().Poll(current_time);
  ctx.sys->Poll(current_time);

  if (!ctx.sys->MscSvc().MscGranted()) {
    ctx.sm->ReqTransition(*ctx.idle_state);
  }
}
