// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "states.hpp"

#include <cmath>

#include "multirotor_mixer.hpp"
#include "state_machine_context.hpp"
#include "stm32_config.hpp"
#include "system.hpp"

static uint32_t g_main_tick_counter = 0;
static ControlLoopLoad g_control_loop_load{};

static void DrainFcLink(StateMachineContext &ctx) {
  FcLink &fc_link = System::GetInstance().FcLinkSvc();
  fc_link.BeginRx();
  while (auto packet =
             fc_link.PopPacket(System::GetInstance().Time().Micros())) {
    System::GetInstance().GetCommandHandler().Dispatch(ctx, *packet);
  }
  fc_link.FlushTx();
}

static void MainTick(StateMachineContext &ctx) {
  auto micros = [&]() -> uint32_t {
    return System::GetInstance().Time().Micros();
  };
  UsbCdc::GetInstance().Poll(micros());

  // Ahead of System::Poll so a fix parsed here reaches the blackboard before
  // TelemetryPublisher reads it, rather than a pass later.
  System::GetInstance().GpsSvc().Poll();

  System::GetInstance().Poll(micros());

  auto &btn = System::GetInstance().Btn();
  btn.Poll(micros() / 1000u);
  if (btn.ConsumePress()) {
    System::GetInstance().Led().Toggle();
  }

  g_main_tick_counter++;
  System::GetInstance().Blackboard().UpdateMainTickCount(g_main_tick_counter);

  System::GetInstance().CrsfLinkSvc().PollRx(micros());
  System::GetInstance().EscSvc().Poll(micros());

  System::GetInstance().LogSvc().Poll(micros());
  System::GetInstance().SensorCalSvc().Poll(micros());

  System::GetInstance().CrsfLinkSvc().PollCommands();
  // Last, so everything the pass queued goes out on it.
  DrainFcLink(ctx);
}

static void EnterFlightLoop(StateMachineContext &ctx,
                            IControlTickState *state) {
  ctx.control_tick_state = state;
  // Stamped before the loop is declared running, or the stamp left over from
  // before a bench state would report the loop dead until its first tick.
  const uint32_t now_us = System::GetInstance().Time().Micros();
  g_control_loop_load.timestamp_us = now_us;
  System::GetInstance().Blackboard().UpdateControlLoopLoad(g_control_loop_load);
  System::GetInstance().Blackboard().SetControlLoopRunning(true);
  System::GetInstance().ResumeFlightComponents();
}

static void StepFlightLoop(StateMachineContext &ctx) {
  if (System::GetInstance().Time().ConsumeTim5Ticks() == 0u) {
    return;
  }

  MainTick(ctx);
}

static void ControlTickFlightLoop() {
  const uint32_t tick_start_cycles = TimeBase::Cycles();
  // AHRS: aggregate IMU burst → averaged ω + integrated quaternion →
  // EstimatorState → Blackboard. Acro reads gyro_body_rad_s; Stabilize also
  // reads attitude_world_to_body.
  // Before the AHRS, which clears the mailbox's fresh flag: that flag is what
  // holds the interrupt off the slot, so the raw burst has to be taken while
  // it still stands.
  System::GetInstance().LogSvc().PushRawImu();

  const EstimatorState estimate = System::GetInstance().AhrsSvc().Process();
  System::GetInstance().Blackboard().UpdateEstimate(estimate);

  // After the AHRS on purpose: calibration has no deadline, so it reads the
  // slot on the sequence rather than holding the interrupt out of it longer.
  System::GetInstance().SensorCalSvc().MaybeCollectBurst();

  // Cascade: sticks → rate_sp → rate PID → torque → mixer → DShot.
  // Mixer and ESC both read the blackboard's armed flag, which Sentinel is
  // the only writer of: Mix() returns all zeros until armed, and the ESC
  // layer checks again at the wire as defense in depth.
  // No freshness check here on purpose: Sentinel owns how long stale RC may
  // keep flying, and its stage-one guard deliberately flies the pilot's last
  // frame so a dropout shorter than the guard costs nothing. Past the guard
  // it disarms, and that reaches the mixer above.
  constexpr float kFastDtSec = kControlLoopDtSec;
  constexpr float kMaxRateRollPitch = kPilotAcroMaxRateRollPitch;
  constexpr float kMaxRateYaw = kPilotAcroMaxRateYaw;

  const RcData &rc = System::GetInstance().Blackboard().GetRc();

  // Published before the cascade below reads it back.
  {
    const FlightMode new_mode =
        rc.channels_raw[kFlightModeChannelSlot] >= kFlightModeThresholdUs
            ? FlightMode::kStabilize
            : FlightMode::kAcro;
    System::GetInstance().Blackboard().SetFlightMode(new_mode);
  }

  // Linear remap of raw stick [0,1] onto [thr_min,1] (PX4 MPC_MANTHR_MIN).
  // Raw `stick` kept separately: the integrator-freeze threshold compares
  // against pilot intent, not post-mapping thrust — see CommitTorque below.
  const float stick = RcReceiver::NormalizedThrottle(rc.throttle_us);
  const float thr_min = System::GetInstance().RcRx().ThrottleMin();
  const float pilot_thrust = thr_min + ((1.0f - thr_min) * stick);

  // Throttle-authority scaling: at low thrust the mixer has little
  // torque headroom above `idle`, so scale rate_sp to what it can track
  // and prevent integrator wind-up + spiral on stick whip during descent.
  //   authority = (pilot_thrust − idle) / (1 − idle)
  // 1 at/above hover; → 0 as pilot_thrust → idle (rate_sp → 0).
  const float mixer_idle = System::GetInstance().MixerSvc().GetConfig().idle;
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
  if (System::GetInstance().Blackboard().GetFlightMode() ==
      FlightMode::kStabilize) {
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
        System::GetInstance().AttitudeControllerSvc().Step(q_desired,
                                              estimate.attitude_world_to_body);
    rate_sp = attitude_rate_sp;
    // Yaw bypasses the attitude loop — stick = desired yaw rate.
    rate_sp.z() = RcReceiver::NormalizedAxis(rc.yaw_us) * kMaxRateYaw;
  } else {
    rate_sp = Eigen::Vector3f{
        RcReceiver::NormalizedAxis(rc.roll_us) * kMaxRateRollPitch,
        RcReceiver::NormalizedAxis(rc.pitch_us) * kMaxRateRollPitch,
        RcReceiver::NormalizedAxis(rc.yaw_us) * kMaxRateYaw,
    };
  }

  rate_sp *= authority;  // bound demand to deliverable torque, see above

  // 1) Pre-clip torque demand; PID integrators not yet committed.
  const auto torque = System::GetInstance().RateControllerSvc().ComputeTorque(
      rate_sp, estimate.gyro_body_rad_s, kFastDtSec);

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
  const auto mix = System::GetInstance().MixerSvc().Mix(in);
  (void)System::GetInstance().EscSvc().WriteMotorsThrust(
      mix.motors, System::GetInstance().Time().Micros());

  // 3) Commit integrators with APPLIED torque. Back-calc anti-windup
  //    drains at rate Kt = Ki/Kp whenever applied ≠ commanded — covers
  //    disarm (applied=0) and armed mixer saturation (applied=scale·cmd).
  //    Raw stick (not pilot_thrust) lets RateController freeze integrators
  //    on commanded descent; post-floor thrust never drops below the
  //    freeze threshold, so it would never freeze at min stick.
  System::GetInstance().RateControllerSvc().CommitTorque(mix.applied_torque,
                                                         kFastDtSec, stick);

  g_control_loop_load.busy_cycles += TimeBase::Cycles() - tick_start_cycles;
  const uint32_t now_us = System::GetInstance().Time().Micros();
  g_control_loop_load.timestamp_us = now_us;
  System::GetInstance().Blackboard().UpdateControlLoopLoad(g_control_loop_load);
}

void StandbyState::OnControlTick(StateMachineContext &) {
  ControlTickFlightLoop();
}

void ArmedState::OnControlTick(StateMachineContext &) {
  ControlTickFlightLoop();
}

void StandbyState::OnEnter(StateMachineContext &ctx) {
  EnterFlightLoop(ctx, this);
  System::GetInstance().Led().Set(false);
}

void StandbyState::OnStep(StateMachineContext &ctx) {
  StepFlightLoop(ctx);

  // No edge to these from Armed, which is what makes the interlock
  // structural: no session can start on a vehicle whose motors are live.
  if (System::GetInstance().MspSvc().EscConfigGranted()) {
    ctx.sm->ReqTransition(ctx.esc_config_state);
    return;
  }
  if (System::GetInstance().MscSvc().MscGranted()) {
    ctx.sm->ReqTransition(ctx.msc_state);
    return;
  }

  if (System::GetInstance().Blackboard().IsArmed()) {
    ctx.sm->ReqTransition(ctx.armed_state);
  }
}

void ArmedState::OnEnter(StateMachineContext &ctx) {
  EnterFlightLoop(ctx, this);
  System::GetInstance().LogSvc().StartFlight(ctx.now_us);
  System::GetInstance().Led().Set(true);
}

void ArmedState::OnExit(StateMachineContext &) {
  System::GetInstance().LogSvc().StopFlight();
}

void ArmedState::OnStep(StateMachineContext &ctx) {
  StepFlightLoop(ctx);

  if (!System::GetInstance().Blackboard().IsArmed()) {
    ctx.sm->ReqTransition(ctx.standby_state);
  }
}

// The grant is revoked from FcLink, which runs after this state's last MSP
// poll, so the record that poll published still claims a port the host has
// already given up. Nothing polls MSP once this state ends, making here the
// only place the correction can be made.
void EscConfigState::OnExit(StateMachineContext &) {
  System::GetInstance().MspSvc().PublishUsbStatus(
      System::GetInstance().Time().Micros());
}

void EscConfigState::OnEnter(StateMachineContext &ctx) {
  // Stop the cascade in both directions. Clearing the hook stops the work --
  // ImuTick returns immediately -- and masking the interrupt stops the
  // thing that would otherwise tear a bit-banged byte apart 520 us at a time.
  ctx.control_tick_state = nullptr;
  System::GetInstance().Blackboard().SetControlLoopRunning(false);
  System::GetInstance().SuspendFlightComponents();
  System::GetInstance().Led().Set(true);
}

void EscConfigState::OnStep(StateMachineContext &ctx) {
  if (System::GetInstance().Time().ConsumeTim5Ticks() == 0u) {
    return;
  }

  const uint32_t current_time = System::GetInstance().Time().Micros();

  UsbCdc::GetInstance().Poll(current_time);
  System::GetInstance().MspSvc().Poll(current_time);
  System::GetInstance().Poll(current_time);
  DrainFcLink(ctx);

  // Gated because passthrough hands the pins to the bit-bang.
  if (!System::GetInstance().FourWaySvc().IsActive()) {
    System::GetInstance().EscSvc().Poll(current_time);
  }

  if (!System::GetInstance().MspSvc().EscConfigGranted()) {
    ctx.sm->ReqTransition(ctx.standby_state);
  }
}

// As EscConfigState::OnExit, for the record MscService keeps.
void MscState::OnExit(StateMachineContext &) {
  System::GetInstance().MscSvc().PublishUsbStatus(
      System::GetInstance().Time().Micros());
}

void MscState::OnEnter(StateMachineContext &ctx) {
  // The card changed hands in SetMscMode, before attach.
  ctx.control_tick_state = nullptr;
  System::GetInstance().Blackboard().SetControlLoopRunning(false);
  System::GetInstance().SuspendFlightComponents();
  System::GetInstance().Led().Set(true);
}

void MscState::OnStep(StateMachineContext &ctx) {
  if (System::GetInstance().Time().ConsumeTim5Ticks() == 0u) {
    return;
  }

  const uint32_t current_time = System::GetInstance().Time().Micros();

  UsbCdc::GetInstance().Poll(current_time);
  System::GetInstance().MscSvc().Poll(current_time);
  System::GetInstance().Poll(current_time);
  DrainFcLink(ctx);

  if (!System::GetInstance().MscSvc().MscGranted()) {
    ctx.sm->ReqTransition(ctx.standby_state);
  }
}
