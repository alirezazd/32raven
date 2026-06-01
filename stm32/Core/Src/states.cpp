#include "states.hpp"

#include <cmath>
#include <cstddef>

#include "multirotor_mixer.hpp"
#include "stm32_config.hpp"

namespace {

// ── Stage-2c Stabilize wiring ────────────────────────────────────────
// RC channel and threshold used to flip between kAcro and kStabilize.
// AUX1 (channel 5 on most transmitters) lives at index 4 of the raw
// channels[] array; mid-range pulse (~1500 µs) is the standard
// 3-position-switch transition. Hardcoded for 2c — moves to Kconfig
// in 2d when tuning is wanted.
constexpr std::size_t kFlightModeChannelIndex = 4u;
constexpr std::uint16_t kFlightModeThresholdUs = 1500u;
static_assert(kFlightModeChannelIndex < stm32_limits::kRcEnabledChannelCount,
              "kFlightModeChannelIndex exceeds the configured RC channel count");

// Max tilt the Stabilize mode will command from a full stick deflection
// (rad). 30° is the conservative beginner / camera default; FPV-leaning
// builds typically run 45–55°. Soft default for first activation; tune
// in 2d.
constexpr float kStabilizeMaxTiltRad = 0.5236f;  // ≈ 30°

}  // namespace

static constexpr uint32_t kLossPanicPerSec =
    Icm42688pReg::OdrHz(kIcm42688pConfig.rates.gyro) / 200u;  // 0.5%
static constexpr uint32_t kLossPanicConsecutiveSec = 3;
static constexpr uint32_t kSlowBudgetFromFastUs = 700;
static constexpr bool kEnableImuDebugLog = false;
static constexpr bool kEnableEspLogs = false;

// Debug counters (diagnostics only)
static uint32_t g_dbg_max_gps_bytes = 0;
static uint32_t g_dbg_max_slow_us = 0;
static uint32_t g_dbg_seq_gap_events = 0;
static uint32_t g_dbg_seq_gap_total = 0;

// Per-section profiling (max µs per 1Hz window)
static uint32_t g_prof_link_us = 0;    // FcLink.Poll()
static uint32_t g_prof_gps_us = 0;     // GPS byte parsing
static uint32_t g_prof_gpspub_us = 0;  // GPS data publish + SendGps
static uint32_t g_prof_step_us = 0;    // Total OnStep (may include >1 tick)
static uint32_t g_prof_fast_us = 0;    // OnFastTick max
static uint32_t g_fast_last_us = 0;
static uint32_t g_fault_led_last_toggle_us = 0;
static uint32_t g_fault_log_last_us = 0;

// --- IdleState (Main State) ---

void IdleState::OnEnter(AppContext &ctx) {
  ctx.sys->Led().Set(false);
  ctx.fast_tick_state = this;

  // Init Protocol Links
  ctx.sys->GetCommandHandler().Init();
  ctx.sys->FcLinkSvc().Init(&ctx);

  last_imu_send_us_ = 0;
  last_status_send_us_ = 0;
  slow_loop_counter_ = 0;
  g_fast_last_us = 0;
}

void IdleState::OnStep(AppContext &ctx, SmTick now) {
  // Slow path is best-effort: never let backlog starve IMU fast loop.
  uint32_t ticks = ctx.sys->Time().ConsumeTim5Ticks();
  if (ticks == 0) return;
  uint32_t step_t0 = ctx.sys->Time().Micros();
  uint32_t t0 = ctx.sys->Time().Micros();
  StepSlow(ctx, now);
  uint32_t t1 = ctx.sys->Time().Micros();
  uint32_t dt = t1 - t0;
  if (dt > g_dbg_max_slow_us) g_dbg_max_slow_us = dt;

  uint32_t step_dt = ctx.sys->Time().Micros() - step_t0;
  if (step_dt > g_prof_step_us) g_prof_step_us = step_dt;
}

void IdleState::OnFastTick(AppContext &ctx,
                           const Icm42688p::SampleBatch &batch) {
  // --- Fast Loop (per IMU burst) ---
  uint32_t fast_t0 = ctx.sys->Time().Micros();
  g_fast_last_us = fast_t0;

  auto &imu = Icm42688p::GetInstance();
  const Icm42688p::Sample &first = batch.samples[0];
  const Icm42688p::Sample &last = batch.samples[batch.count - 1u];

  static uint32_t prev_imu_seq = 0;
  if (prev_imu_seq != 0 && first.seq > prev_imu_seq) {
    const uint32_t seq_gap = first.seq - prev_imu_seq - 1u;
    if (seq_gap > 0) {
      g_dbg_seq_gap_events++;
      g_dbg_seq_gap_total += seq_gap;
      static uint64_t last_seq_gap_log_us = 0;
      if (kEnableEspLogs &&
          first.timestamp_us - last_seq_gap_log_us >= 200000) {
        ctx.sys->FcLinkSvc().SendLog(
            "IMU_SEQ_GAP gap=%lu seq=%lu miss=%lu ovr=%lu", seq_gap, first.seq,
            imu.MainMissedTicks(), imu.ImuPathOverrun());
        last_seq_gap_log_us = first.timestamp_us;
      }
    }
  }
  prev_imu_seq = last.seq;

  static uint64_t last_overrun_log_us = 0;
  uint32_t overruns = imu.ImuPathOverrun();
  if (overruns > 0) {
    if (kEnableEspLogs && last.timestamp_us - last_overrun_log_us >= 1000000) {
      ctx.sys->FcLinkSvc().SendLog("IMU_OVERRUN fault=%lu", overruns);
      last_overrun_log_us = last.timestamp_us;
    }
  }

  // Run AHRS: aggregate the IMU sample burst → averaged ω + integrated
  // quaternion → ImuState → VehicleState. Sub-step 1a: AHRS gains are
  // zero, so the quaternion is pure gyro integration (drifts) and
  // gyro_body_rad_s matches the previous inline averager bit-for-bit.
  // The rate controller only reads gyro_body_rad_s; the quaternion is
  // along for the ride until Stabilize mode lands.
  const ImuState imu_state = ctx.sys->AhrsSvc().Process(batch);
  ctx.sys->Vehicle().UpdateImu(imu_state);

  // 3. Cascade: sticks → rate_sp → rate PID → torque → mixer → DShot.
  //
  // Acro mode (rate-only — no attitude controller yet): stick deflection
  // maps linearly to a body-rate setpoint per axis. Rate controller
  // closes the loop on gyro_measured and emits the torque demand for
  // the mixer. Mixer's `armed` flag still defaults false → Mix() returns
  // all zeros until the arming sequence flips it. ESC layer enforces
  // SetArmed semantics separately as defense in depth.
  //
  // RC source: VehicleState (last update from RcReceiver, via CRSF).
  // No tx_online check yet — disarmed mixer + disarmed ESC means worst
  // case is "harmlessly compute zeros from stale RC." Add tx_online
  // gating when we add an arming sequence.
  {
    constexpr float fast_dt_sec = 0.001f;
    constexpr float max_rate_roll_pitch = 6.0f;  // rad/s, ~340 deg/s
    constexpr float max_rate_yaw = 4.0f;         // rad/s, ~230 deg/s

    const RcData &rc = ctx.sys->Vehicle().GetRc();

    // Sub-step 2c: read the mode switch and publish to VehicleState
    // before the cascade reads it. Two-position switch: AUX1 above
    // mid-pulse → kStabilize, otherwise kAcro. The published mode is
    // visible to any other consumer (telemetry, future failsafe
    // manager, etc.) — the cascade itself reads it via the same
    // VehicleState getter immediately below.
    {
      const FlightMode new_mode =
          rc.channels[kFlightModeChannelIndex] >= kFlightModeThresholdUs
              ? FlightMode::kStabilize
              : FlightMode::kAcro;
      ctx.sys->Vehicle().SetFlightMode(new_mode);
    }

    // Throttle mapping (computed up-front so the authority scaling
    // below can read it before rate_sp is finalised). Linear remap of
    // raw stick [0, 1] onto [thr_min, 1]. PX4 equivalent: MPC_MANTHR_MIN.
    // `stick` is preserved separately because the rate controller's
    // integrator-freeze threshold is compared against pilot INTENT, not
    // the post-mapping effective thrust — see CommitTorque call below.
    const float stick = RcReceiver::NormalizedThrottle(rc.throttle_us);
    const float thr_min = ctx.sys->RcRx().ThrottleMin();
    const float pilot_thrust = thr_min + (1.0f - thr_min) * stick;

    // Throttle-authority scaling: at low pilot thrust the mixer can only
    // deliver a small fraction of full torque (the band headroom near
    // `idle`). Scale the rate setpoint by this fraction so the cascade
    // asks only for what it can actually track — prevents integrator
    // wind-up + spiral when pilot whips sticks during descent.
    //
    //   authority = (pilot_thrust − idle) / (1 − idle)
    //
    // At hover/above: authority = 1, rate_sp unchanged. At pilot_thrust
    // → idle: authority → 0, rate_sp → 0 (drone honors current rate).
    // Equivalent to what PX4 gets implicitly from its position
    // controller holding thrust above idle.
    const float mixer_idle = ctx.sys->MixerSvc().GetConfig().idle;
    const float band = 1.0f - mixer_idle;
    float authority = 1.0f;
    if (band > 0.0f) {
      authority = (pilot_thrust - mixer_idle) / band;
      if (authority < 0.0f) authority = 0.0f;
      if (authority > 1.0f) authority = 1.0f;
    }

    // FlightMode gate on the setpoint source.
    //   kAcro      : sticks → angular-rate setpoint directly (Phase A).
    //   kStabilize : sticks → desired-tilt quaternion → outer attitude
    //                controller → rate setpoint for roll/pitch. Yaw
    //                stays rate-from-stick (no heading reference without
    //                a magnetometer; that lands later).
    Eigen::Vector3f rate_sp;
    if (ctx.sys->Vehicle().GetFlightMode() == FlightMode::kStabilize) {
      // Stick → desired tilt about the world-frame axes the body shares
      // when level: roll about body-X, pitch about body-Y. No yaw
      // component (yaw bypasses the attitude loop). Direct quaternion
      // construction is cheaper than going through AngleAxis and
      // produces less template bloat.
      const float roll_des_rad =
          RcReceiver::NormalizedAxis(rc.roll_us) * kStabilizeMaxTiltRad;
      const float pitch_des_rad =
          RcReceiver::NormalizedAxis(rc.pitch_us) * kStabilizeMaxTiltRad;

      const float half_roll = 0.5f * roll_des_rad;
      const float half_pitch = 0.5f * pitch_des_rad;
      const Eigen::Quaternionf q_roll(std::cos(half_roll), std::sin(half_roll),
                                      0.0f, 0.0f);
      const Eigen::Quaternionf q_pitch(std::cos(half_pitch), 0.0f,
                                       std::sin(half_pitch), 0.0f);
      // q_rp = the tilt geometry from sticks. Composition order matters
      // only at compound tilts; at small angles either order is
      // indistinguishable.
      const Eigen::Quaternionf q_rp = q_roll * q_pitch;

      // Yaw decoupling. Build q_desired so it carries the body's CURRENT
      // yaw — otherwise yaw drift (no magnetometer reference + rate-
      // bypassed yaw) bleeds into the roll/pitch error vector and the
      // cascade injects cross-axis torque whenever the body has yawed
      // from its arm-time heading. User-observed: "yaw left/right then
      // move forward → drone goes a different direction."
      //
      // Closed-form swing-twist decomposition about world-Z: any unit
      // quaternion q = (w, x, y, z) factors as q_twist · q_swing where
      // q_twist is a rotation about world-Z. The twist component has
      // the closed form (w, 0, 0, z) / sqrt(w² + z²) — no atan2, no
      // cos/sin, just one sqrt + one division. Avoids the gimbal-lock
      // edge case of ZYX Euler yaw extraction (which degenerates at
      // pitch ±90°); this form is well-defined for any attitude short
      // of fully inverted (w² + z² ≈ 0, irrelevant for Stabilize).
      const Eigen::Quaternionf &q_meas = imu_state.attitude_world_to_body;
      const float qw = q_meas.w();
      const float qz = q_meas.z();
      const float yaw_norm_sq = qw * qw + qz * qz;
      Eigen::Quaternionf q_yaw = Eigen::Quaternionf::Identity();
      if (yaw_norm_sq > 1e-12f) {
        const float inv_n = 1.0f / std::sqrt(yaw_norm_sq);
        q_yaw = Eigen::Quaternionf(qw * inv_n, 0.0f, 0.0f, qz * inv_n);
      }
      // Rotate the tilt geometry into the body's current heading: pitch-
      // forward stick now always means "pitch forward in current heading",
      // regardless of accumulated yaw drift.
      const Eigen::Quaternionf q_desired = q_yaw * q_rp;

      const Eigen::Vector3f attitude_rate_sp =
          ctx.sys->AttitudeControllerSvc().Step(
              q_desired, imu_state.attitude_world_to_body);
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

    // Apply throttle-authority scaling to rate_sp — see the authority
    // computation above. Bounds the demanded rate to what the mixer
    // can actually deliver at the pilot's current thrust.
    rate_sp *= authority;

    // 1) Compute pre-clip torque demand. PID integrators are NOT yet
    //    committed.
    const auto torque = ctx.sys->RateControllerSvc().ComputeTorque(
        rate_sp, imu_state.gyro_body_rad_s, fast_dt_sec);

    const multirotor_mixer::Inputs in{
        .roll_torque = torque[0],
        .pitch_torque = torque[1],
        .yaw_torque = torque[2],
        .thrust = pilot_thrust,
    };
    // 2) Mix → motor commands + back-projected per-axis applied torque.
    //    `mix.motors` is zero on every channel when disarmed; `mix.applied_torque`
    //    is zero in that case, and reflects the post-saturation-rescale
    //    effective body torque when armed (equals commanded in the linear
    //    region, < commanded after Betaflight motor-mix rescale).
    const auto mix = ctx.sys->MixerSvc().Mix(in);
    (void)ctx.sys->EscSvc().WriteMotorsThrust(mix.motors,
                                                     ctx.sys->Time().Micros());

    // 3) Commit integrators with the per-axis APPLIED torque. Back-calc
    //    anti-windup drains the integrator at rate Kt = Ki/Kp whenever
    //    applied ≠ commanded — covering both the disarm case (applied=0)
    //    and the armed mixer-saturation case (applied = scale·commanded).
    //    Pilot throttle is forwarded so RateController can freeze
    //    integrators when descent throttle is commanded (mixer can't
    //    apply torque there anyway).
    // Pass the RAW stick (pre-floor) to CommitTorque so the freeze
    // threshold reflects pilot intent ("commanding descent"), not the
    // post-floor effective thrust. Without this, the throttle-floor
    // remap above would push the comparand permanently above the freeze
    // threshold and the integrator would never freeze at min stick.
    ctx.sys->RateControllerSvc().CommitTorque(mix.applied_torque, fast_dt_sec,
                                              stick);
  }

  uint32_t t1 = ctx.sys->Time().Micros();

  // Capture fast-tick duration
  uint32_t fast_dt = (t1 > fast_t0) ? (t1 - fast_t0) : 0;
  if (fast_dt > g_prof_fast_us) g_prof_fast_us = fast_dt;
}

void IdleState::StepSlow(AppContext &ctx, SmTick now) {
  auto micros = [&]() -> uint32_t { return ctx.sys->Time().Micros(); };
  auto budget_exhausted = [&]() -> bool {
    if (g_fast_last_us == 0u) return false;
    return (uint32_t)(micros() - g_fast_last_us) >= kSlowBudgetFromFastUs;
  };

  // User-button → LED toggle. Polled BEFORE the budget check so the
  // debounce state machine inside Button::Poll always sees ticks
  // (otherwise budget-exhausted ticks would stretch the debounce
  // window and drop fast presses).
  {
    auto &btn = ctx.sys->Btn();
    btn.Poll(micros() / 1000u);
    if (btn.ConsumePress()) {
      ctx.sys->Led().Toggle();
    }
  }

  if (budget_exhausted()) return;
  slow_loop_counter_++;

  auto &imu = Icm42688p::GetInstance();

  if (imu.ImuPathOverrun() > 0) {
    const uint32_t current_us = micros();
    const uint32_t fault_led_period_us =
        MILLIS_TO_MICROS(kIcm42688pConfig.recovery.fault_led_period_ms);

    if ((current_us - g_fault_led_last_toggle_us) >= fault_led_period_us) {
      ctx.sys->Led().Toggle();
      g_fault_led_last_toggle_us = current_us;
    }

    if (kEnableEspLogs &&
        (current_us - g_fault_log_last_us) >= SECONDS_TO_MICROS(1)) {
      ctx.sys->FcLinkSvc().SendLog(
          "IMU_FAULT ovr=%lu dma=%lu prs=%lu hdr=%lu", imu.ImuPathOverrun(),
          imu.DmaStartFailCount(), imu.ParseFailCount(), imu.LastBadHeader());
      g_fault_log_last_us = current_us;
    }
  }

  // 1. Poll Telemetry Link (UART1 <-> ESP32)
  {
    uint32_t t0 = micros();
    ctx.sys->FcLinkSvc().Poll();
    uint32_t dt = micros() - t0;
    if (dt > g_prof_link_us) g_prof_link_us = dt;
  }
  ctx.sys->CrsfLinkSvc().PollRx(micros());
  ctx.sys->RcRx().Poll(micros());
  ctx.sys->EscSvc().Poll(micros());
  if (budget_exhausted()) return;

  // 2. Poll battery and refresh the blackboard copy.
  {
    ctx.sys->Batt().Poll(micros());
    ctx.sys->Vehicle().UpdateBattery(ctx.sys->Batt().GetData());
  }
  if (budget_exhausted()) return;

  // 3. Poll GPS (UART2 <-> M10)
  {
    uint32_t t0 = micros();
    auto &gps_uart = ctx.sys->GpsUart();
    auto &gps_svc = ctx.sys->GpsSvc();
    uint8_t c;
    uint32_t gps_bytes = 0;
    while (gps_bytes < 32 && gps_uart.Read(c)) {
      gps_svc.ProcessByte(c);
      gps_bytes++;
    }
    if (gps_bytes > g_dbg_max_gps_bytes) g_dbg_max_gps_bytes = gps_bytes;
    uint32_t dt = micros() - t0;
    if (dt > g_prof_gps_us) g_prof_gps_us = dt;
  }
  if (budget_exhausted()) return;

  // 4. Best-effort IMU telemetry (50Hz): consumes latest state only.
  {
    const Icm42688p::SampleBatch batch =
        Icm42688p::GetInstance().GetLatestBatch();
    if (batch.count > 0) {
      const Icm42688p::Sample &latest = batch.samples[batch.count - 1u];
      if (latest.timestamp_us - last_imu_send_us_ >= 20000) {
        last_imu_send_us_ = latest.timestamp_us;
        const Icm42688p::ScaledSample scaled = imu.ScaleSample(latest);
        ctx.sys->FcLinkSvc().SendImu(scaled.timestamp_us, scaled.accel_mps2,
                                     scaled.gyro_rad_s);
      }
    }
  }

  // 5. Process GPS Data (Decoupled)
  auto &gps_svc = ctx.sys->GpsSvc();
  GpsData t{};
  if (gps_svc.PopGpsData(ctx.sys->Time().Micros(), t)) {
    uint32_t t0 = micros();
    ctx.sys->Vehicle().UpdateGps(t);

    const BatteryData &bat = ctx.sys->Vehicle().GetBattery();
    ctx.sys->FcLinkSvc().SendGps(t, bat);

    uint32_t dt = micros() - t0;
    if (dt > g_prof_gpspub_us) g_prof_gpspub_us = dt;
  }

  if (budget_exhausted()) return;
  ctx.sys->CrsfLinkSvc().PollTx(micros());
  if (budget_exhausted()) return;

  // 6. Publish board/vehicle status and diagnostics at 1Hz.
  static uint32_t last_diag_print = 0;
  static uint32_t last_drop_snapshot = 0;
  static uint32_t high_loss_consec = 0;

  uint32_t current_time = micros();
  if (last_status_send_us_ == 0u ||
      current_time - last_status_send_us_ >= 1000000u) {
    last_status_send_us_ = current_time;
    ctx.sys->StatPubSvc().Publish(ctx, current_time, slow_loop_counter_);
  }

  if (current_time - last_diag_print >= 1000000) {
    last_diag_print = current_time;

    uint32_t drops_now = imu.MainMissedTicks();
    uint32_t drop_rate_per_sec = drops_now - last_drop_snapshot;
    last_drop_snapshot = drops_now;

    if (drop_rate_per_sec >= kLossPanicPerSec) {
      high_loss_consec++;
    } else {
      high_loss_consec = 0;
    }

    if (high_loss_consec == kLossPanicConsecutiveSec ||
        (high_loss_consec > kLossPanicConsecutiveSec &&
         high_loss_consec % 5 == 0)) {
      // Panic(ErrorCode::Stm32::kImuDroppedFrame);
    }

    if (kEnableEspLogs && kEnableImuDebugLog) {
      ctx.sys->FcLinkSvc().SendLog(
          "IMU_DBG irq=%lu pub=%lu miss=%lu ovr=%lu dma=%lu prs=%lu cnt=%lu "
          "sg=%lu/%lu",
          imu.IrqCount(), imu.PublishCount(), drops_now, imu.ImuPathOverrun(),
          imu.DmaStartFailCount(), imu.ParseFailCount(), imu.LastFifoCount(),
          g_dbg_seq_gap_events, g_dbg_seq_gap_total);
    }

    if (kEnableEspLogs) {
      ctx.sys->FcLinkSvc().SendLog(
          "Prof: Fast=%lu Link=%lu GPS=%lu GpsPub=%lu Step=%lu Slow=%lu",
          g_prof_fast_us, g_prof_link_us, g_prof_gps_us, g_prof_gpspub_us,
          g_prof_step_us, g_dbg_max_slow_us);
    }

    if (kEnableEspLogs) {
      ctx.sys->FcLinkSvc().SendLog(
          "Sched: GpsB=%lu Drop=%lu/s Phase=%lu Loss=%lu",
          g_dbg_max_gps_bytes, drop_rate_per_sec, 0u, high_loss_consec);
    }

    (void)drop_rate_per_sec;
    (void)drops_now;

    // Reset profiling counters
    g_dbg_max_slow_us = 0;
    g_dbg_max_gps_bytes = 0;
    g_prof_link_us = 0;
    g_prof_gps_us = 0;
    g_prof_gpspub_us = 0;
    g_prof_step_us = 0;
    g_prof_fast_us = 0;
  }
}
