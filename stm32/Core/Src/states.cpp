#include "states.hpp"

#include <cmath>
#include <cstddef>

#include "multirotor_mixer.hpp"
#include "stm32_config.hpp"

namespace {

// RC channel/threshold to flip between kAcro and kStabilize.
// AUX1 (channel 5 on most TX) is index 4 of raw channels[]; mid-pulse
// (~1500 µs) is the standard 3-position-switch transition.
constexpr std::size_t kFlightModeChannelIndex = 4u;
constexpr std::uint16_t kFlightModeThresholdUs = 1500u;
static_assert(
    kFlightModeChannelIndex < stm32_limits::kRcEnabledChannelCount,
    "kFlightModeChannelIndex exceeds the configured RC channel count");

// Max tilt Stabilize commands at full stick deflection (rad). 30° is a
// conservative beginner default; FPV builds run 45–55°.
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

  // AHRS: aggregate IMU burst → averaged ω + integrated quaternion →
  // ImuState → VehicleState. Acro reads gyro_body_rad_s; Stabilize also
  // reads attitude_world_to_body.
  const ImuState imu_state = ctx.sys->AhrsSvc().Process(batch);
  ctx.sys->Vehicle().UpdateImu(imu_state);

  // Cascade: sticks → rate_sp → rate PID → torque → mixer → DShot.
  // Mixer `armed` defaults false → Mix() returns all zeros until armed;
  // ESC layer enforces SetArmed independently as defense in depth.
  // No tx_online check yet — disarmed mixer + disarmed ESC means worst
  // case is harmlessly computing zeros from stale RC.
  {
    constexpr float fast_dt_sec = 0.001f;
    constexpr float max_rate_roll_pitch = 6.0f;  // rad/s, ~340 deg/s
    constexpr float max_rate_yaw = 4.0f;         // rad/s, ~230 deg/s

    const RcData &rc = ctx.sys->Vehicle().GetRc();

    // Publish the mode switch to VehicleState before the cascade reads
    // it back below. AUX1 above mid-pulse → kStabilize, else kAcro.
    {
      const FlightMode new_mode =
          rc.channels[kFlightModeChannelIndex] >= kFlightModeThresholdUs
              ? FlightMode::kStabilize
              : FlightMode::kAcro;
      ctx.sys->Vehicle().SetFlightMode(new_mode);
    }

    // Linear remap of raw stick [0,1] onto [thr_min,1] (PX4 MPC_MANTHR_MIN).
    // Raw `stick` kept separately: the integrator-freeze threshold compares
    // against pilot intent, not post-mapping thrust — see CommitTorque below.
    const float stick = RcReceiver::NormalizedThrottle(rc.throttle_us);
    const float thr_min = ctx.sys->RcRx().ThrottleMin();
    const float pilot_thrust = thr_min + (1.0f - thr_min) * stick;

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
    if (ctx.sys->Vehicle().GetFlightMode() == FlightMode::kStabilize) {
      // Stick → desired tilt: roll about body-X, pitch about body-Y, no
      // yaw (yaw bypasses the attitude loop). Direct quaternion build is
      // cheaper than AngleAxis and avoids template bloat.
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
      // Tilt geometry from sticks. Composition order matters only at
      // compound tilts; indistinguishable at small angles.
      const Eigen::Quaternionf q_rp = q_roll * q_pitch;

      // Yaw decoupling: q_desired must carry the body's CURRENT yaw, else
      // yaw drift (no mag, rate-bypassed yaw) bleeds into the roll/pitch
      // error and the cascade injects cross-axis torque after any heading
      // change.
      //
      // Swing-twist about world-Z: unit q = (w,x,y,z) factors as
      // q_twist·q_swing with twist = (w,0,0,z)/sqrt(w²+z²) — one sqrt +
      // one div, no atan2/trig. Avoids the ZYX-Euler gimbal-lock at pitch
      // ±90°; well-defined for any attitude short of fully inverted
      // (w²+z² ≈ 0, irrelevant for Stabilize).
      const Eigen::Quaternionf &q_meas = imu_state.attitude_world_to_body;
      const float qw = q_meas.w();
      const float qz = q_meas.z();
      const float yaw_norm_sq = qw * qw + qz * qz;
      Eigen::Quaternionf q_yaw = Eigen::Quaternionf::Identity();
      if (yaw_norm_sq > 1e-12f) {
        const float inv_n = 1.0f / std::sqrt(yaw_norm_sq);
        q_yaw = Eigen::Quaternionf(qw * inv_n, 0.0f, 0.0f, qz * inv_n);
      }
      // Rotate tilt geometry into current heading so stick direction
      // tracks heading regardless of yaw drift.
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

    rate_sp *= authority;  // bound demand to deliverable torque, see above

    // 1) Pre-clip torque demand; PID integrators not yet committed.
    const auto torque = ctx.sys->RateControllerSvc().ComputeTorque(
        rate_sp, imu_state.gyro_body_rad_s, fast_dt_sec);

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
  }

  uint32_t t1 = ctx.sys->Time().Micros();

  uint32_t fast_dt = (t1 > fast_t0) ? (t1 - fast_t0) : 0;
  if (fast_dt > g_prof_fast_us) g_prof_fast_us = fast_dt;
}

void IdleState::StepSlow(AppContext &ctx, SmTick now) {
  auto micros = [&]() -> uint32_t { return ctx.sys->Time().Micros(); };
  auto budget_exhausted = [&]() -> bool {
    if (g_fast_last_us == 0u) return false;
    return (uint32_t)(micros() - g_fast_last_us) >= kSlowBudgetFromFastUs;
  };

  // Polled BEFORE the budget check so Button::Poll's debounce SM always
  // sees ticks; budget-exhausted gaps would stretch the window and drop
  // fast presses.
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
          "Sched: GpsB=%lu Drop=%lu/s Phase=%lu Loss=%lu", g_dbg_max_gps_bytes,
          drop_rate_per_sec, 0u, high_loss_consec);
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
