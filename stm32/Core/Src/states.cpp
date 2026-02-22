#include "states.hpp"
#include "board.h"
#include "icm42688p.hpp"
#include "panic.hpp"
#include "spi.hpp"
#include "user_config.hpp"
#include <cstdio>
#include <cstdlib>
#include <cstring>

static constexpr float kPi = 3.14159265358979f;
static constexpr float kG = 9.80665f;

static constexpr float AccelRangeG(Icm42688pReg::AccelFs fs) {
  switch (fs) {
  case Icm42688pReg::AccelFs::k16g:
    return 16.0f;
  case Icm42688pReg::AccelFs::k8g:
    return 8.0f;
  case Icm42688pReg::AccelFs::k4g:
    return 4.0f;
  case Icm42688pReg::AccelFs::k2g:
    return 2.0f;
  default:
    return 16.0f;
  }
}

static constexpr float GyroRangeDps(Icm42688pReg::GyroFs fs) {
  switch (fs) {
  case Icm42688pReg::GyroFs::k2000dps:
    return 2000.0f;
  case Icm42688pReg::GyroFs::k1000dps:
    return 1000.0f;
  case Icm42688pReg::GyroFs::k500dps:
    return 500.0f;
  case Icm42688pReg::GyroFs::k250dps:
    return 250.0f;
  case Icm42688pReg::GyroFs::k125dps:
    return 125.0f;
  case Icm42688pReg::GyroFs::k62_5dps:
    return 62.5f;
  case Icm42688pReg::GyroFs::k31_25dps:
    return 31.25f;
  case Icm42688pReg::GyroFs::k15_625dps:
    return 15.625f;
  default:
    return 2000.0f;
  }
}

// Scale factors from configured full-scale ranges (16-bit FIFO samples).
static constexpr float kAccelScale =
    (AccelRangeG(kIcm42688pConfig.fs.accel) / 32768.0f) * kG;
// deg/s to rad/s
static constexpr float kGyroScale =
    (GyroRangeDps(kIcm42688pConfig.fs.gyro) / 32768.0f) * (kPi / 180.0f);

static constexpr uint32_t kImuPeriodUs = 1000; // 1kHz
static constexpr uint32_t kImuDtTolUs = 200;   // phase-lock tolerance
static constexpr uint32_t kPhaseBadConsecutiveLimit = 16;
static constexpr uint32_t kLossWarnPerSec = 1;  // 0.1% at 1kHz
static constexpr uint32_t kLossPanicPerSec = 5; // 0.5% at 1kHz
static constexpr uint32_t kLossPanicConsecutiveSec = 3;

// Debug counters (diagnostics only)
static uint32_t g_dbg_max_gps_bytes = 0;
static uint32_t g_dbg_max_slow_us = 0;
static int32_t g_dbg_last_ax_mms2 = 0;
static int32_t g_dbg_last_ay_mms2 = 0;
static int32_t g_dbg_last_az_mms2 = 0;
static int32_t g_dbg_last_gx_mrads = 0;
static int32_t g_dbg_last_gy_mrads = 0;
static int32_t g_dbg_last_gz_mrads = 0;

// Per-section profiling (max µs per 1Hz window)
static uint32_t g_prof_link_us = 0;   // FcLink.Poll()
static uint32_t g_prof_gps_us = 0;    // GPS byte parsing
static uint32_t g_prof_gpspub_us = 0; // GPS data publish + SendGps
static uint32_t g_prof_tsync_us = 0;  // TimeSync send
static uint32_t g_prof_step_us = 0;   // Total OnStep (may include >1 tick)
static uint32_t g_prof_fast_us = 0;   // OnFastTick max

// --- IdleState (Main State) ---

void IdleState::OnEnter(AppContext &ctx, SmTick now) {
  (void)now;
  ctx.sys->Led().Set(false);

  // Init Protocol Links
  ctx.sys->GetCommandHandler().Init();
  ctx.sys->GetFcLink().Init(&ctx);

  last_slow_us_ = 0;
}

void IdleState::OnStep(AppContext &ctx, SmTick now) {
  // Run slow tasks on TIM5 tick(s), decoupled from IMU fast path.
  uint32_t ticks = ctx.sys->Time().ConsumeTim5Ticks();
  if (ticks == 0)
    return;
  uint32_t step_t0 = (uint32_t)ctx.sys->Time().MicrosCorrected();
  while (ticks--) {
    uint32_t t0 = (uint32_t)ctx.sys->Time().MicrosCorrected();
    StepSlow(ctx, now);
    uint32_t t1 = (uint32_t)ctx.sys->Time().MicrosCorrected();
    uint32_t dt = t1 - t0;
    if (dt > g_dbg_max_slow_us)
      g_dbg_max_slow_us = dt;
  }
  uint32_t step_dt = (uint32_t)ctx.sys->Time().MicrosCorrected() - step_t0;
  if (step_dt > g_prof_step_us)
    g_prof_step_us = step_dt;
}

void IdleState::OnFastTick(AppContext &ctx, const Icm42688p::Sample &raw) {
  // --- Fast Loop (1kHz) ---
  uint32_t fast_t0 = (uint32_t)ctx.sys->Time().MicrosCorrected();

  static uint32_t prev_seq = 0;
  static uint64_t prev_raw_ts = 0;
  static uint32_t phase_bad_consec = 0;
  auto &imu = Icm42688p::GetInstance();

  uint32_t seq_gap = 0;
  if (prev_seq != 0 && raw.seq > prev_seq) {
    seq_gap = raw.seq - prev_seq - 1;
    if (seq_gap > max_seq_gap_)
      max_seq_gap_ = seq_gap;
  }
  prev_seq = raw.seq;

  if (prev_raw_ts != 0 && raw.timestamp_us > prev_raw_ts) {
    uint32_t dt = (uint32_t)(raw.timestamp_us - prev_raw_ts);
    if (dt > max_raw_dt_us_)
      max_raw_dt_us_ = dt;

    // Hard phase-lock guard (after warmup): require sustained violations.
    uint32_t lo = kImuPeriodUs - kImuDtTolUs;
    uint32_t hi = kImuPeriodUs + kImuDtTolUs;
    if (dt < lo || dt > hi) {
      phase_bad_consec++;
      if (phase_bad_consec > max_phase_bad_consec_)
        max_phase_bad_consec_ = phase_bad_consec;

      if (phase_bad_consec == kPhaseBadConsecutiveLimit ||
          (phase_bad_consec % 1000 == 0)) {
        ctx.sys->GetFcLink().SendLog(
            "IMU_PHASE_LOST: dt=%lu expected=%lu+-%lu bad=%lu seq=%lu", dt,
            kImuPeriodUs, kImuDtTolUs, phase_bad_consec, raw.seq);
        // Panic(ErrorCode::kImuDroppedFrame);
      }
    } else {
      phase_bad_consec = 0;
    }
  }
  prev_raw_ts = raw.timestamp_us;

  static uint64_t last_overrun_log_us = 0;
  uint32_t overruns = imu.ImuPathOverrun();
  if (overruns > 0) {
    if (raw.timestamp_us - last_overrun_log_us >= 1000000) {
      ctx.sys->GetFcLink().SendLog("PANIC [0x00000035]: IMU Path Overrun (%lu)",
                                   overruns);
      last_overrun_log_us = raw.timestamp_us;
    }
  }

  // 1. Convert & Scale
  ImuData d{};
  d.timestamp_us = raw.timestamp_us;
  d.accel[0] = (float)raw.accel[0] * kAccelScale;
  d.accel[1] = (float)raw.accel[1] * kAccelScale;
  d.accel[2] = (float)raw.accel[2] * kAccelScale;
  d.gyro[0] = (float)raw.gyro[0] * kGyroScale;
  d.gyro[1] = (float)raw.gyro[1] * kGyroScale;
  d.gyro[2] = (float)raw.gyro[2] * kGyroScale;
  d.valid = true;
  g_dbg_last_ax_mms2 = (int32_t)(d.accel[0] * 1000.0f);
  g_dbg_last_ay_mms2 = (int32_t)(d.accel[1] * 1000.0f);
  g_dbg_last_az_mms2 = (int32_t)(d.accel[2] * 1000.0f);
  g_dbg_last_gx_mrads = (int32_t)(d.gyro[0] * 1000.0f);
  g_dbg_last_gy_mrads = (int32_t)(d.gyro[1] * 1000.0f);
  g_dbg_last_gz_mrads = (int32_t)(d.gyro[2] * 1000.0f);

  // 2. Update Blackboard (Attitude Estimator would go here)
  ctx.sys->GetVehicleState().UpdateImu(d);

  // 3. Controller / Mixer / DShot (future)

  // 4. Send High-Rate Telemetry (50Hz)
  if (d.timestamp_us - last_imu_send_us_ >= 20000) {
    uint32_t t0 = ctx.sys->Time().MicrosCorrected();
    last_imu_send_us_ = d.timestamp_us;
    ctx.sys->GetFcLink().SendImu(d);
    uint32_t t1 = ctx.sys->Time().MicrosCorrected();
    if (t1 - t0 > max_send_us_)
      max_send_us_ = t1 - t0;
  }

  uint32_t t1 = (uint32_t)ctx.sys->Time().MicrosCorrected();

  // Capture fast-tick duration
  uint32_t fast_dt = (t1 > fast_t0) ? (t1 - fast_t0) : 0;
  if (fast_dt > g_prof_fast_us)
    g_prof_fast_us = fast_dt;
}

void IdleState::StepSlow(AppContext &ctx, SmTick now) {
  auto micros = [&]() -> uint32_t {
    return (uint32_t)ctx.sys->Time().MicrosCorrected();
  };

  // 1. Poll Telemetry Link (UART1 <-> ESP32)
  {
    uint32_t t0 = micros();
    ctx.sys->GetFcLink().Poll();
    uint32_t dt = micros() - t0;
    if (dt > g_prof_link_us)
      g_prof_link_us = dt;
  }

  // 2. Poll GPS (UART2 <-> M9N)
  {
    uint32_t t0 = micros();
    auto &gps_uart = ctx.sys->GetUart2();
    auto &gps_svc = ctx.sys->ServiceM9N();
    uint8_t c;
    uint32_t gps_bytes = 0;
    while (gps_uart.Read(c)) {
      gps_svc.ProcessByte(c);
      gps_bytes++;
    }
    if (gps_bytes > g_dbg_max_gps_bytes)
      g_dbg_max_gps_bytes = gps_bytes;
    uint32_t dt = micros() - t0;
    if (dt > g_prof_gps_us)
      g_prof_gps_us = dt;
  }

  // 3. Process GPS Data (Decoupled)
  auto &gps_svc = ctx.sys->ServiceM9N();
  if (gps_svc.NewDataAvailable()) {
    uint32_t t0 = micros();
    const auto &pvt = gps_svc.GetData();
    const auto &cov = gps_svc.GetCOV();
    // Note: DOP message disabled (redundant with COV)

    GpsData t;
    t.timestamp_us = ctx.sys->Time().MicrosCorrected();
    t.lat = pvt.lat;
    t.lon = pvt.lon;
    t.alt = pvt.hMSL;
    t.vel = (uint16_t)(pvt.gSpeed / 10);    // mm/s -> cm/s
    t.hdg = (uint16_t)(pvt.headMot / 1000); // 1e-5 deg -> cdeg
    t.num_sats = pvt.numSV;
    t.fix_type = pvt.fixType;
    t.year = pvt.year;
    t.month = pvt.month;
    t.day = pvt.day;
    t.hour = pvt.hour;
    t.min = pvt.min;
    t.sec = pvt.sec;
    t.hAcc = pvt.hAcc;
    t.vAcc = pvt.vAcc;

    // Covariance matrix (essential for EKF sensor fusion)
    t.posCovValid = cov.posCovValid;
    t.velCovValid = cov.velCovValid;
    t.posCovNN = cov.posCovNN;
    t.posCovEE = cov.posCovEE;
    t.posCovDD = cov.posCovDD;

    // Note: DOP fields omitted (redundant with covariance + hAcc/vAcc)

    // Update Blackboard
    ctx.sys->GetVehicleState().UpdateGps(t);

    // Forward to ESP32
    BatteryData bat = {0};
    ctx.sys->GetFcLink().SendGps(t, bat);

    gps_svc.ClearNewDataFlag();
    uint32_t dt = micros() - t0;
    if (dt > g_prof_gpspub_us)
      g_prof_gpspub_us = dt;
  }

  // 4. Send TimeSync (1Hz)
  if (now - last_time_sync_ms_ >= 1000000) { // now is micros!
    uint32_t t0 = micros();
    last_time_sync_ms_ = now;

    message::TimeSyncMsg msg;
    msg.timestamp = ctx.sys->Time().MicrosCorrected();
    msg.drift_micros = (int32_t)ctx.sys->Time().GetDriftMicros();
    msg.synced = ctx.sys->Time().IsGpsSynchronized() ? 1 : 0;

    ctx.sys->GetFcLink().SendTimeSync(msg);
    uint32_t dt = micros() - t0;
    if (dt > g_prof_tsync_us)
      g_prof_tsync_us = dt;
  }

  // 5. Print Diagnostics (1Hz) - moved from OnFastTick to avoid blocking IMU
  static uint32_t last_diag_print = 0;
  static uint32_t last_drop_snapshot = 0;
  static uint32_t high_loss_consec = 0;

  uint32_t current_time = micros();
  if (current_time - last_diag_print >= 1000000) {
    last_diag_print = current_time;

    auto &imu = Icm42688p::GetInstance();
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
      // Panic(ErrorCode::kImuDroppedFrame);
    }

    // Send diagnostics as binary (ESP32 does formatting)
    // Format ID 1: "Prof: Fast=%lu Link=%lu GPS=%lu GpsPub=%lu TSync=%lu
    // Step=%lu Slow=%lu Send=%lu"
    {
      uint32_t args[8] = {g_prof_fast_us,    g_prof_link_us,  g_prof_gps_us,
                          g_prof_gpspub_us,  g_prof_tsync_us, g_prof_step_us,
                          g_dbg_max_slow_us, max_send_us_};
      ctx.sys->GetFcLink().SendLogBinary(1, 8, args);
    }

    // Format ID 2: "Sched: GpsB=%lu SeqGap=%lu DtMax=%lu Drop=%lu/s Phase=%lu
    // Loss=%lu"
    {
      uint32_t args[6] = {g_dbg_max_gps_bytes,   max_seq_gap_,
                          max_raw_dt_us_,        drop_rate_per_sec,
                          max_phase_bad_consec_, high_loss_consec};
      ctx.sys->GetFcLink().SendLogBinary(2, 6, args);
    }

    (void)drop_rate_per_sec;
    (void)drops_now;

    // Reset profiling counters
    max_send_us_ = 0;
    g_dbg_max_slow_us = 0;
    g_dbg_max_gps_bytes = 0;
    max_seq_gap_ = 0;
    max_raw_dt_us_ = 0;
    max_phase_bad_consec_ = 0;
    g_prof_link_us = 0;
    g_prof_gps_us = 0;
    g_prof_gpspub_us = 0;
    g_prof_tsync_us = 0;
    g_prof_step_us = 0;
    g_prof_fast_us = 0;
  }
}

void IdleState::OnExit(AppContext &ctx, SmTick now) {
  (void)ctx;
  (void)now;
}

// --- NotIdleState (Unused) ---

void NotIdleState::OnEnter(AppContext &ctx, SmTick now) {
  (void)ctx;
  (void)now;
}
void NotIdleState::OnStep(AppContext &ctx, SmTick now) {
  (void)ctx;
  (void)now;
}
void NotIdleState::OnExit(AppContext &ctx, SmTick now) {
  (void)ctx;
  (void)now;
}
