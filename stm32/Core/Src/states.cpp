#include "states.hpp"

#include "board.h"
#include "user_config.hpp"

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
  ctx.sys->GetFcLink().Init(&ctx);

  last_imu_send_us_ = 0;
  last_status_send_us_ = 0;
  slow_loop_counter_ = 0;
  prev_imu_seq_ = 0;
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

  uint32_t seq_gap = 0;
  if (prev_imu_seq_ != 0 && first.seq > prev_imu_seq_) {
    seq_gap = first.seq - prev_imu_seq_ - 1u;
    if (seq_gap > max_seq_gap_) max_seq_gap_ = seq_gap;
    if (seq_gap > 0) {
      g_dbg_seq_gap_events++;
      g_dbg_seq_gap_total += seq_gap;
      static uint64_t last_seq_gap_log_us = 0;
      if (kEnableEspLogs &&
          first.timestamp_us - last_seq_gap_log_us >= 200000) {
        ctx.sys->GetFcLink().SendLog(
            "IMU_SEQ_GAP gap=%lu seq=%lu miss=%lu ovr=%lu", seq_gap, first.seq,
            imu.MainMissedTicks(), imu.ImuPathOverrun());
        last_seq_gap_log_us = first.timestamp_us;
      }
    }
  }
  prev_imu_seq_ = last.seq;

  static uint64_t last_overrun_log_us = 0;
  uint32_t overruns = imu.ImuPathOverrun();
  if (overruns > 0) {
    if (kEnableEspLogs && last.timestamp_us - last_overrun_log_us >= 1000000) {
      ctx.sys->GetFcLink().SendLog("IMU_OVERRUN fault=%lu", overruns);
      last_overrun_log_us = last.timestamp_us;
    }
  }

  for (uint8_t i = 0; i < batch.count; ++i) {
    const Icm42688p::Sample &raw = batch.samples[i];
    if (i > 0) {
      const uint32_t dt_us =
          (uint32_t)(raw.timestamp_us - batch.samples[i - 1u].timestamp_us);
      if (dt_us > max_raw_dt_us_) max_raw_dt_us_ = dt_us;
    }
  }

  // 3. Controller / Mixer / DShot (future)

  uint32_t t1 = ctx.sys->Time().Micros();

  // Capture fast-tick duration
  uint32_t fast_dt = (t1 > fast_t0) ? (t1 - fast_t0) : 0;
  if (fast_dt > g_prof_fast_us) g_prof_fast_us = fast_dt;
}

void IdleState::StepSlow(AppContext &ctx, SmTick now) {
  auto micros = [&]() -> uint32_t { return ctx.sys->Time().Micros(); };
  auto budget_exhausted = [&]() -> bool {
    return (uint32_t)(micros() - g_fast_last_us) >= kSlowBudgetFromFastUs;
  };

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
      ctx.sys->GetFcLink().SendLog(
          "IMU_FAULT ovr=%lu dma=%lu prs=%lu hdr=%lu", imu.ImuPathOverrun(),
          imu.DmaStartFailCount(), imu.ParseFailCount(), imu.LastBadHeader());
      g_fault_log_last_us = current_us;
    }
  }

  // 1. Poll Telemetry Link (UART1 <-> ESP32)
  {
    uint32_t t0 = micros();
    ctx.sys->GetFcLink().Poll(32, 32);
    uint32_t dt = micros() - t0;
    if (dt > g_prof_link_us) g_prof_link_us = dt;
  }
  ctx.sys->GetCrsfLinkService().PollRx(micros());
  ctx.sys->GetRcReceiver().Poll(micros());
  if (budget_exhausted()) return;

  // 2. Poll battery and refresh the blackboard copy.
  {
    ctx.sys->GetBattery().Poll(micros());
    ctx.sys->GetVehicleState().UpdateBattery(ctx.sys->GetBattery().GetData());
  }
  if (budget_exhausted()) return;

  // 3. Poll GPS (UART2 <-> M10)
  {
    uint32_t t0 = micros();
    auto &gps_uart = ctx.sys->GetUart2();
    auto &gps_svc = ctx.sys->ServiceGps();
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
    uint32_t t0 = micros();
    const Icm42688p::SampleBatch batch =
        Icm42688p::GetInstance().GetLatestBatch();
    if (batch.count > 0) {
      const Icm42688p::Sample &latest = batch.samples[batch.count - 1u];
      if (latest.timestamp_us - last_imu_send_us_ >= 20000) {
        last_imu_send_us_ = latest.timestamp_us;
        const Icm42688p::ScaledSample scaled = imu.ScaleSample(latest);
        ctx.sys->GetFcLink().SendImu(scaled.timestamp_us, scaled.accel_mps2,
                                     scaled.gyro_rad_s);
      }
    }
    uint32_t dt = micros() - t0;
    if (dt > max_send_us_) max_send_us_ = dt;
  }

  // 5. Process GPS Data (Decoupled)
  auto &gps_svc = ctx.sys->ServiceGps();
  GpsData t{};
  if (gps_svc.PopGpsData(ctx.sys->Time().Micros(), t)) {
    uint32_t t0 = micros();
    ctx.sys->GetVehicleState().UpdateGps(t);

    const BatteryData &bat = ctx.sys->GetVehicleState().GetBattery();
    ctx.sys->GetFcLink().SendGps(t, bat);

    uint32_t dt = micros() - t0;
    if (dt > g_prof_gpspub_us) g_prof_gpspub_us = dt;
  }

  if (budget_exhausted()) return;
  ctx.sys->GetCrsfLinkService().PollTx(micros());
  if (budget_exhausted()) return;

  // 6. Publish board/vehicle status and diagnostics at 1Hz.
  static uint32_t last_diag_print = 0;
  static uint32_t last_drop_snapshot = 0;
  static uint32_t high_loss_consec = 0;

  uint32_t current_time = micros();
  if (last_status_send_us_ == 0u ||
      current_time - last_status_send_us_ >= 1000000u) {
    last_status_send_us_ = current_time;
    ctx.sys->GetStatPublisher().Publish(ctx, current_time, slow_loop_counter_);
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
      // Panic(ErrorCode::kImuDroppedFrame);
    }

    if (kEnableEspLogs && kEnableImuDebugLog) {
      ctx.sys->GetFcLink().SendLog(
          "IMU_DBG irq=%lu pub=%lu miss=%lu ovr=%lu dma=%lu prs=%lu cnt=%lu "
          "sg=%lu/%lu dt=%lu",
          imu.IrqCount(), imu.PublishCount(), drops_now, imu.ImuPathOverrun(),
          imu.DmaStartFailCount(), imu.ParseFailCount(), imu.LastFifoCount(),
          g_dbg_seq_gap_events, g_dbg_seq_gap_total, max_raw_dt_us_);
    }

    if (kEnableEspLogs) {
      ctx.sys->GetFcLink().SendLog(
          "Prof: Fast=%lu Link=%lu GPS=%lu GpsPub=%lu Step=%lu Slow=%lu "
          "Send=%lu",
          g_prof_fast_us, g_prof_link_us, g_prof_gps_us, g_prof_gpspub_us,
          g_prof_step_us, g_dbg_max_slow_us, max_send_us_);
    }

    if (kEnableEspLogs) {
      ctx.sys->GetFcLink().SendLog(
          "Sched: GpsB=%lu SeqGap=%lu DtMax=%lu Drop=%lu/s Phase=%lu Loss=%lu",
          g_dbg_max_gps_bytes, max_seq_gap_, max_raw_dt_us_, drop_rate_per_sec,
          0u, high_loss_consec);
    }

    (void)drop_rate_per_sec;
    (void)drops_now;

    // Reset profiling counters
    max_send_us_ = 0;
    g_dbg_max_slow_us = 0;
    g_dbg_max_gps_bytes = 0;
    max_seq_gap_ = 0;
    max_raw_dt_us_ = 0;
    g_prof_link_us = 0;
    g_prof_gps_us = 0;
    g_prof_gpspub_us = 0;
    g_prof_step_us = 0;
    g_prof_fast_us = 0;
  }
}
