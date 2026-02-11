#include "states.hpp"
#include "board.h"
#include "icm42688p.hpp"
#include "spi.hpp"
#include <cstdio>
#include <cstring>

// Scale factors for ICM-42688-P at ±16g / ±2000dps
// Accel: 16g / 32768 * 9.80665 = m/s²  per LSB
// Gyro:  2000 / 32768 * π/180  = rad/s  per LSB
static constexpr float kAccelScale =
    16.0f / 32768.0f * 9.80665f; // ~0.00479 m/s²
static constexpr float kGyroScale =
    2000.0f / 32768.0f * 3.14159265358979f / 180.0f; // ~0.00107 rad/s

// --- IdleState (Main State) ---

void IdleState::OnEnter(AppContext &ctx, SmTick now) {
  (void)now;
  ctx.sys->Led().Set(false);

  // Init Protocol Links
  ctx.sys->GetCommandHandler().Init();
  ctx.sys->GetFcLink().Init(&ctx);
}

void IdleState::OnStep(AppContext &ctx, SmTick now) {
  // 1. Poll Telemetry Link (UART1 <-> ESP32)
  ctx.sys->GetFcLink().Poll();

  // 2. Poll GPS (UART2 <-> M9N)
  auto &gps_uart = ctx.sys->GetUart2();
  auto &gps_svc = ctx.sys->ServiceM9N();
  uint8_t c;
  while (gps_uart.Read(c)) {
    gps_svc.ProcessByte(c);
  }

  // 3. Process GPS Data (Decoupled)
  if (gps_svc.NewDataAvailable()) {
    const auto &pvt = gps_svc.GetData();

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

    // Update Blackboard
    ctx.sys->GetVehicleState().UpdateGps(t);

    // Forward to ESP32 (if needed immediately, or let FcLink poll the
    // blackboard) For now, let's explicit push to FcLink to maintain original
    // behavior
    BatteryData bat = {0};
    ctx.sys->GetFcLink().SendGps(t, bat);

    gps_svc.ClearNewDataFlag();
  }

  // 4. Process IMU Data
  {
    auto &imu = Icm42688p::GetInstance();
    Icm42688p::Sample raw;
    // TODO: remove – temp rate logger
    static uint32_t imu_cnt = 0;
    static uint32_t imu_log_ms = 0;
    if (imu.PopSample(raw)) {
      imu_cnt++;
      ImuData d{};
      d.timestamp_us = raw.timestamp_us;
      d.accel[0] = (float)raw.accel[0] * kAccelScale;
      d.accel[1] = (float)raw.accel[1] * kAccelScale;
      d.accel[2] = (float)raw.accel[2] * kAccelScale;
      d.gyro[0] = (float)raw.gyro[0] * kGyroScale;
      d.gyro[1] = (float)raw.gyro[1] * kGyroScale;
      d.gyro[2] = (float)raw.gyro[2] * kGyroScale;
      d.valid = true;
      ctx.sys->GetVehicleState().UpdateImu(d);

      // Send over telemetry at 50Hz (every 20ms)
      if (d.timestamp_us - last_imu_send_us_ >= 20000) {
        last_imu_send_us_ = d.timestamp_us;
        ctx.sys->GetFcLink().SendImu(d);
      }
    }
    // TODO: remove – temp rate logger
    // temp rate logger (uses raw TIM2 CNT, 1 MHz assumed)
    static uint32_t imu_log_t0 = 0;
    static uint32_t imu_cnt0 = 0;

    uint32_t now_us = TIM2->CNT; // raw 1MHz tick (wraps ~71 min)

    if (imu_log_t0 == 0) {
      imu_log_t0 = now_us;
      imu_cnt0 = imu_cnt;
    }

    uint32_t dt_us = now_us - imu_log_t0;
    if (dt_us >= 1000000u) {
      uint32_t dcnt = imu_cnt - imu_cnt0;

      // milli-Hz = samples / seconds * 1000
      uint32_t hz_x1000 = (uint32_t)((uint64_t)dcnt * 1000000000ull / dt_us);

      ctx.sys->GetFcLink().SendLog("IMU: %lu.%03lu Hz ovr:%lu",
                                   hz_x1000 / 1000u, hz_x1000 % 1000u,
                                   imu.OverrunCount());

      imu_log_t0 = now_us;
      imu_cnt0 = imu_cnt;
    }
  }

  // 5. Send TimeSync (1Hz)
  if (now - last_time_sync_ms_ >= 1000) {
    last_time_sync_ms_ = now;

    message::TimeSyncMsg msg;
    msg.timestamp = ctx.sys->Time().MicrosCorrected();
    msg.drift_micros = (int32_t)ctx.sys->Time().GetDriftMicros();
    msg.synced = ctx.sys->Time().IsGpsSynchronized() ? 1 : 0;

    ctx.sys->GetFcLink().SendTimeSync(msg);
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
