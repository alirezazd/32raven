#include "states.hpp"
#include "board.h"
#include "spi.hpp"
#include <cstdio>
#include <cstring>

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
    t.timestamp_us = ctx.sys->Time().Micros();
    t.lat = pvt.lat;
    t.lon = pvt.lon;
    t.alt = pvt.hMSL;
    t.vel = (uint16_t)(pvt.gSpeed / 10);    // mm/s -> cm/s
    t.hdg = (uint16_t)(pvt.headMot / 1000); // 1e-5 deg -> cdeg
    t.num_sats = pvt.numSV;
    t.fix_type = pvt.fixType;

    // Update Blackboard
    ctx.sys->GetVehicleState().UpdateGps(t);

    // Forward to ESP32 (if needed immediately, or let FcLink poll the
    // blackboard) For now, let's explicit push to FcLink to maintain original
    // behavior
    ctx.sys->GetFcLink().SendGps(t);

    gps_svc.ClearNewDataFlag();
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
