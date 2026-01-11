#include "states.hpp"
#include <cstdio>
#include <cstdlib>

void IdleState::OnEnter(AppContext &ctx, SmTick now) {
  (void)now;
  // Default is idle
  ctx.sys->Led().Set(false);

  // Configure GPS
  ublox::M9NConfig config;
  config.baudrate = 38400; // Match Uart2 Config
  ctx.sys->GetGps().configure(config, [&](const uint8_t *data, size_t len) {
    ctx.sys->GetUart2().Send(data, len);
  });
}

void IdleState::OnStep(AppContext &ctx, SmTick now) {
  auto &btn = ctx.sys->Btn();
  btn.Poll(now);

  auto &uart = ctx.sys->GetUart();      // Console
  auto &gps_uart = ctx.sys->GetUart2(); // GPS
  auto &gps = ctx.sys->GetGps();

  // Heartbeat Blink
  static uint32_t last_blink = 0;
  if (now - last_blink > 500) {
    ctx.sys->Led().Toggle();
    last_blink = now;
  }

  // Process GPS Data
  uint8_t c;
  while (gps_uart.Read(c)) {
    if (gps.parse(c)) {
      const auto &pvt = gps.pvt();
      char buf[128];

      int32_t lat = pvt.lat;
      int32_t lon = pvt.lon;
      char lat_c = (lat >= 0) ? 'N' : 'S';
      char lon_c = (lon >= 0) ? 'E' : 'W';
      lat = std::abs(lat);
      lon = std::abs(lon);

      std::snprintf(
          buf, sizeof(buf),
          "\r\nGPSv2: Fix:%d Sats:%d Lat:%ld.%07ld %c Lon:%ld.%07ld %c "
          "Alt:%dm Spd:%ld.%02ldm/s Hdg:%ld.%02lddeg\r\n",
          pvt.fixType, pvt.numSV, (long)lat / 10000000, (long)lat % 10000000,
          lat_c, (long)lon / 10000000, (long)lon % 10000000, lon_c,
          (int)pvt.hMSL / 1000, (long)pvt.gSpeed / 1000,
          ((long)pvt.gSpeed % 1000) / 10, (long)pvt.headMot / 100000,
          ((long)pvt.headMot % 100000) / 1000);
      uart.Send(buf);
    }
  }

  // Echo Console (optional, keeping minimal for now to avoid clutter)
  while (uart.Read(c)) {
    // uart.Send(&c, 1);
  }

  if (btn.ConsumeLongPress()) {
    ctx.sm->ReqTransition(*ctx.not_idle);
  }

  // Discard press events as we only care about long press here
  btn.ConsumePress();
}

void IdleState::OnExit(AppContext &ctx, SmTick now) {
  (void)ctx;
  (void)now;
}

void NotIdleState::OnEnter(AppContext &ctx, SmTick now) {
  (void)now;
  ctx.sys->Led().Set(true);
  ctx.sys->GetUart().Send("\nHello From STM32\r\n");
}

void NotIdleState::OnStep(AppContext &ctx, SmTick now) {
  auto &btn = ctx.sys->Btn();
  btn.Poll(now);

  // If button is pressed again (short or long), go back to idle
  // Discard short press, only long press aborts
  btn.ConsumePress();

  if (btn.ConsumeLongPress()) {
    ctx.sys->GetUart().Send("ABORT\r\n");
    ctx.sm->ReqTransition(*ctx.idle);
  }
}

void NotIdleState::OnExit(AppContext &ctx, SmTick now) {
  (void)ctx;
  (void)now;
}
