#include "states.hpp"
#include "board.h"
#include "spi.hpp"
#include <cstdio>
#include <cstdlib>

void IdleState::OnEnter(AppContext &ctx, SmTick now) {
  (void)now;
  ctx.sys->Led().Set(false);
}

void IdleState::OnStep(AppContext &ctx, SmTick now) {
  // Ensure IMU is initialized (lazy init)

  auto &btn = ctx.sys->Btn();
  btn.Poll(now);

  auto &uart = ctx.sys->GetUart();      // Console
  auto &gps_uart = ctx.sys->GetUart2(); // GPS

  // Loop & Mag (IMU) Rate Statistics
  static uint32_t last_stats_time = 0;
  static uint32_t loop_count = 0;

  loop_count++;

  if (now - last_stats_time >= 1000) {
    uint32_t dt = now - last_stats_time;

    uint32_t loop_hz = loop_count * 1000 / dt;

    // Print as kHz with 2 decimal places (e.g. 12345 Hz -> 12.34 kHz)
    char buf[64];
    std::snprintf(buf, sizeof(buf), "Loop: %lu.%02lu kHz | Last IRQ: %lu\r\n",
                  (unsigned long)(loop_hz / 1000),
                  (unsigned long)((loop_hz % 1000) / 10),
                  (unsigned long)ctx.sys->GetImu().GetLastDrdyTime());
    uart.Send(buf);

    last_stats_time = now;
    loop_count = 0;
  }

  // Process GPS Data (Validation Only)
  auto &service = ctx.sys->ServiceM9N();
  uint8_t c;
  while (gps_uart.Read(c)) {
    service.ProcessByte(c);
    if (service.NewDataAvailable()) {
      static uint32_t last_gps_print = 0;
      if (now - last_gps_print > 1000) {
        const auto &pvt = service.GetData();
        char fix_str[16];
        if (pvt.fixType == 2) {
          std::snprintf(fix_str, sizeof(fix_str), "2D");
        } else if (pvt.fixType == 3) {
          if (pvt.flags & 0x02) { // diffSoln
            std::snprintf(fix_str, sizeof(fix_str), "3D/DGNSS");
          } else {
            std::snprintf(fix_str, sizeof(fix_str), "3D");
          }
        } else {
          std::snprintf(fix_str, sizeof(fix_str), "No Fix");
        }

        char buf[64];
        std::snprintf(buf, sizeof(buf), "GPS: %s (Sats: %u)\r\n", fix_str,
                      pvt.numSV);
        uart.Send(buf);
        last_gps_print = now;
      }
      service.ClearNewDataFlag();
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
