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

  // Heartbeat Blink + WhoAmI Check
  static uint32_t last_blink = 0;
  if (now - last_blink > 500) {
    ctx.sys->Led().Toggle();
    last_blink = now;

    uint32_t ts = ctx.sys->GetImu().GetLastDrdyTime();

    char buf[64];
    std::snprintf(buf, sizeof(buf), "IMU Last IRQ: %lu\r\n", ts);
    uart.Send(buf);
  }

  // Process GPS Data (Validation Only)
  auto &service = ctx.sys->ServiceM9N();
  uint8_t c;
  while (gps_uart.Read(c)) {
    service.ProcessByte(c);
    // GPS printing removed as requested
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
