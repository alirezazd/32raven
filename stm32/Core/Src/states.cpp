#include "states.hpp"

void IdleState::OnEnter(AppContext &ctx, SmTick now) {
  (void)now;
  // Default is idle
  ctx.sys->Led().Off();
}

void IdleState::OnStep(AppContext &ctx, SmTick now) {
  auto &btn = ctx.sys->Btn();
  btn.Poll(now);

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
  ctx.sys->Led().On();
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
