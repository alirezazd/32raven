// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "state_machine_context.hpp"
#include "system.hpp"

namespace {

StateMachineContext ctx{};
StateMachine<StateMachineContext> sm(ctx);

}  // namespace

// The control tick is phase-locked to the IMU's, so the IMU tick drives it.
extern "C" void ImuTick(void) {
  if (ctx.control_tick_state != nullptr) {
    ctx.control_tick_state->OnControlTick(ctx);
  }
}

int main(void) {
  System::GetInstance().Init();
  ctx.sm = &sm;

  ctx.now_us = System::GetInstance().Time().Micros();
  sm.Start(ctx.standby_state);
  while (1) {
    ctx.now_us = System::GetInstance().Time().Micros();
    ctx.sm->Step();
    System::GetInstance().Wdg().Kick();
    __WFI();
  }
}
