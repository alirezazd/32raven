// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "ctx.hpp"
#include "states.hpp"
#include "system.hpp"

namespace {

AppContext app{};
StateMachine<AppContext> sm(app);

IdleState idle_state;
ArmedState armed_state;
EscConfigState esc_config_state;
MscState msc_state;

}  // namespace

// The control tick, body of PendSV_Handler. Nothing here calls it: the sample
// interrupt pends PendSV once a burst is parsed, so the rate is the IMU's, not
// the main loop's, and this runs above every thread-mode caller below.
extern "C" void ImuTick(void) {
  // The burst lands in the SharedState's mailbox and Ahrs releases it once it
  // has consumed -- so there is nothing to acknowledge here. The sample
  // interrupt can pend this before main() has assigned the states, which is why
  // the hook is checked rather than assumed.
  if (app.control_tick_state != nullptr) {
    app.control_tick_state->OnControlTick(app);
  }
}

int main(void) {
  System::GetInstance().Init();
  app.sys = &System::GetInstance();
  app.sm = &sm;
  app.idle_state = &idle_state;
  app.armed_state = &armed_state;
  app.esc_config_state = &esc_config_state;
  app.msc_state = &msc_state;
  System::GetInstance().GetCommandHandler().Init();
  System::GetInstance().FcLinkSvc().Init(&app);

  app.now_us = app.sys->Time().Micros();
  sm.Start(idle_state);
  while (1) {
    app.now_us = app.sys->Time().Micros();
    app.sm->Step();
    app.sys->Wdg().Kick();  // main-loop liveness; wedged loop -> reset
    __WFI();
  }
}
