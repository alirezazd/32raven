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

}  // namespace

extern "C" void ExpressMain(void) {
  if (!app.fast_tick_state) {
    return;  // ISR can fire before main() assigns the fast-tick state
  }
  const Icm42688p::SampleBatch batch =
      Icm42688p::GetInstance().GetLatestBatch();
  app.fast_tick_state->OnFastTick(app, batch);
}

int main(void) {
  System::GetInstance().Init();
  app.sys = &System::GetInstance();
  app.sm = &sm;
  app.idle_state = &idle_state;
  app.armed_state = &armed_state;
  app.esc_config_state = &esc_config_state;
  System::GetInstance().GetCommandHandler().Init();
  System::GetInstance().FcLinkSvc().Init(&app);

  sm.Start(idle_state);
  while (1) {
    app.sm->Step(app.sys->Time().Micros());
    app.sys->Wdg().Kick();  // main-loop liveness; wedged loop -> reset
    __WFI();
  }
}
