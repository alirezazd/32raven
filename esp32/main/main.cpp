// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "state_machine.hpp"
#include "state_machine_context.hpp"
#include "system.hpp"
extern "C" {
#include "freertos/FreeRTOS.h"  // IWYU pragma: keep
#include "freertos/task.h"
}

namespace {

StateMachineContext ctx{};
StateMachine<StateMachineContext> sm(ctx);

}  // namespace

extern "C" void app_main(void) {  // NOLINT: IDF requires this exact signature
  System::GetInstance().Init();
  ctx.sm = &sm;

  ctx.now_ms = System::GetInstance().Timebase().NowMs();
  sm.Start(ctx.serving_state);
  while (true) {
    ctx.now_ms = System::GetInstance().Timebase().NowMs();
    sm.Step();
    vTaskDelay(1);  // the idle task's watchdog needs the loop to block
  }
}
