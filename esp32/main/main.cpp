#include "button.hpp"
#include "ctx.hpp"
#include "state_machine.hpp"
#include "states.hpp"
#include "system.hpp"
#include "timebase.hpp"

extern "C" {
#include "freertos/FreeRTOS.h" // IWYU pragma: keep
#include "freertos/task.h"
}

extern "C" void app_main(void) { // NOLINT
  System::init();

  AppContext ctx{};

  StateMachine<AppContext> sm(ctx);
  ctx.sm = &sm;

  if (ctx.listen_state) {
    ctx.listen_state->SetBlinkPeriod(300); // ms
  }

  if (ctx.idle_state) {
    sm.Start(*ctx.idle_state, NowMs());
  }
  while (true) {
    sm.Step(NowMs());
    vTaskDelay(1); // must block at least 1 tick for watchdog
  }
}
