#include "button.hpp"
#include "ctx.hpp"
#include "state_machine.hpp"
#include "states.hpp"
#include "system.hpp"
#include "timebase.hpp"

extern "C" {
#include "esp_timer.h"
#include "freertos/FreeRTOS.h" // IWYU pragma: keep
#include "freertos/task.h"
}

extern "C" void app_main(void) { // NOLINT
  // 1) Initialize all drivers
  System::init();

  // 2) Build application context
  // 2) Build application context
  AppContext ctx{};
  // sys, idle, listen are initialized in constructor

  // 3) Create state machine
  StateMachine<AppContext> sm(ctx);
  ctx.sm = &sm;

  // 4) Configure states
  if (ctx.listen) {
    ctx.listen->SetBlinkPeriod(300); // ms
  }

  // 5) Start in idle
  if (ctx.idle) {
    sm.start(*ctx.idle, NowMs());
  }

  // 6) Main loop
  while (true) {
    sm.step(NowMs());
    vTaskDelay(1); // must block at least 1 tick for watchdog
  }
}
