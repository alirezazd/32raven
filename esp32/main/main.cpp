#include "button.hpp"
#include "ctx.hpp"
#include "state_machine.hpp"
#include "states.hpp"
#include "system.hpp"

extern "C" {
#include "esp_timer.h"
#include "freertos/FreeRTOS.h" // IWYU pragma: keep
#include "freertos/task.h"
}

static inline sm_tick_t NowMs() {
  return (sm_tick_t)(esp_timer_get_time() / 1000);
}

extern "C" void app_main(void) { // NOLINT
  // 1) Initialize all drivers
  System::init();

  // 2) Build application context
  AppContext ctx{};
  ctx.sys = &System::getInstance();

  // 3) Create state machine
  StateMachine<AppContext> sm(ctx);
  ctx.sm = &sm;

  // 4) Create states
  IdleState idle;
  ListenState listen;

  idle.SetListen(&listen);
  listen.SetIdle(&idle);
  listen.SetBlinkPeriod(300); // ms

  // 5) Start in idle
  sm.start(idle, NowMs());

  // 6) Main loop
  while (true) {
    sm.step(NowMs());
    vTaskDelay(1); // must block at least 1 tick for watchdog
  }
}
