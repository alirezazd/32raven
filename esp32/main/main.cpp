#include "ctx.hpp"
#include "state_machine.hpp"
#include "states.hpp"
#include "system.hpp"
extern "C" {
#include "freertos/FreeRTOS.h"  // IWYU pragma: keep
#include "freertos/task.h"
}

namespace {

AppContext MakeAppContext() {
  static ServingState serving;
  static DfuState dfu;
  static ProgramState program;
  static HardErrorState hard_error;

  AppContext ctx{};
  ctx.sys = &Sys();
  ctx.serving_state = &serving;
  ctx.dfu_state = &dfu;
  ctx.program_state = &program;
  ctx.hard_error_state = &hard_error;
  return ctx;
}

}  // namespace

extern "C" void app_main(void) {  // NOLINT as IDF requires app_main
  Sys().Init();
  AppContext ctx = MakeAppContext();
  StateMachine<AppContext> sm(ctx);
  ctx.sm = &sm;
  sm.Start(*ctx.serving_state);
  while (true) {
    sm.Step(Sys().Timebase().NowMs());
    vTaskDelay(1);  // must block at least 1 tick for watchdog
  }
}
