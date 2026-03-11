#include "ctx.hpp"
#include "states.hpp"
#include "system.hpp"
#include "user_config.hpp"
#include <cstdint>

namespace {

AppContext app{};
StateMachine<AppContext> sm(app);

} // namespace

extern "C" void ExpressMain(void) {
  if (!app.fast_tick_state) {
    return; // Fast lane not ready, skip. This can happen during startup before
            // main() runs.
  }
  const Icm42688p::SampleBatch batch =
      Icm42688p::GetInstance().GetLatestBatch();
  app.fast_tick_state->OnFastTick(app, batch);
}

int main(void) {
  System::GetInstance().Init(kSystemDefault);
  app.sm = &sm;
  sm.Start(*app.idle, app.sys->Time().Micros());
  while (1) {
    app.sm->Step(static_cast<SmTick>(app.sys->Time().Micros()));
    __WFI();
  }
}
