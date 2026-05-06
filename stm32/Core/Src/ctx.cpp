#include "ctx.hpp"

#include "states.hpp"
#include "system.hpp"
#include "user_config.hpp"

AppContext::AppContext() {
  sys = &System::GetInstance();
  telemetry_interval_ms = kFcLinkTelemetryIntervalMs;

  static IdleState idle_state;
  idle = &idle_state;
}
