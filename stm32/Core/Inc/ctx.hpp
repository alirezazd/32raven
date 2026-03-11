#pragma once
#include <cstdint>

struct IdleState;

class System;

template <typename Context> class StateMachine;
struct IFastTickState;

struct AppContext {
  AppContext();

  System *sys = nullptr;
  StateMachine<AppContext> *sm = nullptr;
  IFastTickState *fast_tick_state = nullptr;

  IdleState *idle = nullptr;

  uint32_t telemetry_interval_ms = 0;
};
