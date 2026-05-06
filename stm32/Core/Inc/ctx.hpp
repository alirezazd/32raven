#pragma once

class System;

template <typename Context>
class StateMachine;
struct IFastTickState;

struct AppContext {
  AppContext();

  System *sys = nullptr;
  StateMachine<AppContext> *sm = nullptr;
  IFastTickState *fast_tick_state = nullptr;
};
