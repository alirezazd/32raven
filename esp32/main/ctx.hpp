#pragma once

// Forward declarations
struct IdleState;
struct ListenState;

class System;

template <typename Context> class StateMachine;

struct AppContext {
  AppContext(); // Logic moved to cpp

  System *sys = nullptr;
  StateMachine<AppContext> *sm = nullptr;

  IdleState *idle_state = nullptr;
  ListenState *listen_state = nullptr;
};