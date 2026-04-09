#pragma once

// Forward declarations
struct ServingState;
struct MavlinkWifiState;
struct DfuState;
struct ProgramState;

class System;

template <typename Context>
class StateMachine;

struct AppContext {
  System *sys = nullptr;
  StateMachine<AppContext> *sm = nullptr;
  ServingState *serving_state = nullptr;
  MavlinkWifiState *mavlink_wifi_state = nullptr;
  DfuState *dfu_state = nullptr;
  ProgramState *program_state = nullptr;
};
