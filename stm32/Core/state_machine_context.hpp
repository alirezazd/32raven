// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include <cstdint>

#include "state_machine.hpp"
#include "states.hpp"

struct StateMachineContext {
  StateMachine<StateMachineContext> *sm = nullptr;
  // Sampled once per pass, so everything in a pass agrees on when it started.
  uint32_t now_us = 0;
  IControlTickState *control_tick_state = nullptr;
  StandbyState standby_state;
  ArmedState armed_state;
  EscConfigState esc_config_state;
  MscState msc_state;
};
