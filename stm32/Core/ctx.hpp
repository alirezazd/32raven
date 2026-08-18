// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include <cstdint>

struct IdleState;
struct ArmedState;
struct EscConfigState;
struct MscState;
struct IControlTickState;

class System;

template <typename Context>
class StateMachine;

struct AppContext {
  System *sys = nullptr;
  StateMachine<AppContext> *sm = nullptr;
  // Sampled once per pass, so everything in a pass agrees on when it started.
  uint32_t now_us = 0;
  IControlTickState *control_tick_state = nullptr;
  IdleState *idle_state = nullptr;
  ArmedState *armed_state = nullptr;
  EscConfigState *esc_config_state = nullptr;
  MscState *msc_state = nullptr;
};
