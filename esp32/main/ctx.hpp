// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

// Forward declarations
struct ServingState;
struct MavlinkWifiState;
struct MavlinkUsbState;
struct DfuState;
struct ProgramState;
struct EscConfigState;

class System;

template <typename Context>
class StateMachine;

struct AppContext {
  System *sys = nullptr;
  StateMachine<AppContext> *sm = nullptr;
  ServingState *serving_state = nullptr;
  MavlinkWifiState *mavlink_wifi_state = nullptr;
  MavlinkUsbState *mavlink_usb_state = nullptr;
  DfuState *dfu_state = nullptr;
  ProgramState *program_state = nullptr;
  EscConfigState *esc_config_state = nullptr;
};
