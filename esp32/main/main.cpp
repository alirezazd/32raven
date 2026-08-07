// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

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
  static ServingState serving_state;
  static MavlinkWifiState mavlink_wifi_state;
  static MavlinkUsbState mavlink_usb_state;
  static DfuState dfu_state;
  static ProgramState program_state;
  static EscConfigState esc_config_state;

  AppContext ctx{};
  ctx.sys = &Sys();
  ctx.serving_state = &serving_state;
  ctx.mavlink_wifi_state = &mavlink_wifi_state;
  ctx.mavlink_usb_state = &mavlink_usb_state;
  ctx.dfu_state = &dfu_state;
  ctx.program_state = &program_state;
  ctx.esc_config_state = &esc_config_state;
  return ctx;
}

}  // namespace

extern "C" void app_main(void) {  // NOLINT: IDF requires this exact signature
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
