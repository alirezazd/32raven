// mavlink_tx_task.cpp
#include "panic.hpp"
#include "system.hpp"

extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
}

namespace {

// Keep the TX cadence stable and readable.
// 10ms = 100Hz is a good default for low-latency acks and smooth scheduling.
static constexpr TickType_t kTxPeriodTicks = pdMS_TO_TICKS(10);

static void MavlinkTxTask(void *) {
  // Align to a periodic schedule to reduce jitter.
  TickType_t last_wake = xTaskGetTickCount();

  while (true) {
    Sys().Mavlink().TxTick(Sys().Timebase().NowMs());
    vTaskDelayUntil(&last_wake, kTxPeriodTicks);
  }
}

}  // namespace

void StartMavlinkTxTask() {
  // One owner for RcRx UART TX.
  static constexpr uint32_t kStackBytes = 4096;
  static constexpr UBaseType_t kPrio = 10;
  static StaticTask_t task_buffer;
  static StackType_t task_stack[kStackBytes];
  static TaskHandle_t task_handle = nullptr;

  if (task_handle != nullptr) {
    Panic(ErrorCode::kMavlinkInitFailed);
  }

  task_handle = xTaskCreateStaticPinnedToCore(MavlinkTxTask, "mav_tx",
                                              kStackBytes, nullptr, kPrio,
                                              task_stack, &task_buffer, 0);
  if (task_handle == nullptr) {
    Panic(ErrorCode::kMavlinkInitFailed);
  }
}
