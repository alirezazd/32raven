// mavlink_tx_task.cpp
#include "panic.hpp"
#include "system.hpp"
#include "timebase.hpp"

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
    System::GetInstance().Mavlink().TxTick(NowMs());
    vTaskDelayUntil(&last_wake, kTxPeriodTicks);
  }
}

} // namespace

void StartMavlinkTxTask() {
  // One owner for RcRx UART TX.
  static constexpr uint32_t kStackWords = 4096 / sizeof(StackType_t);
  static constexpr UBaseType_t kPrio = 10;

  if (xTaskCreatePinnedToCore(MavlinkTxTask, "mav_tx", kStackWords, nullptr,
                              kPrio, nullptr, 0) != pdPASS) {
    Panic(ErrorCode::kMavlinkInitFailed);
  }
}
