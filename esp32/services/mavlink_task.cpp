#include "esp32_limits.hpp"
#include "panic.hpp"
#include "system.hpp"

extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
}

namespace {

static constexpr TickType_t kWorkerPeriodTicks = pdMS_TO_TICKS(10);

static void MavlinkTask(void *) {
  TickType_t last_wake = xTaskGetTickCount();

  while (true) {
    Sys().Mavlink().WorkerTick(Sys().Timebase().NowMs());
    vTaskDelayUntil(&last_wake, kWorkerPeriodTicks);

    // Monitor stack usage and panic if critically low.
    const size_t stack_high_water_bytes =
        uxTaskGetStackHighWaterMark(nullptr) * sizeof(StackType_t);
    if (stack_high_water_bytes <
        esp32_limits::kMavlinkStackPanicThresholdBytes) {
      Panic(ErrorCode::kMavlinkStackOverflow);
    }
  }
}

}  // namespace

void StartMavlinkTask() {
  static constexpr uint32_t kStackDepthWords =
      static_cast<uint32_t>(esp32_limits::kMavlinkTaskStackDepthWords);
  static constexpr UBaseType_t kPrio = 10;
  static StaticTask_t task_buffer;
  static StackType_t task_stack[kStackDepthWords];
  static TaskHandle_t task_handle = nullptr;

  if (task_handle != nullptr) {
    Panic(ErrorCode::kMavlinkInitFailed);
  }

  task_handle = xTaskCreateStaticPinnedToCore(MavlinkTask, "mavlink",
                                              kStackDepthWords, nullptr, kPrio,
                                              task_stack, &task_buffer, 0);
  if (task_handle == nullptr) {
    Panic(ErrorCode::kMavlinkInitFailed);
  }
}
