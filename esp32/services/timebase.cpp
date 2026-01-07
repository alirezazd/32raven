#include "timebase.hpp"

extern "C" {
#include "esp_timer.h"
#include "freertos/FreeRTOS.h" // IWYU pragma: keep
#include "freertos/task.h"
}

TimeMs NowMs() { return (TimeMs)(esp_timer_get_time() / 1000); }

void SleepMs(TimeMs ms) {
  if (ms == 0)
    return;
  vTaskDelay(pdMS_TO_TICKS(ms));
}
