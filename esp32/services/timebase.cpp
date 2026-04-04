#include "timebase.hpp"

extern "C" {
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"  // IWYU pragma: keep
#include "freertos/task.h"
}

TimeMs Timebase::NowMs() const { return (TimeMs)(esp_timer_get_time() / 1000); }

TimeUs Timebase::NowUs() const { return (TimeUs)esp_timer_get_time(); }

void Timebase::SleepMs(TimeMs ms) const { vTaskDelay(pdMS_TO_TICKS(ms)); }
