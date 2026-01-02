#include "timebase.hpp"

extern "C" {
#include "esp_timer.h"
}

time_ms_t now_ms() { return (time_ms_t)(esp_timer_get_time() / 1000); }
