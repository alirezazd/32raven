#include "timebase.hpp"

extern "C" {
#include "esp_timer.h"
}

TimeMs NowMs() { return (TimeMs)(esp_timer_get_time() / 1000); }
