#include "panic.hpp"
#include "driver/gpio.h"
#include "esp32_config.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h" // IWYU pragma: keep
#include "freertos/task.h"

static constexpr const char *kTag = "panic";

void Panic(ErrorCode code) {
  const char *msg = GetMessage(code);
  gpio_reset_pin(kPinMap.led);
  gpio_set_direction(kPinMap.led, GPIO_MODE_OUTPUT);
  ESP_LOGE(kTag, "PANIC [0x%08lX]: %s", (unsigned long)code, msg);
  int level = 0;
  while (true) {
    gpio_set_level(kPinMap.led, level);
    level = !level;
    ESP_LOGE(kTag, "PANIC [0x%08lX]: %s", (unsigned long)code, msg);
    vTaskDelay(pdMS_TO_TICKS(40));
  }
}
