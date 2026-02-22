#include "panic.hpp"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h" // IWYU pragma: keep
#include "freertos/task.h"

static constexpr const char *kTag = "panic";

void Panic(ErrorCode code) {
  const char *msg = GetMessage(code);
  gpio_reset_pin(GPIO_NUM_8);
  gpio_set_direction(GPIO_NUM_8, GPIO_MODE_OUTPUT);
  ESP_LOGE(kTag, "PANIC [0x%08lX]: %s", (unsigned long)code, msg);
  int level = 0;
  while (true) {
    gpio_set_level(GPIO_NUM_8, level);
    level = !level;
    ESP_LOGE(kTag, "PANIC [0x%08lX]: %s", (unsigned long)code, msg);
    vTaskDelay(pdMS_TO_TICKS(40));
  }
}
