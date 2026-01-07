#pragma once

// TODO: Drivers, services and generics
// Set per-file log level before including esp_log.h
#ifndef LOG_LOCAL_LEVEL
#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#endif

extern "C" {
#include "esp_log.h"
}

// ANSI Color Codes
#define LOG_COLOR_RESET "\033[0m"
#define LOG_COLOR_RED "\033[0;31m"
#define LOG_COLOR_GREEN "\033[0;32m"
#define LOG_COLOR_YELLOW "\033[0;33m"
#define LOG_COLOR_BLUE "\033[0;34m"
#define LOG_COLOR_MAGENTA "\033[0;35m"
#define LOG_COLOR_CYAN "\033[0;36m"

// Convenience macros with color
#define LOGI(TAG, fmt, ...)                                                    \
  ESP_LOGI(TAG, LOG_COLOR_GREEN fmt LOG_COLOR_RESET, ##__VA_ARGS__)
#define LOGW(TAG, fmt, ...)                                                    \
  ESP_LOGW(TAG, LOG_COLOR_YELLOW fmt LOG_COLOR_RESET, ##__VA_ARGS__)
#define LOGE(TAG, fmt, ...)                                                    \
  ESP_LOGE(TAG, LOG_COLOR_RED fmt LOG_COLOR_RESET, ##__VA_ARGS__)
#define LOGD(TAG, fmt, ...)                                                    \
  ESP_LOGD(TAG, LOG_COLOR_CYAN fmt LOG_COLOR_RESET, ##__VA_ARGS__)
#define LOGV(TAG, fmt, ...) ESP_LOGV(TAG, fmt, ##__VA_ARGS__)
