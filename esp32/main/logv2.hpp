#pragma once

// Set per-file log level before including esp_log.h
#ifndef LOG_LOCAL_LEVEL
#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#endif

extern "C" {
#include "esp_log.h"
}

// Convenience macros (optional)
#define LOGI(TAG, fmt, ...) ESP_LOGI(TAG, fmt, ##__VA_ARGS__)
#define LOGW(TAG, fmt, ...) ESP_LOGW(TAG, fmt, ##__VA_ARGS__)
#define LOGE(TAG, fmt, ...) ESP_LOGE(TAG, fmt, ##__VA_ARGS__)
#define LOGD(TAG, fmt, ...) ESP_LOGD(TAG, fmt, ##__VA_ARGS__)
#define LOGV(TAG, fmt, ...) ESP_LOGV(TAG, fmt, ##__VA_ARGS__)
