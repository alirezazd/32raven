// Widget dependency note:
// This widget uses FreeRTOS idle run-time stats for CPU%.
//
// Required ESP-IDF menuconfig settings:
//   Component config -> FreeRTOS -> Kernel -> configGENERATE_RUN_TIME_STATS
//     CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS=y
//   Component config -> FreeRTOS -> Port -> Choose the clock source for run
//   time stats
//     CONFIG_FREERTOS_RUN_TIME_STATS_USING_ESP_TIMER=y
//
// Automatically selected by configGENERATE_RUN_TIME_STATS:
//   Component config -> FreeRTOS -> Kernel -> configUSE_TRACE_FACILITY
//     CONFIG_FREERTOS_USE_TRACE_FACILITY=y
//   Component config -> FreeRTOS -> Kernel ->
//   configUSE_STATS_FORMATTING_FUNCTIONS
//     CONFIG_FREERTOS_USE_STATS_FORMATTING_FUNCTIONS=y

#include "resource_utilization_widget.hpp"

#include <algorithm>
#include <cstdio>

#include "system.hpp"
#include "timebase.hpp"

extern "C" {
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
}

namespace {

constexpr DisplayTextStyle kTextStyle{
    .scale = 1,
};
constexpr TimeMs kMinUpdatePeriodMs = 1;
constexpr int16_t kTextInsetX = 1;
constexpr int16_t kLineSpacing = 1;
constexpr char kSampleLine[] = "CPU:100%";

uint32_t ToKiB(size_t bytes) { return static_cast<uint32_t>(bytes / 1024u); }

}  // namespace

void ResourceUtilizationWidget::OnEnter(WidgetContext &ctx) {
  SampleMetrics();
  Render(ctx);
  next_update_ms_ = TimeAfter(Sys().Timebase().NowMs(), kUpdatePeriodMs);
}

void ResourceUtilizationWidget::OnStep(WidgetContext &ctx, TimeMs now) {
  if (!TimeReached(now, next_update_ms_)) {
    return;
  }

  SampleMetrics();
  Render(ctx);
  next_update_ms_ =
      TimeAfter(now, std::max(kUpdatePeriodMs, kMinUpdatePeriodMs));
}

void ResourceUtilizationWidget::SampleMetrics() {
  free_heap_bytes_ = heap_caps_get_free_size(MALLOC_CAP_8BIT);
  min_free_heap_bytes_ = heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT);
  largest_block_bytes_ = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);

  const uint64_t sample_time_us = Sys().Timebase().NowUs();
  const uint32_t idle_runtime = ulTaskGetIdleRunTimeCounter();
  const uint32_t idle_percent = std::min<uint32_t>(
      100u, static_cast<uint32_t>(ulTaskGetIdleRunTimePercent()));

  uint32_t cpu_percent = 100u - idle_percent;
  if (have_runtime_sample_ && sample_time_us > previous_sample_time_us_ &&
      idle_runtime >= previous_idle_runtime_) {
    const uint64_t elapsed_us = sample_time_us - previous_sample_time_us_;
    const uint32_t idle_delta = idle_runtime - previous_idle_runtime_;
    if (elapsed_us > 0) {
      const uint32_t sampled_idle_percent = std::min<uint32_t>(
          100u, static_cast<uint32_t>(
                    (static_cast<uint64_t>(idle_delta) * 100u) / elapsed_us));
      cpu_percent = 100u - sampled_idle_percent;
    }
  }

  previous_idle_runtime_ = idle_runtime;
  previous_sample_time_us_ = sample_time_us;
  have_runtime_sample_ = true;
  cpu_percent_ = cpu_percent;
}

void ResourceUtilizationWidget::Render(WidgetContext &ctx) const {
  if (ctx.ui == nullptr || ctx.renderer == nullptr) {
    return;
  }

  Ui &ui = *ctx.ui;
  DisplayRenderer &renderer = *ctx.renderer;
  const DisplayTextBounds sample_bounds =
      renderer.MeasureText(kSampleLine, kTextStyle);
  if (sample_bounds.width == 0 || sample_bounds.height == 0) {
    renderer.Clear();
    return;
  }

  const int16_t line_height = static_cast<int16_t>(sample_bounds.height);
  const int16_t line_step = static_cast<int16_t>(line_height + kLineSpacing);
  const int16_t total_height =
      static_cast<int16_t>((4 * line_height) + (3 * kLineSpacing));
  int16_t line_top = std::max<int16_t>(
      0, static_cast<int16_t>(ui.Height() - total_height) / 2);

  char lines[4][16];
  std::snprintf(lines[0], sizeof(lines[0]), "CPU:%3u%%",
                static_cast<unsigned>(cpu_percent_));
  std::snprintf(lines[1], sizeof(lines[1]), "RAM:%3uK",
                static_cast<unsigned>(ToKiB(free_heap_bytes_)));
  std::snprintf(lines[2], sizeof(lines[2]), "MIN:%3uK",
                static_cast<unsigned>(ToKiB(min_free_heap_bytes_)));
  std::snprintf(lines[3], sizeof(lines[3]), "BLK:%3uK",
                static_cast<unsigned>(ToKiB(largest_block_bytes_)));

  renderer.Clear();
  for (const auto &line : lines) {
    const int16_t cursor_y = static_cast<int16_t>(line_top - sample_bounds.y);
    renderer.DrawText(line, kTextInsetX, cursor_y, kTextStyle);
    line_top = static_cast<int16_t>(line_top + line_step);
  }
}
