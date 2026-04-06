#pragma once

#include <cstddef>
#include <cstdint>

#include "ui.hpp"

class ResourceUtilizationWidget : public IWidget {
 public:
  const char *Name() const override { return "resource_utilization"; }

  void OnEnter(WidgetContext &ctx) override;
  void OnStep(WidgetContext &ctx, TimeMs now) override;

 private:
  void SampleMetrics();
  void Render(WidgetContext &ctx) const;

  static constexpr TimeMs kUpdatePeriodMs = 250;

  TimeMs next_update_ms_ = 0;
  uint32_t cpu_percent_ = 0;
  size_t free_heap_bytes_ = 0;
  size_t min_free_heap_bytes_ = 0;
  size_t largest_block_bytes_ = 0;
  uint32_t previous_idle_runtime_ = 0;
  uint64_t previous_sample_time_us_ = 0;
  bool have_runtime_sample_ = false;
};
