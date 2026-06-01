// control::SetpointSmoother — rate-limit + first-order LPF chained.
//
// Use anywhere a discontinuous reference signal (RC stick step, mode
// switch, mission waypoint hand-off) would feed a feedback controller.
// Shapes the reference into a continuous trajectory before the
// controller sees it.
//
// rate_limit ≤ 0 disables slew limiting. lpf_alpha ≥ 1 disables LPF
// (pass-through). Both defaults give pass-through.

#pragma once

namespace control {

struct SmootherConfig {
  float rate_limit = 0.0f;
  float lpf_alpha = 1.0f;
};

class SetpointSmoother {
 public:
  constexpr SetpointSmoother() = default;

  void Init(const SmootherConfig &cfg) {
    cfg_ = cfg;
    cfg_.lpf_alpha = ClampUnit(cfg.lpf_alpha);
    Reset();
  }

  void SetConfig(const SmootherConfig &cfg) {
    cfg_ = cfg;
    cfg_.lpf_alpha = ClampUnit(cfg.lpf_alpha);
  }

  void Reset() {
    current_ = 0.0f;
    initialized_ = false;
  }

  void Reset(float initial) {
    current_ = initial;
    initialized_ = true;
  }

  // dt_s ≤ 0 returns the current value unchanged.
  float Update(float target, float dt_s) {
    if (dt_s <= 0.0f) return current_;

    if (!initialized_) {
      current_ = target;
      initialized_ = true;
      return current_;
    }

    float ramped = target;
    if (cfg_.rate_limit > 0.0f) {
      const float max_delta = cfg_.rate_limit * dt_s;
      const float delta = target - current_;
      if (delta > max_delta)
        ramped = current_ + max_delta;
      else if (delta < -max_delta)
        ramped = current_ - max_delta;
    }

    current_ = cfg_.lpf_alpha * ramped + (1.0f - cfg_.lpf_alpha) * current_;
    return current_;
  }

  float Current() const { return current_; }
  const SmootherConfig &Config() const { return cfg_; }

 private:
  static constexpr float ClampUnit(float a) {
    return (a <= 0.0f) ? 1.0f : (a > 1.0f ? 1.0f : a);
  }

  SmootherConfig cfg_{};
  float current_ = 0.0f;
  bool initialized_ = false;
};

}  // namespace control
