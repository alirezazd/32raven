// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include <cstdint>

#include "error_code.hpp"
#include "esc_service.hpp"
#include "fc_link.hpp"
#include "icm42688p.hpp"
#include "rate_controller.hpp"
#include "shared_state.hpp"

// Arm-refusal reasons, a bitmask because several often hold at once.
// No never-seen-RC bit: arming over FcLink with no transmitter must work.
inline constexpr uint16_t kArmBlockNotStandby = 1u << 0;
inline constexpr uint16_t kArmBlockSwitchNotCycled = 1u << 1;
inline constexpr uint16_t kArmBlockRcLoss = 1u << 2;
inline constexpr uint16_t kArmBlockLowBattery = 1u << 3;
inline constexpr uint16_t kArmBlockThrottleHigh = 1u << 4;
inline constexpr uint16_t kArmBlockGyroUncalibrated = 1u << 5;
inline constexpr uint16_t kArmBlockMagInterference = 1u << 6;

// How far the RC inputs are trusted, not a failsafe state: no phase changes
// what the vehicle does, so #52's descent belongs in the state machine.
enum class RcLinkPhase : uint8_t {
  // Frames arriving, or none ever seen.
  kUp = 0,
  // Silent past the timeout, still flying the last frame. Expiry disarms on
  // the way to kRecovering rather than through a phase of its own.
  kGuard,
  // Disarmed, holding arming shut until a clean stream proves the link.
  kRecovering,
};

// The board's safety authority. It writes only the arm state and failsafe
// flags; the state machine sequences flight and EscService drives the motors.
class Sentinel {
 public:
  static Sentinel &GetInstance();

  // Thresholds only: a driver counts, Sentinel decides how much is too much.
  struct Config {
    // Derived by the generator from the record rate, so it tracks rate changes.
    uint32_t imu_loss_threshold_samples;
    uint32_t imu_loss_window_us;
    // Windows in a row over the threshold; 1 removes the hysteresis.
    uint32_t imu_loss_consecutive;
    uint32_t imu_fault_threshold;
    uint32_t imu_fault_window_us;
    uint32_t imu_stall_timeout_us;
    // Silence from the bench host before the deadman cuts the throttle.
    uint32_t test_throttle_silence_us;
    // 1-based; 0 leaves arming reachable only over FcLink. Read raw off
    // channels_raw, because calibration stops at the four control axes.
    uint8_t arm_rc_channel;
    // Inclusive. A window rather than a threshold selects a three-position
    // switch's middle detent and reverses without a polarity knob.
    uint16_t arm_range_min_us;
    uint16_t arm_range_max_us;
    // The only RC-loss detector (CRSF has no failsafe bit; ELRS goes quiet).
    // Starts the guard; not a disarm.
    uint32_t rc_loss_timeout_us;
    // Flight on the pilot's last frame before the disarm: the window a dropout
    // can recover in, and the time spent uncommanded.
    uint32_t failsafe_guard_us;
    // Clean RC required before arming is allowed again, so an intermittent
    // link cannot re-arm on a switch nobody moved.
    uint32_t failsafe_recovery_us;
    // Pack mV below which arming is refused; 0 disables. Ground-only: in
    // flight it sags with throttle, and acting on it there is #50's landing.
    uint32_t arm_battery_min_mv;
    // Calibrated throttle above which arming is refused; 0 disables. Backs up
    // a handset that has no throttle-idle arm condition of its own.
    uint16_t arm_throttle_max_us;
  };

  // Every arm request lands here. Refusals are announced inside rather than
  // returned, so no caller can forget to sound one.
  void RequestArm(bool armed);

  // Main tick only, never the control loop: PendSV is pended by the sample
  // interrupt, so a watchdog there would go silent when the IMU stalls.
  void Supervise(uint32_t now_us);

 private:
  friend class System;

  void Init(const Config &cfg, SharedState &blackboard, EscService &esc,
            RateController &rate_controller, Icm42688p &imu, FcLink &fc_link);

  Sentinel() = default;
  ~Sentinel() = default;
  Sentinel(const Sentinel &) = delete;
  Sentinel &operator=(const Sentinel &) = delete;

  void SuperviseImu(uint32_t now_us);
  // The one failure a stop frame cannot answer: DShot refusing every write.
  void SuperviseEscOutput(uint32_t now_us);
  void SuperviseTestThrottle(uint32_t now_us);
  void RecoverStalledImu(uint32_t now_us);
  void RaiseImuFault(ErrorCode::Stm32 code);

  // One tone for every refusal and the bench cut: nothing audible yet says
  // which interlock said no.
  void EmitWarning();

  // Takes the blockers raised elsewhere so the published set is written once.
  void SuperviseRc(uint32_t now_us, uint16_t state_blockers);
  // Also disarms when the guard runs out.
  RcLinkPhase StepRcLink(uint32_t now_us, bool link_ok);
  uint16_t BatteryBlocker() const;

  Config cfg_{};
  SharedState *blackboard_ = nullptr;
  EscService *esc_ = nullptr;
  RateController *rate_controller_ = nullptr;
  Icm42688p *imu_ = nullptr;
  FcLink *fc_link_ = nullptr;
  uint32_t last_imu_recovery_us_ = 0;
  uint32_t last_loss_window_us_ = 0;
  uint32_t last_missed_samples_ = 0;
  uint32_t last_fault_window_us_ = 0;
  uint32_t last_path_faults_ = 0;
  uint32_t last_esc_window_us_ = 0;
  uint32_t last_esc_drops_ = 0;
  uint32_t high_loss_consec_ = 0;
  // 0 means no throttle standing, so a stale stamp cannot cut a new session.
  uint32_t last_host_us_ = 0;
  uint32_t last_msp_requests_ = 0;
  // Blocks arming until the first Supervise pass computes the real set.
  uint16_t arm_blockers_ = kArmBlockNotStandby;
  RcLinkPhase rc_link_phase_ = RcLinkPhase::kUp;
  // Stamped on every phase change; only kGuard reads it.
  uint32_t rc_link_entered_us_ = 0;
  // Zeroed on any bad pass so recovery times a clean stretch, not one frame.
  uint32_t rc_recovery_since_us_ = 0;
  // Set at boot, by every disarm and refusal, and while the loop is stopped, so
  // only a switch movement can arm; cleared by seeing the switch out of window.
  bool arm_switch_blocked_ = true;
  // The switch can disarm only once seen in-window: a bench fixture leaves
  // the arm channel at zero, and that must not cut a test.
  bool arm_switch_authority_ = false;
  // Edge, so a switch held against an interlock sounds once, not every pass;
  // a privileged request announces every time.
  uint16_t switch_refusal_announced_ = 0;
  bool imu_fault_latched_ = false;
  // Only read while latched, so its initial value never reaches a Panic.
  ErrorCode::Stm32 imu_fault_code_{};
  bool initialized_ = false;
};
