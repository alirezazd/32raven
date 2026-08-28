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

// Why an arm request would be refused. A bitmask rather than an enum because
// the common case is several at once -- an uncycled switch on a link that has
// just come back is one situation, not two competing answers. Roadmap #28
// gives the reason to keep them apart at all: "it will not arm and will not
// say why" is its own failure mode.
//
// Sentinel's own, like the phase below: the gate is the only reader, and
// nothing off the board is told which interlock refused -- the tone a refusal
// makes is the same whichever it was. A reader that wanted the reason is what
// would put this back on the blackboard.
//
// Deliberately no never-seen-RC member. Arming over FcLink with no transmitter
// powered on has to keep working, which is #15's _manual_control_lost_at_arming
// rule stated from the other side.
inline constexpr uint16_t kArmBlockNotIdle = 1u << 0;
inline constexpr uint16_t kArmBlockSwitchNotCycled = 1u << 1;
inline constexpr uint16_t kArmBlockRcLoss = 1u << 2;
inline constexpr uint16_t kArmBlockLowBattery = 1u << 3;
inline constexpr uint16_t kArmBlockThrottleHigh = 1u << 4;

// How much the RC link is currently trusted. Deliberately not called a
// failsafe *state*: none of these change what the vehicle does. kGuard flies
// the pilot's last frame exactly as kUp does, and kRecovering is an arming
// interlock on a vehicle already sitting disarmed. The state machine owns
// what the vehicle does; this owns how much its inputs are believed.
//
// That division is what #52 has to keep when a failsafe gains somewhere to go:
// a descent or a return is sequencing, so it belongs in the main state
// machine, requested from here rather than run from here.
//
// Sentinel's own, not the blackboard's: nothing outside reads it. What leaves
// the board is kVehicleFailsafeFlagRcLoss, which the ESP32 turns into
// MAV_STATE_CRITICAL.
enum class RcLinkPhase : uint8_t {
  // Frames arriving, or none ever seen -- the bench case, where arming over
  // FcLink with no transmitter has to keep working.
  kUp = 0,
  // Silent past the timeout and still flying the last frame. A link that
  // comes back inside the guard is simply resumed; one that does not gets
  // the disarm on the way out, rather than in a phase of its own -- only one
  // branch reaches it, so there is nothing for a phase to join.
  kGuard,
  // Disarmed, holding arming shut until frames have streamed cleanly long
  // enough to be a link rather than a lucky packet.
  kRecovering,
};

// The board's safety authority. It owns the arm state end to end: the sources
// that ask for it, the interlocks that refuse it, the effects that make a
// transition safe, and the one variable that records the answer.
//
// Its writes are deliberately narrow: SharedState::armed_, the decision, and
// the failsafe flags that say why -- which leave the board so an operator is
// not left guessing at a disarm. It never drives motors and never sequences
// flight modes: it decides, the state machine sequences, and EscService
// enforces at the wire. Every condition it gains -- RC loss today, sensor
// staleness and battery next -- has to arrive as another reason to call the
// same transition, not as another thing it writes.
class Sentinel {
 public:
  // Thresholds only. Every one is a policy question the components that raise
  // the conditions are deliberately not asked -- a driver counts, this decides
  // how much counts as too much.
  struct Config {
    // Resolved by the generator from a share of the record rate, so a rate
    // change moves it rather than silently meaning something else.
    uint32_t imu_loss_threshold_samples;
    uint32_t imu_loss_window_us;
    // Windows in a row that must exceed the threshold. 1 removes the
    // hysteresis.
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
    // Receiver silence before the link counts as lost. CRSF has no failsafe
    // bit and ExpressLRS stops sending rather than flagging, so this is the
    // only detector there is. Starts the staged response; not a disarm.
    uint32_t rc_loss_timeout_us;
    // Flight on the pilot's last frame before the procedure runs. The knob
    // that makes a brief dropout survivable, and the window the aircraft
    // spends uncommanded.
    uint32_t failsafe_guard_us;
    // Clean RC required before arming is allowed again, so an intermittent
    // link cannot re-arm on a switch nobody moved.
    uint32_t failsafe_recovery_us;
    // Pack millivolts below which arming is refused; 0 disables. Ground-only
    // by design: in the air the same reading sags with throttle, and acting
    // on it there is #50's landing, not a cut.
    uint32_t arm_battery_min_mv;
    // Calibrated throttle above which arming is refused; 0 disables. A
    // backstop behind the handset's own idle condition, for the transmitter
    // that was never told to carry one.
    uint16_t arm_throttle_max_us;
  };

  // Every arm request on the board lands here -- the privileged command and
  // the RC switch alike. A refusal is announced from inside rather than
  // returned: the caller has no way to know which interlock said no, and two
  // callers each remembering to make a noise is how one of them stops.
  void RequestArm(bool armed);

  // Called from the main tick, never the control loop: PendSV is pended by the
  // sample interrupt, so a watchdog living there would fall silent in exactly
  // the failure it exists to catch.
  void Supervise(uint32_t now_us);

 private:
  friend class System;

  void Init(const Config &cfg, SharedState &blackboard, EscService &esc,
            RateController &rate_controller, Icm42688p &imu, FcLink &fc_link);

  // Weighs the driver's counters against the thresholds it deliberately does
  // not know, and answers per the arm state.
  void SuperviseImu(uint32_t now_us);
  // Watches the one failure a stop frame cannot answer: the DShot path
  // refusing every write.
  void SuperviseEscOutput(uint32_t now_us);
  void SuperviseTestThrottle(uint32_t now_us);
  void RecoverStalledImu(uint32_t now_us);
  // Halts when disarmed; armed, raises the failsafe flag and remembers the
  // code so the halt still happens once the aircraft is down.
  void RaiseImuFault(ErrorCode::Stm32 code);

  // Every refusal and the bench cut sound the same for now: the bitmask can
  // say which interlock said no, but nothing the pilot can hear distinguishes
  // them yet.
  void EmitWarning();

  // The RC half of Supervise: link loss, then the arm switch. Takes the
  // blockers raised outside it so the published set is written once, by the
  // half that also acts on it.
  void SuperviseRc(uint32_t now_us, uint16_t state_blockers);
  // Advances the link phase and, when the guard runs out, disarms. Returns
  // the phase it settled on, so the caller reads it once.
  RcLinkPhase StepRcLink(uint32_t now_us, bool link_ok);
  uint16_t BatteryBlocker() const;

  Config cfg_{};
  SharedState *blackboard_ = nullptr;
  EscService *esc_ = nullptr;
  RateController *rate_controller_ = nullptr;
  Icm42688p *imu_ = nullptr;
  FcLink *fc_link_ = nullptr;
  // Rate-limits recovery to one attempt per stall window, rather than every
  // main tick while the heartbeat stays frozen.
  uint32_t last_imu_recovery_us_ = 0;
  uint32_t last_loss_window_us_ = 0;
  uint32_t last_missed_samples_ = 0;
  uint32_t last_fault_window_us_ = 0;
  uint32_t last_path_faults_ = 0;
  uint32_t last_esc_window_us_ = 0;
  uint32_t last_esc_drops_ = 0;
  uint32_t high_loss_consec_ = 0;
  // 0 means no throttle is standing, so a stale stamp from a previous
  // session can never cut the next one on its first pass.
  uint32_t last_host_us_ = 0;
  uint32_t last_msp_requests_ = 0;
  // Refused until the first Supervise has computed a real set, so a request
  // that beats the supervisor to the first pass is answered by the interlock
  // rather than by a word nobody has written yet.
  uint16_t arm_blockers_ = kArmBlockNotIdle;
  RcLinkPhase rc_link_phase_ = RcLinkPhase::kUp;
  // Stamped on every phase change; only kGuard reads it back, to age itself
  // out against the guard.
  uint32_t rc_link_entered_us_ = 0;
  // Restarted by every frame that arrives while the link is judged bad, so
  // recovery measures a clean stretch rather than a single lucky frame.
  uint32_t rc_recovery_since_us_ = 0;
  // Set by every disarm, so a
  // switch left up cannot re-arm behind a GCS or failsafe disarm; cleared only
  // by seeing the switch outside its window. True at boot, so a board powered
  // up with the switch already on stays disarmed until it is cycled.
  bool arm_switch_blocked_ = true;
  // The switch earns its kill authority by being seen in-window once. Until
  // then it cannot disarm -- a bench fixture injecting channels it does not
  // drive reads zero on the arm slot, and that must not cut a test.
  bool arm_switch_authority_ = false;
  // Edge, so holding the switch up against an interlock is one refusal rather
  // than one per pass. A privileged request is its own event and announces
  // every time, which is why only the switch path carries this.
  uint16_t switch_refusal_announced_ = 0;
  bool imu_fault_latched_ = false;
  // Only read while latched, so its initial value never reaches a Panic.
  ErrorCode::Stm32 imu_fault_code_{};
  bool initialized_ = false;
};
