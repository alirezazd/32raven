#include "multirotor_mixer.hpp"

#include <algorithm>
#include <cmath>

#include "error_code.hpp"
#include "panic.hpp"

// PX4 sequential-desaturation port (MC_AIRMODE=0 / "Disabled" variant).
//
// Faithful port of
//   PX4-Autopilot/src/lib/control_allocation/control_allocation/
//     ControlAllocationSequentialDesaturation.{hpp,cpp}
// reduced to the multirotor-only path (mixAirmodeDisabled + mixYaw), with
// PX4's pseudo-inverse / trim machinery elided because this codebase already
// represents motors in the normalized [idle, 1] band — there is no hover
// trim to subtract.
//
// The "Disabled" airmode is the only variant that lets the drone honour a
// commanded descent: it never lifts collective thrust to unsaturate motors.
// When pilot stick is low and torque demand would otherwise saturate the
// low end of the motor band, the algorithm scales the torque axes down
// (giving up attitude authority) instead of raising thrust. This trade is
// exactly what the SIL workflow needs: descent always works; authority
// degrades smoothly toward zero as the pilot commits to coming down.

namespace multirotor_mixer {

namespace {

// PX4 constant — see ControlAllocationSequentialDesaturation.hpp:63.
// Yaw is given a temporary +15 % expansion on the upper limit so the
// drone retains some yaw authority even when motors are at saturation;
// the post-yaw thrust pass (decrease-only) clips back into the real band.
constexpr float kMinimumYawMargin = 0.15f;

// PX4 effectiveness floor below which an actuator is excluded from
// desaturation gain computation. See ControlAllocationSequentialDesat.cpp:96.
constexpr float kEffectivenessFloor = 0.2f;

bool IsConfigValid(const Mixer::Config &cfg) {
  return cfg.idle >= 0.0f && cfg.idle <= 1.0f;
}

// Port of ControlAllocationSequentialDesaturation::computeDesaturationGain.
//
// Returns the gain k by which `desat` should be added to `sp` to drive the
// worst-saturated actuator back toward the [min_lim, max_lim] band. The
// k_min/k_max accumulation captures both ends of saturation; summing them
// puts the result roughly at the centre of the residual saturation.
//
// Actuators whose desat-direction magnitude is below kEffectivenessFloor
// are skipped — they're too weakly coupled to the desat axis to be worth
// touching (using them would inflate gain magnitudes unhelpfully).
float ComputeDesaturationGain(const float desat[4], const float sp[4],
                              float min_lim, float max_lim) {
  float k_min = 0.0f;
  float k_max = 0.0f;
  for (int i = 0; i < 4; ++i) {
    if (std::fabs(desat[i]) < kEffectivenessFloor) continue;
    if (sp[i] < min_lim) {
      const float k = (min_lim - sp[i]) / desat[i];
      if (k < k_min) k_min = k;
      if (k > k_max) k_max = k;
    }
    if (sp[i] > max_lim) {
      const float k = (max_lim - sp[i]) / desat[i];
      if (k < k_min) k_min = k;
      if (k > k_max) k_max = k;
    }
  }
  return k_min + k_max;
}

// Port of ControlAllocationSequentialDesaturation::desaturateActuators.
//
// Apply the desaturation gain along `desat` in two passes — full, then
// half of the residual — which refines the fit because the first
// k_min + k_max sum overshoots when both ends of the band are saturated.
//
// `block_motor_lift` blocks the "raise motors" direction. PX4 names this
// `increase_only` but uses a thrust-axis convention where the desat
// vector has negative sign per motor; we use thrust_z[i] = +1 (motor
// command rises with positive thrust input), so the equivalent guard is
// `gain > 0`. The behaviour matches PX4 line-for-line: motors must not
// be lifted to unsaturate.
void DesaturateActuators(float sp[4], const float desat[4], float min_lim,
                         float max_lim, bool block_motor_lift = false) {
  float gain = ComputeDesaturationGain(desat, sp, min_lim, max_lim);
  if (block_motor_lift && gain > 0.0f) return;
  for (int i = 0; i < 4; ++i) sp[i] += gain * desat[i];
  gain = 0.5f * ComputeDesaturationGain(desat, sp, min_lim, max_lim);
  for (int i = 0; i < 4; ++i) sp[i] += gain * desat[i];
}

}  // namespace

void Mixer::Init(const Config &cfg) {
  if (!IsConfigValid(cfg)) {
    Panic(ErrorCode::Stm32::kMixerInvalidConfig);
  }
  cfg_ = cfg;
  armed_ = false;
}

void Mixer::SetConfig(const Config &cfg) {
  if (!IsConfigValid(cfg)) {
    Panic(ErrorCode::Stm32::kMixerInvalidConfig);
  }
  cfg_ = cfg;
}

// PX4 mixAirmodeDisabled + mixYaw, in order:
//
//   1. Build pre-yaw actuator setpoint:  sp[i] = R·mix_R + P·mix_P + T·mix_T.
//   2. Desaturate thrust direction, BLOCKING motor lift — drops the whole
//      band uniformly if any motor is above max; does nothing if motors are
//      below min (PX4's "never increase thrust" rule).
//   3. Desaturate roll direction (bidirectional) — shrinks roll authority
//      to fit the band.
//   4. Same for pitch.
//   5. Add yaw to the actuator setpoint.
//   6. Desaturate yaw with the upper limit temporarily expanded by +15 %
//      (kMinimumYawMargin) — preserves yaw authority right up to the
//      saturation point.
//   7. Desaturate thrust again, BLOCKING motor lift — restores the real
//      upper limit by dropping motors uniformly if needed.
//   8. Final clamp to [idle, 1] for FP-noise safety.
//
// applied_torque is the orthogonal projection of the resulting motor
// vector onto each torque axis (factor_axis · motors / 4 for the QuadX
// factors whose columns are mutually orthogonal with squared-norm 4).
// The rate PID's back-calc anti-windup feeds on the gap between
// commanded and applied per axis.
MixOutput Mixer::Mix(const Inputs &in) const {
  if (!armed_) {
    return MixOutput{
        .motors = {0.0f, 0.0f, 0.0f, 0.0f},
        .applied_torque = {0.0f, 0.0f, 0.0f},
    };
  }

  // Desaturation direction vectors. For QuadX the roll/pitch/yaw vectors
  // come straight from the mix matrix columns; thrust is +1 per motor
  // because every motor produces equal upward thrust.
  float roll[4];
  float pitch[4];
  float yaw[4];
  float thrust_z[4];
  for (int i = 0; i < 4; ++i) {
    roll[i] = QuadX::kFactors[i][0];
    pitch[i] = QuadX::kFactors[i][1];
    yaw[i] = QuadX::kFactors[i][2];
    thrust_z[i] = 1.0f;
  }

  // Step 1: pre-yaw setpoint (mix R, P, T into per-motor command).
  float sp[4];
  for (int i = 0; i < 4; ++i) {
    sp[i] = roll[i] * in.roll_torque + pitch[i] * in.pitch_torque +
            thrust_z[i] * in.thrust;
  }

  const float min_lim = cfg_.idle;
  const float max_lim = 1.0f;

  // Steps 2-4: PX4 mixAirmodeDisabled.
  DesaturateActuators(sp, thrust_z, min_lim, max_lim, /*block_motor_lift=*/true);
  DesaturateActuators(sp, roll, min_lim, max_lim);
  DesaturateActuators(sp, pitch, min_lim, max_lim);

  // Step 5: mix yaw.
  for (int i = 0; i < 4; ++i) {
    sp[i] += yaw[i] * in.yaw_torque;
  }

  // Step 6: yaw desat with relaxed upper limit (PX4 mixYaw).
  const float max_expanded = max_lim + (max_lim - min_lim) * kMinimumYawMargin;
  DesaturateActuators(sp, yaw, min_lim, max_expanded);

  // Step 7: thrust desat with the real upper limit, decrease-only.
  DesaturateActuators(sp, thrust_z, min_lim, max_lim, /*block_motor_lift=*/true);

  // Step 8: emit. Final clamp is a paranoid safety; the algorithm leaves
  // motors inside [idle, 1] by construction except for the tiny
  // kMinimumYawMargin overshoot that step 7 was supposed to absorb.
  MixOutput out;
  for (int i = 0; i < 4; ++i) {
    out.motors[i] = std::clamp(sp[i], min_lim, max_lim);
  }

  // applied_torque: orthogonal projection of the final motor vector onto
  // each torque axis. QuadX factor columns are mutually orthogonal with
  // ||·||² = 4, and the thrust direction (1,1,1,1) is orthogonal to each
  // of them, so this projection cleanly recovers the actually-delivered
  // R/P/Y components for back-calc anti-windup.
  float r_applied = 0.0f;
  float p_applied = 0.0f;
  float y_applied = 0.0f;
  for (int i = 0; i < 4; ++i) {
    r_applied += QuadX::kFactors[i][0] * out.motors[i];
    p_applied += QuadX::kFactors[i][1] * out.motors[i];
    y_applied += QuadX::kFactors[i][2] * out.motors[i];
  }
  out.applied_torque = {r_applied * 0.25f, p_applied * 0.25f, y_applied * 0.25f};
  return out;
}

}  // namespace multirotor_mixer
