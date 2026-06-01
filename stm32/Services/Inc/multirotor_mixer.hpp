// Multirotor motor mixer — stateless thrust allocator for a quad-X frame.
//
// Maps body-frame torque demands (roll/pitch/yaw) and collective thrust to
// per-motor normalized thrust in [0, 1]. The caller (EscService /
// DShotCodec) maps to DShot units (typically [48, 2047]).
//
// Representation-agnostic w.r.t. attitude — torques are 3-vectors regardless
// of whether the upstream layer uses Euler, quaternions, rotation matrices,
// or any other parameterization. The mixer's contract never changes.
//
// PX4 MC_AIRMODE=0 ("Disabled") sequential-desaturation port:
//   - Faithful port of PX4-Autopilot's
//     ControlAllocationSequentialDesaturation (mixAirmodeDisabled + mixYaw),
//     reduced to the multirotor path with PX4's pseudo-inverse / trim
//     machinery elided (this codebase represents motors directly in
//     normalized [idle, 1] with no hover trim).
//   - Pilot's collective thrust is NEVER raised by the mixer to unsaturate
//     motors. If torque demand can't fit in the band around the pilot's
//     thrust, roll/pitch/yaw are scaled down (per-axis, sequentially)
//     until motors fit — the drone gives up attitude authority rather
//     than refusing to descend. This is the only PX4 airmode mode that
//     honours commanded descent.
//   - Yaw is given a temporary +15 % expansion on the upper limit during
//     its desat pass (PX4's MINIMUM_YAW_MARGIN), then thrust is dropped
//     uniformly to bring motors back into the real band. Preserves yaw
//     authority right up to the saturation boundary.
//   - `applied_torque` reports {R, P, Y}_applied projected back from the
//     final motor vector (factor_axis · motors / 4 — exact for QuadX
//     because the factor columns are mutually orthogonal with ‖·‖² = 4).
//     RateController::CommitTorque drains each integrator by the gap
//     between commanded and actually delivered.
//
// Algorithm (full detail in multirotor_mixer.cpp):
//   1. Build pre-yaw setpoint sp[i] = R·factor[i][R] + P·factor[i][P]
//                                   + T (T is the collective per motor).
//   2. Desaturate along thrust direction, BLOCK MOTOR LIFT — drops the
//      whole band uniformly if any motor is above max; does nothing if
//      motors are below min.
//   3. Desaturate along roll direction (bidirectional).
//   4. Same for pitch.
//   5. Add yaw: sp[i] += Y · factor[i][Y].
//   6. Desaturate yaw with upper limit expanded by 15 %.
//   7. Desaturate thrust again with real upper limit, block motor lift.
//   8. Final clamp to [idle, 1] (FP safety only).
//   If disarmed, all outputs = 0.
//
// Frame convention (looking down from above, body Z points down):
//   M1 = front-right (CCW prop)
//   M2 = back-right  (CW)
//   M3 = back-left   (CCW)
//   M4 = front-left  (CW)
// Matches Betaflight `mixerQuadX` numbering.

#pragma once

#include <array>

namespace multirotor_mixer {

// Per-motor normalized thrust output, [0, 1]. Caller scales to DShot units.
using MotorThrust = std::array<float, 4>;

// Mix() output bundle: motor commands AND the per-axis body torque that
// was actually projected through the mixer after saturation rescale.
//
//   `motors`         — per-motor normalized thrust in [0, 1].
//   `applied_torque` — {τR_eff, τP_eff, τY_eff}. Equals the commanded
//                      torque in the linear region; equals
//                      `scale × commanded` after the saturation rescale;
//                      equals {0, 0, 0} when disarmed.
//
// Returned by Mix() so the caller can feed `applied_torque` into the
// PID's back-calculation anti-windup (CommitIntegrator / CommitTorque).
struct MixOutput {
  MotorThrust motors;
  std::array<float, 3> applied_torque;  // {roll, pitch, yaw}
};

// Inputs consumed by the mixer. Body-frame NED, normalized.
// All signs follow the aerospace right-hand rule about NED body axes
// (X = forward, Y = right, Z = down):
//   +roll_torque  → right wing down (rotates +Y_right toward +Z_down)
//   +pitch_torque → nose down       (rotates +X_fwd  toward +Z_down)
//   +yaw_torque   → nose right      (rotates +X_fwd  toward +Y_right)
struct Inputs {
  float roll_torque;   // [-1, +1]  body X — right wing down = +
  float pitch_torque;  // [-1, +1]  body Y — nose down       = +
  float yaw_torque;    // [-1, +1]  body Z — nose right      = +
  float thrust;        // [ 0, +1]  collective along body -Z (up)
};

// QuadX mix matrix. Rows = motors (M1..M4), cols = (roll, pitch, yaw).
// Throttle column is implicit +1 for every motor.
//
// Sign convention (all NED):
//   Pitch +1 (nose down): front motors (M1, M4) slow; back motors
//     (M2, M3) speed up — tail lifts, nose dips.
//   Yaw  +1 (nose right): CCW props (M1, M3) speed up; CW props
//     (M2, M4) slow down. Faster CCW prop → more CW reaction torque
//     on the body (Newton's 3rd) → body yaws right.
//   Roll +1 (right wing down): right motors (M1, M2) slow; left
//     motors (M3, M4) speed up — left side lifts, right side dips.
struct QuadX {
  static constexpr float kFactors[4][3] = {
      // R    P    Y
      {-1, +1, +1},  // M1 front-right (CCW prop)
      {-1, -1, -1},  // M2 back-right  (CW  prop)
      {+1, -1, +1},  // M3 back-left   (CCW prop)
      {+1, +1, -1},  // M4 front-left  (CW  prop)
  };
};

// Owned as an instance member of System (matches CrsfLinkService / EscService
// pattern), not a singleton — pure logic, no hardware ownership.
class Mixer {
 public:
  // Init-time configuration. Runtime state (arm flag) lives separately.
  struct Config {
    // Motor floor when armed; outputs >= idle. [0, 1]. Set just above
    // the motor's stall RPM — crossing it stops the motor and recovery
    // is not free.
    float idle;
  };

  Mixer() = default;

  // Init. Panics on invalid config; called once by System::InitComponent.
  void Init(const Config &cfg);

  // Runtime config swap; preserves arm state. Use for tuning during a
  // SIL session without restarting the mixer. Panics on invalid config.
  void SetConfig(const Config &cfg);

  const Config &GetConfig() const { return cfg_; }

  // Runtime controls.
  void SetArmed(bool armed) { armed_ = armed; }
  bool IsArmed() const { return armed_; }

  // Compute motor commands AND back-projected per-axis applied torque.
  // Disarmed -> motors all zero, applied_torque all zero (the safety gate).
  MixOutput Mix(const Inputs &in) const;

 private:
  Config cfg_{};
  bool armed_ = false;
};

}  // namespace multirotor_mixer
