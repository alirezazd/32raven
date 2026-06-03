// Multirotor motor mixer — stateless thrust allocator for a quad-X frame.
//
// Maps body-frame torque demands (roll/pitch/yaw) and collective thrust to
// per-motor normalized thrust in [0, 1]; the caller (EscService / DShotCodec)
// scales to DShot units. Attitude-representation-agnostic: torques are
// 3-vectors regardless of the upstream parameterization.
//
// PX4 MC_AIRMODE=0 ("Disabled") sequential-desaturation port of
// ControlAllocationSequentialDesaturation (mixAirmodeDisabled + mixYaw),
// reduced to the multirotor path; PX4's pseudo-inverse / hover-trim machinery
// is elided because motors are already in normalized [idle, 1]. Algorithm
// detail lives in multirotor_mixer.cpp. Key contract:
//   - Collective thrust is NEVER raised to unsaturate motors. If torque can't
//     fit the band, roll/pitch/yaw are scaled down per-axis instead — the only
//     airmode mode that honours commanded descent (gives up attitude authority
//     rather than refusing to descend).
//   - Yaw gets a temporary +15 % upper-limit expansion during its desat pass
//     (PX4 MINIMUM_YAW_MARGIN), preserving yaw authority to the saturation
//     boundary; thrust is then dropped uniformly back into the real band.
//   - `applied_torque` is {R,P,Y} projected back from the final motor vector;
//     RateController::CommitTorque drains integrators by the commanded-vs-
//     applied gap. (Projection math: see MixOutput / Mix().)
//   - Disarmed -> all outputs 0.
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

// Mix() output. `applied_torque` {τR,τP,τY}_eff is the body torque actually
// projected through the mixer after saturation rescale: equals commanded in
// the linear region, scale×commanded after rescale, {0,0,0} when disarmed.
// Caller feeds it into the rate PID's back-calculation anti-windup.
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

// Instance member of System (not a singleton) — pure logic, no hardware.
class Mixer {
 public:
  struct Config {
    // Armed motor floor; outputs >= idle. [0, 1]. Set just above stall RPM —
    // crossing it stops the motor and recovery is not free.
    float idle;
  };

  Mixer() = default;

  // Panics on invalid config; called once by System::InitComponent.
  void Init(const Config &cfg);

  // Runtime config swap for tuning; preserves arm state. Panics if invalid.
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
