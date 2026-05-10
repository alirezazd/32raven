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
// Algorithm:
//   1. Matrix multiply: 4x3 QuadX factors × (roll, pitch, yaw).
//   2. Saturation rescale (Betaflight motorMixRange): if motor spread > 1,
//      scale axis demands down so they fit. Preserves *relative* torque
//      across axes; sacrifices absolute magnitude.
//   3. Add throttle, clamp each motor to [idle, 1]. v1 has no airmode, so
//      attitude authority drops at low throttle (acceptable for hover-only
//      first flight; v2 adds airmode for inverted / aggressive use).
//   4. If disarmed, all outputs = 0.
//
// Frame convention (looking down from above, body Z points down):
//   M1 = front-right (CCW prop)
//   M2 = back-right  (CW)
//   M3 = back-left   (CCW)
//   M4 = front-left  (CW)
// Matches Betaflight `mixerQuadX` numbering.

#pragma once

#include <array>
#include <cstdint>

namespace multirotor_mixer {

// Per-motor normalized thrust output, [0, 1]. Caller scales to DShot units.
using MotorThrust = std::array<float, 4>;

// Inputs consumed by the mixer. Body-frame, normalized — quaternion choice
// upstream does not affect this struct.
struct Inputs {
  float roll_torque;  // [-1, +1]   moment around body X (right wing down = +)
  float
      pitch_torque;  // [-1, +1]               around body Y (nose up      = +)
  float yaw_torque;  // [-1, +1]               around body Z (nose right   = +)
  float thrust;      // [ 0, +1]   collective along body -Z
};

// QuadX mix matrix. Rows = motors (M1..M4), cols = (roll, pitch, yaw).
// Throttle column is implicit +1 for every motor.
struct QuadX {
  static constexpr float kFactors[4][3] = {
      // R    P    Y
      {-1, +1, -1},  // M1 front-right (CCW)
      {-1, -1, +1},  // M2 back-right  (CW)
      {+1, -1, -1},  // M3 back-left   (CCW)
      {+1, +1, +1},  // M4 front-left  (CW)
  };
};

// Owned as an instance member of System (matches CrsfLinkService / EscService
// pattern), not a singleton — pure logic, no hardware ownership.
class Mixer {
 public:
  // Init-time configuration. Runtime state (arm flag) lives separately.
  struct Config {
    float idle;  // motor floor when armed; outputs >= idle. [0, 1].
  };

  Mixer() = default;

  // Init. Panics on invalid config; called once by System::InitComponent.
  void Init(const Config &cfg);

  // Runtime controls.
  void SetArmed(bool armed) { armed_ = armed; }
  bool IsArmed() const { return armed_; }
  float Idle() const { return cfg_.idle; }

  // Compute motor commands. Disarmed -> all zeros (the safety gate).
  MotorThrust Mix(const Inputs &in) const;

 private:
  Config cfg_{};
  bool armed_ = false;
};

}  // namespace multirotor_mixer
