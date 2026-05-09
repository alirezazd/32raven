// Multirotor motor mixer — pure header-only library, no platform deps.
//
// Inputs:  body-frame torque demands (roll, pitch, yaw) in [-1, +1] and
//          collective thrust in [0, +1]. Representation-agnostic with
//          respect to attitude (Euler vs quaternion is an upstream choice).
//
// Output:  per-motor normalized thrust in [idle, 1]. The caller maps to
//          DShot units (typically [48, 2047]) via DShotCodec / EscService.
//
// Algorithm:
//   1. Matrix multiply: 4x3 QuadX factors × (roll, pitch, yaw) → axis demand
//      per motor.
//   2. Saturation rescale (Betaflight `motorMixRange` style): if the spread
//      across motors exceeds 1.0, scale all axis demands down so they fit.
//      This preserves *relative* torque across axes at the cost of
//      *absolute* magnitude — the craft does what you asked, just less.
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

// Per-motor normalized thrust output, [0, 1]. The caller scales to DShot.
using MotorThrust = std::array<float, 4>;

// What the upstream layer (RC passthrough day-0 / rate controller day-N)
// hands the mixer. Always body-frame, always normalized; quaternion choice
// upstream does not affect this struct.
struct Inputs {
    float roll_torque;    // [-1, +1]   moment around body X (right wing down = +)
    float pitch_torque;   // [-1, +1]               around body Y (nose up = +)
    float yaw_torque;     // [-1, +1]               around body Z (nose right = +)
    float thrust;         // [ 0, +1]   collective along body -Z
};

struct Config {
    float idle = 0.0f;    // motor floor when armed; mixer outputs >= idle
    bool armed = false;   // when false, all outputs are forced to 0
};

// QuadX mix matrix.
// Rows = motors (M1..M4), cols = (roll, pitch, yaw) factors.
// Throttle column is implicit +1 for every motor.
struct QuadX {
    static constexpr float kFactors[4][3] = {
        // R    P    Y
        { -1, +1, -1 },  // M1 front-right (CCW)
        { -1, -1, +1 },  // M2 back-right  (CW)
        { +1, -1, -1 },  // M3 back-left   (CCW)
        { +1, +1, +1 },  // M4 front-left  (CW)
    };
};

class Mixer {
public:
    constexpr Mixer() = default;
    explicit constexpr Mixer(const Config& cfg) : cfg_(cfg) {}

    void SetArmed(bool armed) { cfg_.armed = armed; }
    void SetIdle(float idle) { cfg_.idle = Clamp01(idle); }
    bool IsArmed() const { return cfg_.armed; }
    float Idle() const { return cfg_.idle; }

    // Compute motor commands from torque demands + collective thrust.
    // Stateless: pure function of (cfg_, in).
    MotorThrust Mix(const Inputs& in) const {
        if (!cfg_.armed) {
            return MotorThrust{0.0f, 0.0f, 0.0f, 0.0f};
        }

        // Step 1: matrix multiply → axis demand per motor.
        float axis[4];
        for (int i = 0; i < 4; ++i) {
            axis[i] = QuadX::kFactors[i][0] * in.roll_torque
                    + QuadX::kFactors[i][1] * in.pitch_torque
                    + QuadX::kFactors[i][2] * in.yaw_torque;
        }

        // Step 2: Betaflight-style saturation rescale.
        float lo = axis[0], hi = axis[0];
        for (int i = 1; i < 4; ++i) {
            if (axis[i] < lo) lo = axis[i];
            if (axis[i] > hi) hi = axis[i];
        }
        const float range = hi - lo;
        if (range > 1.0f) {
            const float scale = 1.0f / range;
            for (int i = 0; i < 4; ++i) {
                axis[i] *= scale;
            }
        }

        // Step 3: add throttle, clamp to [idle, 1]. No airmode (v1).
        MotorThrust out;
        for (int i = 0; i < 4; ++i) {
            out[i] = Clamp(axis[i] + in.thrust, cfg_.idle, 1.0f);
        }
        return out;
    }

private:
    Config cfg_{};

    static constexpr float Clamp(float v, float lo, float hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }
    static constexpr float Clamp01(float v) { return Clamp(v, 0.0f, 1.0f); }
};

}  // namespace multirotor_mixer
