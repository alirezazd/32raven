// math::Quaternion — unit quaternion attitude representation for the
// flight stack. Hamilton convention (ij = k, jk = i, ki = j). World ↔
// body frames; the convention adopted across the codebase is that q
// rotates a vector from BODY frame to WORLD frame:
//
//     v_world = q ⊗ v_body ⊗ q*
//
// Initialized to identity (1, 0, 0, 0). Stays on the unit sphere by
// renormalizing after each integration step.
//
// Cost on Cortex-M4 with FPU: every operation here is ~10-30 VFP
// instructions, single-precision. No allocations, no virtuals.

#pragma once

#include <cmath>

#include "math/vec3.hpp"

namespace math {

struct Quaternion {
  // Scalar-first storage: (w, x, y, z) ≡ w + x·i + y·j + z·k.
  float w = 1.0f;
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;

  constexpr Quaternion() = default;
  constexpr Quaternion(float ww, float xx, float yy, float zz)
      : w(ww), x(xx), y(yy), z(zz) {}

  constexpr Vec3 Vec() const { return {x, y, z}; }
};

constexpr Quaternion Identity() { return Quaternion{1.0f, 0.0f, 0.0f, 0.0f}; }

// ── algebraic ───────────────────────────────────────────────────────

// Hamilton product q1 ⊗ q2. Reads "apply q2 first, then q1" when
// rotating vectors via v' = q ⊗ v ⊗ q*. Not commutative.
constexpr Quaternion Compose(const Quaternion &q1, const Quaternion &q2) {
  return Quaternion{
      q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z,
      q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y,
      q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x,
      q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w,
  };
}

// For a unit quaternion this is the inverse.
constexpr Quaternion Conjugate(const Quaternion &q) {
  return Quaternion{q.w, -q.x, -q.y, -q.z};
}

inline float NormSquared(const Quaternion &q) {
  return q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
}

inline float Norm(const Quaternion &q) { return std::sqrt(NormSquared(q)); }

// Rescale to unit norm. Falls back to identity if the input is
// degenerate (norm too small) — better than producing NaN.
inline Quaternion Normalize(const Quaternion &q) {
  const float n2 = NormSquared(q);
  if (n2 < 1e-30f) return Identity();
  const float inv = 1.0f / std::sqrt(n2);
  return Quaternion{q.w * inv, q.x * inv, q.y * inv, q.z * inv};
}

// ── action on a vector ──────────────────────────────────────────────

// Rotate v by q. Equivalent to taking the vector part of q ⊗ (0,v) ⊗ q*
// but written as the cheaper rearrangement (≈ 15 mul + 9 add).
inline Vec3 Rotate(const Quaternion &q, const Vec3 &v) {
  const Vec3 qv{q.x, q.y, q.z};
  const Vec3 t = 2.0f * Cross(qv, v);
  return v + q.w * t + Cross(qv, t);
}

// ── kinematics: integrate body angular rate into attitude ───────────

// First-order Euler step of dq/dt = (1/2) q ⊗ (0, ω_b). Renormalizes
// to absorb the small linearization drift. dt_s is in SECONDS.
//
// At 1 kHz tick / dt = 1 ms, the first-order error is < 1e-6 per step
// for body rates up to a few rad/s. Plenty for a rate-loop integrator.
// Sub-loops that need higher accuracy (IMU pre-integration across 8 kHz
// samples) can call this repeatedly with smaller dt.
inline Quaternion IntegrateBodyRate(const Quaternion &q, const Vec3 &omega_b,
                                    float dt_s) {
  const float half_dt = 0.5f * dt_s;
  const Quaternion dq_dt{
      -half_dt * (q.x * omega_b.x + q.y * omega_b.y + q.z * omega_b.z),
      half_dt * (q.w * omega_b.x + q.y * omega_b.z - q.z * omega_b.y),
      half_dt * (q.w * omega_b.y - q.x * omega_b.z + q.z * omega_b.x),
      half_dt * (q.w * omega_b.z + q.x * omega_b.y - q.y * omega_b.x),
  };
  return Normalize(Quaternion{
      q.w + dq_dt.w,
      q.x + dq_dt.x,
      q.y + dq_dt.y,
      q.z + dq_dt.z,
  });
}

// ── attitude error → small-angle rotation vector ────────────────────

// Given a setpoint attitude and a measured attitude (both unit
// quaternions in the same world↔body convention), return the
// rotation vector (in radians per axis) that would rotate the
// measured attitude into the setpoint.
//
// For a small error this is twice the vector part of
// (q_sp ⊗ q_m⁻¹). The sign is chosen so we always take the short
// path around the unit sphere — q and -q represent the same
// rotation, but we want the rotation vector with magnitude < π.
//
// This is the function Phase B's attitude controller calls each tick
// to convert quaternion attitude error into an ω_sp vector (in rad/s)
// for the rate controller.
inline Vec3 ErrorRotationVector(const Quaternion &q_setpoint,
                                const Quaternion &q_measured) {
  const Quaternion q_err = Compose(q_setpoint, Conjugate(q_measured));
  // Short-path: if scalar part is negative, q_err and -q_err describe
  // the same rotation but the latter has smaller |vec| → use it.
  const float s = (q_err.w < 0.0f) ? -1.0f : 1.0f;
  return Vec3{2.0f * s * q_err.x, 2.0f * s * q_err.y, 2.0f * s * q_err.z};
}

// ── axis-angle helper (mostly for tests + initial-attitude setup) ───

// Build a quaternion from an axis (must be unit) and an angle in
// radians. Used in tests to construct known rotations; not on the
// fast path.
inline Quaternion FromAxisAngle(const Vec3 &axis_unit, float angle_rad) {
  const float half = 0.5f * angle_rad;
  const float s = std::sin(half);
  return Quaternion{std::cos(half), axis_unit.x * s, axis_unit.y * s,
                    axis_unit.z * s};
}

}  // namespace math
