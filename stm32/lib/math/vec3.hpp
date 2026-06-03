// math::Vec3 — 3-float vector. Single-precision to match the F4 FPU.

#pragma once

#include <cmath>

namespace math {

struct Vec3 {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;

  constexpr Vec3() = default;
  constexpr Vec3(float xx, float yy, float zz) : x(xx), y(yy), z(zz) {}
};

// ── component-wise ──────────────────────────────────────────────────

constexpr Vec3 operator+(const Vec3 &a, const Vec3 &b) {
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}

constexpr Vec3 operator-(const Vec3 &a, const Vec3 &b) {
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}

constexpr Vec3 operator-(const Vec3 &a) { return {-a.x, -a.y, -a.z}; }

constexpr Vec3 operator*(const Vec3 &a, float s) {
  return {a.x * s, a.y * s, a.z * s};
}
constexpr Vec3 operator*(float s, const Vec3 &a) { return a * s; }

constexpr Vec3 operator/(const Vec3 &a, float s) {
  return {a.x / s, a.y / s, a.z / s};
}

// ── inner products ──────────────────────────────────────────────────

constexpr float Dot(const Vec3 &a, const Vec3 &b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

constexpr Vec3 Cross(const Vec3 &a, const Vec3 &b) {
  return {
      a.y * b.z - a.z * b.y,
      a.z * b.x - a.x * b.z,
      a.x * b.y - a.y * b.x,
  };
}

// ── norm ────────────────────────────────────────────────────────────

inline float LengthSquared(const Vec3 &a) { return Dot(a, a); }
inline float Length(const Vec3 &a) { return std::sqrt(LengthSquared(a)); }

// Returns zero vector when too small to divide; avoids NaN from div-by-zero.
inline Vec3 Normalize(const Vec3 &a) {
  const float n2 = LengthSquared(a);
  if (n2 < 1e-30f) return {0.0f, 0.0f, 0.0f};
  return a * (1.0f / std::sqrt(n2));
}

}  // namespace math
