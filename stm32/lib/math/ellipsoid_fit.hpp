// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include <Eigen/Core>
#include <cstdint>
#include <span>

namespace math {

// A sphere of `radius`, displaced by `offset` and distorted by a symmetric
// matrix. Corrected point = S * (raw - offset), S being
//   [ diag.x     offdiag.x  offdiag.y ]
//   [ offdiag.x  diag.y     offdiag.z ]
//   [ offdiag.y  offdiag.z  diag.z    ]
struct EllipsoidFitParams {
  Eigen::Vector3f offset = Eigen::Vector3f::Zero();
  Eigen::Vector3f diag = Eigen::Vector3f::Ones();
  Eigen::Vector3f offdiag = Eigen::Vector3f::Zero();
  float radius = 1.0f;
};

// Least squares on (|S (x - offset)| - radius)^2, one iteration per Step. The
// sphere stage fits offset and radius, the ellipsoid stage offset and S.
class EllipsoidFit {
 public:
  enum class Stage : uint8_t { kSphere, kEllipsoid };
  enum class Status : uint8_t { kRunning, kConverged, kFailed };

  // A fit converges only with its radius strictly inside these.
  struct RadiusBounds {
    float min;
    float max;
  };

  void Start(Stage stage, const EllipsoidFitParams &seed, RadiusBounds bounds);
  Status Step(std::span<const float> x, std::span<const float> y,
              std::span<const float> z);

  const EllipsoidFitParams &Params() const { return params_; }
  // RMS distance of the points from the fitted surface.
  float Cost() const { return cost_; }
  uint8_t Iteration() const { return iteration_; }

 private:
  using Vector9 = Eigen::Matrix<float, 9, 1>;
  using Matrix9 = Eigen::Matrix<float, 9, 9>;

  // J^T J, J^T e and the squared residuals; the sphere uses the leading 4x4.
  struct Linearization {
    Matrix9 jtj = Matrix9::Zero();
    Vector9 jte = Vector9::Zero();
    float sum_sq = 0.0f;
    bool valid = false;
  };

  int Unknowns() const;
  Vector9 Pack(const EllipsoidFitParams &p) const;
  EllipsoidFitParams Unpack(const Vector9 &theta) const;
  Linearization Linearize(const EllipsoidFitParams &p, std::span<const float> x,
                          std::span<const float> y,
                          std::span<const float> z) const;
  Status Finish() const;

  EllipsoidFitParams params_{};
  RadiusBounds bounds_{};
  Stage stage_ = Stage::kSphere;
  Linearization at_{};  // at params_; invalid until the first Step
  float damping_ = 0.0f;
  float damping_growth_ = 2.0f;
  float cost_ = 0.0f;
  uint8_t iteration_ = 0;
};

}  // namespace math
