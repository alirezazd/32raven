// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi
//
// Ported from PX4 Autopilot, src/modules/commander/lm_fit.cpp, under the
// BSD-3-Clause terms reproduced in mag_fit.hpp. Three departures from the
// upstream: the sphere stage's first candidate cost subtracted the Z offset
// with the wrong sign, the ellipsoid stage summed its first candidate cost
// twice, and a step that made both candidates worse was still written back.

#include "mag_fit.hpp"

#include <cmath>
#include <utility>

namespace {

constexpr float kLmDamping = 10.0f;
constexpr uint8_t kMaxIterations = 100;
constexpr uint8_t kMinIterations = 10;
constexpr float kCostThreshold = 0.01f;
constexpr float kStepThreshold = 0.001f;

Eigen::Matrix3f SoftIron(const Eigen::Vector3f &diag,
                         const Eigen::Vector3f &offdiag) {
  Eigen::Matrix3f s;
  s << diag.x(), offdiag.x(), offdiag.y(),  //
      offdiag.x(), diag.y(), offdiag.z(),   //
      offdiag.y(), offdiag.z(), diag.z();
  return s;
}

// Root of the summed squared residual over the count, as upstream normalises
// it -- which is why the threshold on it is as small as it is.
float Residual(std::span<const float> x, std::span<const float> y,
           std::span<const float> z, const Eigen::Matrix3f &s,
           const Eigen::Vector3f &offset, float radius) {
  float sum = 0.0f;
  for (size_t k = 0; k < x.size(); ++k) {
    const Eigen::Vector3f d(x[k] - offset.x(), y[k] - offset.y(),
                            z[k] - offset.z());
    const float residual = radius - (s * d).norm();
    sum += residual * residual;
  }
  return std::sqrt(sum) / static_cast<float>(x.size());
}

// One damped step: solve (JTJ + lambda I) delta = JTFI by Gauss-Jordan
// elimination with row pivoting, or report a singular system. Written out
// rather than taken from Eigen's decompositions, whose fixed-size paths still
// carry an allocation the firmware's linker refuses.
template <int N>
bool Solve(const Eigen::Matrix<float, N, N> &jtj,
           const Eigen::Matrix<float, N, 1> &jtfi, float lambda,
           Eigen::Matrix<float, N, 1> &delta) {
  Eigen::Matrix<float, N, N> a = jtj;
  Eigen::Matrix<float, N, 1> b = jtfi;
  for (int i = 0; i < N; ++i) {
    a(i, i) += lambda;
  }
  for (int col = 0; col < N; ++col) {
    int pivot = col;
    for (int row = col + 1; row < N; ++row) {
      if (std::fabs(a(row, col)) > std::fabs(a(pivot, col))) {
        pivot = row;
      }
    }
    if (!(std::fabs(a(pivot, col)) > 1e-9f)) {
      return false;
    }
    if (pivot != col) {
      a.row(col).swap(a.row(pivot));
      std::swap(b(col), b(pivot));
    }
    const float inv = 1.0f / a(col, col);
    a.row(col) *= inv;
    b(col) *= inv;
    for (int row = 0; row < N; ++row) {
      if (row == col) {
        continue;
      }
      const float factor = a(row, col);
      if (factor != 0.0f) {
        a.row(row) -= factor * a.row(col);
        b(row) -= factor * b(col);
      }
    }
  }
  delta = b;
  return true;
}

}  // namespace

void MagFit::Start(Stage stage, const MagFitParams &seed) {
  stage_ = stage;
  params_ = seed;
  cost_ = 1e30f;
  damping_ = 1.0f;
  iteration_ = 0;
}

MagFit::Status MagFit::Step(std::span<const float> x, std::span<const float> y,
                            std::span<const float> z) {
  if (iteration_ >= kMaxIterations) {
    return Status::kFailed;
  }
  const bool improved = stage_ == Stage::kSphere
                            ? SphereIteration(x, y, z)
                            : EllipsoidIteration(x, y, z);
  const uint8_t i = iteration_;
  ++iteration_;
  if (improved && params_.radius > kMinRadius && params_.radius < kMaxRadius &&
      i > kMinIterations &&
      (cost_ < kCostThreshold || damping_ < kStepThreshold)) {
    return Status::kConverged;
  }
  return iteration_ >= kMaxIterations ? Status::kFailed : Status::kRunning;
}

bool MagFit::SphereIteration(std::span<const float> x, std::span<const float> y,
                             std::span<const float> z) {
  const Eigen::Matrix3f s = SoftIron(params_.diag, params_.offdiag);
  Eigen::Matrix4f jtj = Eigen::Matrix4f::Zero();
  Eigen::Vector4f jtfi = Eigen::Vector4f::Zero();

  for (size_t k = 0; k < x.size(); ++k) {
    const Eigen::Vector3f d(x[k] - params_.offset.x(),
                            y[k] - params_.offset.y(),
                            z[k] - params_.offset.z());
    const Eigen::Vector3f sd = s * d;
    const float length = sd.norm();
    const Eigen::Vector3f d_offset = (s * sd) / length;
    const Eigen::Vector4f jacobian(1.0f, d_offset.x(), d_offset.y(),
                                   d_offset.z());
    const float residual = params_.radius - length;
    jtj += jacobian * jacobian.transpose();
    jtfi += jacobian * residual;
  }

  Eigen::Vector4f delta1;
  Eigen::Vector4f delta2;
  if (!Solve<4>(jtj, jtfi, damping_, delta1) ||
      !Solve<4>(jtj, jtfi, damping_ / kLmDamping, delta2)) {
    return false;
  }
  const Eigen::Vector4f current(params_.radius, params_.offset.x(),
                                params_.offset.y(), params_.offset.z());
  Eigen::Vector4f candidate1 = current - delta1;
  const Eigen::Vector4f candidate2 = current - delta2;

  const float fit1 =
      Residual(x, y, z, s, candidate1.tail<3>(), candidate1(0));
  const float fit2 =
      Residual(x, y, z, s, candidate2.tail<3>(), candidate2(0));

  float fitness = cost_;
  if (fit1 > cost_ && fit2 > cost_) {
    damping_ *= kLmDamping;
    candidate1 = current;
  } else if (fit2 < cost_ && fit2 < fit1) {
    damping_ /= kLmDamping;
    candidate1 = candidate2;
    fitness = fit2;
  } else if (fit1 < cost_) {
    fitness = fit1;
  }

  if (!std::isfinite(fitness) || fitness > cost_) {
    return false;
  }
  cost_ = fitness;
  params_.radius = candidate1(0);
  params_.offset = candidate1.tail<3>();
  return true;
}

bool MagFit::EllipsoidIteration(std::span<const float> x,
                                std::span<const float> y,
                                std::span<const float> z) {
  using Vector9f = Eigen::Matrix<float, 9, 1>;
  using Matrix9f = Eigen::Matrix<float, 9, 9>;

  const Eigen::Matrix3f s = SoftIron(params_.diag, params_.offdiag);
  Matrix9f jtj = Matrix9f::Zero();
  Vector9f jtfi = Vector9f::Zero();

  for (size_t k = 0; k < x.size(); ++k) {
    const Eigen::Vector3f d(x[k] - params_.offset.x(),
                            y[k] - params_.offset.y(),
                            z[k] - params_.offset.z());
    const Eigen::Vector3f sd = s * d;
    const float length = sd.norm();
    const float residual = params_.radius - length;
    const Eigen::Vector3f d_offset = (s * sd) / length;
    Vector9f jacobian;
    jacobian << d_offset.x(), d_offset.y(), d_offset.z(),
        -(d.x() * sd.x()) / length, -(d.y() * sd.y()) / length,
        -(d.z() * sd.z()) / length,
        -((d.y() * sd.x()) + (d.x() * sd.y())) / length,
        -((d.z() * sd.x()) + (d.x() * sd.z())) / length,
        -((d.z() * sd.y()) + (d.y() * sd.z())) / length;
    jtj += jacobian * jacobian.transpose();
    jtfi += jacobian * residual;
  }

  Vector9f delta1;
  Vector9f delta2;
  if (!Solve<9>(jtj, jtfi, damping_, delta1) ||
      !Solve<9>(jtj, jtfi, damping_ / kLmDamping, delta2)) {
    return false;
  }
  Vector9f current;
  current << params_.offset, params_.diag, params_.offdiag;
  Vector9f candidate1 = current - delta1;
  const Vector9f candidate2 = current - delta2;

  const auto cost_of = [&](const Vector9f &p) {
    return Residual(x, y, z, SoftIron(p.segment<3>(3), p.segment<3>(6)),
                p.head<3>(), params_.radius);
  };
  const float fit1 = cost_of(candidate1);
  const float fit2 = cost_of(candidate2);

  float fitness = cost_;
  if (fit1 > cost_ && fit2 > cost_) {
    damping_ *= kLmDamping;
    candidate1 = current;
  } else if (fit2 < cost_ && fit2 < fit1) {
    damping_ /= kLmDamping;
    candidate1 = candidate2;
    fitness = fit2;
  } else if (fit1 < cost_) {
    fitness = fit1;
  }

  if (!std::isfinite(fitness) || fitness > cost_) {
    return false;
  }
  cost_ = fitness;
  params_.offset = candidate1.head<3>();
  params_.diag = candidate1.segment<3>(3);
  params_.offdiag = candidate1.segment<3>(6);
  return true;
}
