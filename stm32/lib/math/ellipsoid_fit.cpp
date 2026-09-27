// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi
//
// Levenberg-Marquardt per Madsen, Nielsen and Tingleff (DTU, 2004), algorithm
// 3.16; the sphere starts from Kasa's algebraic fit (IEEE TIM, 1976).

#include "math/ellipsoid_fit.hpp"

#include <algorithm>
#include <cmath>
#include <optional>

namespace math {

namespace {

using Vector9 = Eigen::Matrix<float, 9, 1>;
using Matrix9 = Eigen::Matrix<float, 9, 9>;

// The damping starts at this share of the largest diagonal term of J^T J.
constexpr float kInitialDamping = 1e-3f;
// Stop once the mean gradient per point or the relative step is below these.
constexpr float kGradientTolerance = 1e-7f;
constexpr float kStepTolerance = 1e-6f;
constexpr uint8_t kMaxIterations = 50;
// Fewer points cannot pin down nine unknowns.
constexpr size_t kMinPoints = 12;

Eigen::Matrix3f Shape(const EllipsoidFitParams &p) {
  Eigen::Matrix3f s;
  s << p.diag.x(), p.offdiag.x(), p.offdiag.y(),  //
      p.offdiag.x(), p.diag.y(), p.offdiag.z(),   //
      p.offdiag.y(), p.offdiag.z(), p.diag.z();
  return s;
}

Eigen::Vector3f Point(std::span<const float> x, std::span<const float> y,
                      std::span<const float> z, size_t k) {
  return {x[k], y[k], z[k]};
}

// Cholesky solve of (a + damping I) h = -g over the leading n unknowns;
// nullopt when the damped matrix is not positive definite.
std::optional<Vector9> SolveDamped(const Matrix9 &a, const Vector9 &g,
                                   float damping, int n) {
  Matrix9 l = Matrix9::Zero();
  for (int j = 0; j < n; ++j) {
    float pivot = a(j, j) + damping;
    for (int k = 0; k < j; ++k) {
      pivot -= l(j, k) * l(j, k);
    }
    if (!(pivot > 0.0f)) {
      return std::nullopt;
    }
    l(j, j) = std::sqrt(pivot);
    for (int i = j + 1; i < n; ++i) {
      float sum = a(i, j);
      for (int k = 0; k < j; ++k) {
        sum -= l(i, k) * l(j, k);
      }
      l(i, j) = sum / l(j, j);
    }
  }
  Vector9 h = Vector9::Zero();
  for (int i = 0; i < n; ++i) {
    float sum = -g(i);
    for (int k = 0; k < i; ++k) {
      sum -= l(i, k) * h(k);
    }
    h(i) = sum / l(i, i);
  }
  for (int i = n - 1; i >= 0; --i) {
    float sum = h(i);
    for (int k = i + 1; k < n; ++k) {
      sum -= l(k, i) * h(k);
    }
    h(i) = sum / l(i, i);
  }
  return h;
}

// |x|^2 = 2 b.x + c is linear in b and c = R^2 - |b|^2. Centred on the mean
// first, so a large offset cannot ill-condition the solve.
std::optional<Eigen::Vector3f> AlgebraicCentre(std::span<const float> x,
                                               std::span<const float> y,
                                               std::span<const float> z) {
  Eigen::Vector3f mean = Eigen::Vector3f::Zero();
  for (size_t k = 0; k < x.size(); ++k) {
    mean += Point(x, y, z, k);
  }
  mean /= static_cast<float>(x.size());

  Matrix9 ata = Matrix9::Zero();
  Vector9 aty = Vector9::Zero();
  for (size_t k = 0; k < x.size(); ++k) {
    const Eigen::Vector3f q = Point(x, y, z, k) - mean;
    const Eigen::Vector4f row(2.0f * q.x(), 2.0f * q.y(), 2.0f * q.z(), 1.0f);
    const float target = q.squaredNorm();
    for (int i = 0; i < 4; ++i) {
      aty(i) += row(i) * target;
      for (int j = i; j < 4; ++j) {
        ata(i, j) += row(i) * row(j);
      }
    }
  }
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < i; ++j) {
      ata(i, j) = ata(j, i);
    }
  }
  const std::optional<Vector9> w = SolveDamped(ata, -aty, 0.0f, 4);
  if (!w) {
    return std::nullopt;
  }
  const Eigen::Vector3f centre = w->head<3>();
  if (!((*w)(3) + centre.squaredNorm() > 0.0f)) {
    return std::nullopt;
  }
  return mean + centre;
}

// For a fixed offset and S, the best radius is the mean distance.
float MeanLength(const EllipsoidFitParams &p, std::span<const float> x,
                 std::span<const float> y, std::span<const float> z) {
  const Eigen::Matrix3f s = Shape(p);
  float sum = 0.0f;
  for (size_t k = 0; k < x.size(); ++k) {
    sum += (s * (Point(x, y, z, k) - p.offset)).norm();
  }
  return sum / static_cast<float>(x.size());
}

}  // namespace

void EllipsoidFit::Start(Stage stage, const EllipsoidFitParams &seed,
                         RadiusBounds bounds) {
  stage_ = stage;
  params_ = seed;
  bounds_ = bounds;
  at_ = Linearization{};
  damping_ = 0.0f;
  damping_growth_ = 2.0f;
  cost_ = 0.0f;
  iteration_ = 0;
}

EllipsoidFit::Status EllipsoidFit::Step(std::span<const float> x,
                                        std::span<const float> y,
                                        std::span<const float> z) {
  if (x.size() != y.size() || x.size() != z.size() || x.size() < kMinPoints ||
      iteration_ >= kMaxIterations) {
    return Status::kFailed;
  }
  ++iteration_;
  const int n = Unknowns();
  const float count = static_cast<float>(x.size());

  if (!at_.valid) {
    if (stage_ == Stage::kSphere) {
      const std::optional<Eigen::Vector3f> centre = AlgebraicCentre(x, y, z);
      if (!centre) {
        return Status::kFailed;
      }
      params_.offset = *centre;
      params_.radius = MeanLength(params_, x, y, z);
    }
    at_ = Linearize(params_, x, y, z);
    if (!at_.valid) {
      return Status::kFailed;
    }
    cost_ = std::sqrt(at_.sum_sq / count);
    damping_ = kInitialDamping * at_.jtj.diagonal().head(n).maxCoeff();
    return Status::kRunning;
  }

  const std::optional<Vector9> h = SolveDamped(at_.jtj, at_.jte, damping_, n);
  if (h) {
    const Vector9 theta = Pack(params_);
    if (h->head(n).norm() <=
        kStepTolerance * (theta.head(n).norm() + kStepTolerance)) {
      return Finish();
    }
    const EllipsoidFitParams trial = Unpack(theta + *h);
    const Linearization next = Linearize(trial, x, y, z);
    // Actual over predicted reduction; predicted = 1/2 h^T (damping h - J^T e).
    const float predicted =
        0.5f * h->head(n).dot((damping_ * h->head(n)) - at_.jte.head(n));
    const float gain =
        next.valid ? (0.5f * (at_.sum_sq - next.sum_sq)) / predicted : -1.0f;
    if (gain > 0.0f) {
      params_ = trial;
      at_ = next;
      cost_ = std::sqrt(at_.sum_sq / count);
      const float shrink = (2.0f * gain) - 1.0f;
      damping_ *= std::max(1.0f / 3.0f, 1.0f - (shrink * shrink * shrink));
      damping_growth_ = 2.0f;
      if (at_.jte.head(n).lpNorm<Eigen::Infinity>() <=
          kGradientTolerance * count) {
        return Finish();
      }
      return iteration_ >= kMaxIterations ? Status::kFailed : Status::kRunning;
    }
  }
  damping_ *= damping_growth_;
  damping_growth_ *= 2.0f;
  if (!std::isfinite(damping_) || iteration_ >= kMaxIterations) {
    return Status::kFailed;
  }
  return Status::kRunning;
}

int EllipsoidFit::Unknowns() const { return stage_ == Stage::kSphere ? 4 : 9; }

// Sphere: offset, radius. Ellipsoid: offset, diag, offdiag.
EllipsoidFit::Vector9 EllipsoidFit::Pack(const EllipsoidFitParams &p) const {
  Vector9 theta = Vector9::Zero();
  theta.head<3>() = p.offset;
  if (stage_ == Stage::kSphere) {
    theta(3) = p.radius;
  } else {
    theta.segment<3>(3) = p.diag;
    theta.segment<3>(6) = p.offdiag;
  }
  return theta;
}

EllipsoidFitParams EllipsoidFit::Unpack(const Vector9 &theta) const {
  EllipsoidFitParams p = params_;
  p.offset = theta.head<3>();
  if (stage_ == Stage::kSphere) {
    p.radius = theta(3);
  } else {
    p.diag = theta.segment<3>(3);
    p.offdiag = theta.segment<3>(6);
  }
  return p;
}

// e = |v| - R, u = x - offset, v = S u: de/doffset = -S v/|v|, de/dR = -1,
// de/dS_ii = u_i v_i/|v|, de/dS_ij = (u_i v_j + u_j v_i)/|v|.
EllipsoidFit::Linearization EllipsoidFit::Linearize(
    const EllipsoidFitParams &p, std::span<const float> x,
    std::span<const float> y, std::span<const float> z) const {
  const int n = Unknowns();
  const Eigen::Matrix3f s = Shape(p);
  Linearization out;
  for (size_t k = 0; k < x.size(); ++k) {
    const Eigen::Vector3f u = Point(x, y, z, k) - p.offset;
    const Eigen::Vector3f v = s * u;
    const float length = v.norm();
    if (!(length > 1e-9f)) {
      return out;  // a point at the centre has no direction to fit
    }
    const float residual = length - p.radius;
    Vector9 row = Vector9::Zero();
    row.head<3>() = -(s * v) / length;
    if (stage_ == Stage::kSphere) {
      row(3) = -1.0f;
    } else {
      row.segment<3>(3) = u.cwiseProduct(v) / length;
      row(6) = ((u.x() * v.y()) + (u.y() * v.x())) / length;
      row(7) = ((u.x() * v.z()) + (u.z() * v.x())) / length;
      row(8) = ((u.y() * v.z()) + (u.z() * v.y())) / length;
    }
    for (int i = 0; i < n; ++i) {
      out.jte(i) += row(i) * residual;
      for (int j = i; j < n; ++j) {
        out.jtj(i, j) += row(i) * row(j);
      }
    }
    out.sum_sq += residual * residual;
  }
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < i; ++j) {
      out.jtj(i, j) = out.jtj(j, i);
    }
  }
  out.valid = std::isfinite(out.sum_sq) && out.jte.head(n).allFinite() &&
              out.jtj.topLeftCorner(n, n).allFinite();
  return out;
}

EllipsoidFit::Status EllipsoidFit::Finish() const {
  return params_.radius > bounds_.min && params_.radius < bounds_.max
             ? Status::kConverged
             : Status::kFailed;
}

}  // namespace math
