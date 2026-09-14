// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi
//
// Ported from PX4 Autopilot, src/modules/commander/lm_fit.{hpp,cpp}:
//
// Copyright (c) 2021 PX4 Development Team. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name PX4 nor the names of its contributors may be used to
//    endorse or promote products derived from this software without specific
//    prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include <Eigen/Core>
#include <cstdint>
#include <span>

// The field as the magnetometer sees it, in gauss: a sphere of the Earth's
// magnitude, displaced by the hard iron and squashed by the soft iron.
// Corrected sample = S * (raw - offset), S the symmetric matrix
//   [ diag.x     offdiag.x  offdiag.y ]
//   [ offdiag.x  diag.y     offdiag.z ]
//   [ offdiag.y  offdiag.z  diag.z    ]
struct MagFitParams {
  Eigen::Vector3f offset = Eigen::Vector3f::Zero();
  Eigen::Vector3f diag = Eigen::Vector3f::Ones();
  Eigen::Vector3f offdiag = Eigen::Vector3f::Zero();
  float radius = 0.2f;
};

// Levenberg-Marquardt least squares, one iteration per Step so the loop that
// owns the samples is never held for a whole fit. The sphere stage solves
// radius and offset with the scale fixed; the ellipsoid stage solves the nine
// scale and offset terms with the radius fixed, and must be seeded with a
// converged sphere.
class MagFit {
 public:
  enum class Stage : uint8_t { kSphere, kEllipsoid };
  enum class Status : uint8_t { kRunning, kConverged, kFailed };

  // The Earth's field is 0.25 to 0.65 gauss; a radius outside this is not a
  // fit of it.
  static constexpr float kMinRadius = 0.2f;
  static constexpr float kMaxRadius = 0.7f;

  void Start(Stage stage, const MagFitParams &seed);
  Status Step(std::span<const float> x, std::span<const float> y,
              std::span<const float> z);

  const MagFitParams &Params() const { return params_; }
  float Cost() const { return cost_; }

 private:
  bool SphereIteration(std::span<const float> x, std::span<const float> y,
                       std::span<const float> z);
  bool EllipsoidIteration(std::span<const float> x, std::span<const float> y,
                          std::span<const float> z);

  MagFitParams params_{};
  Stage stage_ = Stage::kSphere;
  float cost_ = 0.0f;
  float damping_ = 1.0f;
  uint8_t iteration_ = 0;
};
