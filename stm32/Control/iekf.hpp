// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

// The estimator loop (#27), empty until the filter exists. Step() runs in the
// TIM5 interrupt and must stay bounded: TIM5 re-fires every millisecond, and a
// step outlasting its period leaves the interrupt permanently pending with
// thread mode never running again.
class Iekf {
 public:
  void Step() {}
};

extern "C" void EstimatorOnTim5Irq(void);
