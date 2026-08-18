// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "iekf.hpp"

namespace {

Iekf g_iekf;

}  // namespace

extern "C" void EstimatorOnTim5Irq(void) { g_iekf.Step(); }
