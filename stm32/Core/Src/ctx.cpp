// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "ctx.hpp"

#include "system.hpp"

AppContext::AppContext() { sys = &System::GetInstance(); }
