// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

// The RTTTL score for every message::Tone, in enum order.

#pragma once

#include <cstddef>

#include "message.hpp"

namespace tone_scores {

inline constexpr char kBeepRtttl[] = "Beep:d=16,o=6,b=240:c";
inline constexpr char kConfirmRtttl[] = "Confirm:d=32,o=6,b=200:g,c7,16e7";
inline constexpr char kWarningRtttl[] = "Warning:d=32,o=6,b=180:c,p,e,p,c5";
inline constexpr char kErrorRtttl[] = "Error:d=32,o=6,b=180:a,p,a,p,a,p,a,p,a";
inline constexpr char kAm32StartupRtttl[] =
    "am32_startup:d=16,o=5,b=75:16a4,16d,16a#";
inline constexpr char kAm32BrushedRtttl[] =
    "am32_brushed:d=16,o=5,b=50:16d,16g,16a#,16d6";
inline constexpr char kAm32DuskingRtttl[] =
    "am32_dusking:d=32,o=4,b=75:16g,32a.,32a#.,32c5,32a#,32a,16a#5,32a.";
inline constexpr char kAm32InputRtttl[] =
    "am32_input:d=32,o=4,b=75:32d,32e,32d5";
inline constexpr char kAm32Input2Rtttl[] =
    "am32_input2:d=32,o=4,b=100:32g,32d,32c";
inline constexpr char kAm32DefaultRtttl[] =
    "am32_default:d=32,o=4,b=50:32a#,32g5";
inline constexpr char kAm32ChangedRtttl[] =
    "am32_changed:d=32,o=4,b=50:32d5,32d";
inline constexpr char kDoomRtttl[] =  //    \m/
    "doom:d=4,o=5,b=110:32e4.,64p,32e4.,64p,32e.,64p,32e4.,64p,32e4.,64p,"
    "32d.,64p,32e4.,64p,32e4.,64p,32c.,64p,32e4.,64p,32e4.,64p,32a#4.,64p"
    ",32e4.,64p,32e4.,64p,32b4.,64p,32c.,64p,32e4.,64p,32e4.,64p,32e.,64p"
    ",32e4.,64p,32e4.,64p,32d.,64p,32e4.,64p,32e4.,64p,32c.,64p,32e4.,64p"
    ",32e4.,64p,a#4,16p,32e4.,64p,32e4.,64p,32e.,64p,32e4.,64p,32e4.,64p,"
    "32d.,64p,32e4.,64p,32e4.,64p,32c.,64p,32e4.,64p,32e4.,64p,32a#4.,64p"
    ",32e4.,64p,32e4.,64p,32b4.,64p,32c.,64p,32e4.,64p,32e4.,64p,32e.,64p"
    ",32e4.,64p,32e4.,64p,32d.,64p,32e4.,64p,32e4.,64p,32c.,64p,32e4.,64p"
    ",32e4.,64p,a#4,16p,32a4.,64p,32a4.,64p,32a.,64p,32a4.,64p,32a4.,64p,"
    "32g.,64p,32a4.,64p,32a4.,64p,32f.,64p,32a4.,64p,32a4.,64p,32d#.,64p,"
    "32a4.,64p,32a4.,64p,32e.,64p,32f.,64p,32a4.,64p,32a4.,64p,32a.,64p,3"
    "2a4.,64p,32a4.,64p,32g.,64p,32a4.,64p,32a4.,64p,32f.,64p,32a4.,64p,3"
    "2a4.,64p,d#";
inline constexpr char kDoomShortRtttl[] =
    "doom_short:d=4,o=5,b=110:32e4.,64p,32e4.,64p,32e.,64p,32e4.,64p,32e4"
    ".,64p,32d.,64p,32e4.,64p,32e4.,64p,32c.,64p,32e4.,64p,32e4.,64p,32a#"
    "4.,64p,32e4.,64p,32e4.,64p,32b4.,64p,32c.,64p";

// Indexed by message::Tone. A new enumerator without a score fails the
// static_assert rather than falling through to silence at runtime.
inline constexpr const char *kAll[] = {
    kBeepRtttl,        kConfirmRtttl,     kWarningRtttl,     kErrorRtttl,
    kAm32StartupRtttl, kAm32BrushedRtttl, kAm32DuskingRtttl, kAm32InputRtttl,
    kAm32Input2Rtttl,  kAm32DefaultRtttl, kAm32ChangedRtttl, kDoomRtttl,
    kDoomShortRtttl,
};

static_assert(std::size(kAll) ==
                  static_cast<std::size_t>(message::Tone::kCount),
              "every message::Tone needs a score in tone_scores::kAll");

}  // namespace tone_scores
