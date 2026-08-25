// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include <cstdint>

// What came of asking something to do work, for callers with more than one
// sensible response. Deliberately not named for transports: kRejected is
// backpressure wherever it appears -- a full TX ring, a queue that will not
// take another item, a service declining while it is busy -- and it means the
// same thing to the caller in all of them.
//
// Named for what the caller can do rather than for the flag that produced it,
// so the hardware detail stays in each driver's fault counters, where a reader
// can take a delta over it.
enum class Outcome : uint8_t {
  kOk = 0,
  // Refused before anything happened. Nothing was written, nothing was
  // consumed, so the same call retried later can succeed.
  kRejected,
  // Abandoned part-way, because whatever was being waited on stopped
  // answering. Any work already done is a fragment, and any output buffer
  // holds only what arrived before that.
  kTimeout,
  // It ran and it answered, but the answer failed its integrity check. Unlike
  // kInvalid the same call may well succeed next time, so this is worth a
  // retry and worth counting separately from one that never answered.
  kCorrupt,
  // The request could not be formed, or the target is not open. Retrying with
  // these arguments fails the same way.
  kInvalid,
};
