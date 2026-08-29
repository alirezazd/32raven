// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include <cstdint>
#include <cstring>

#include "checksum.hpp"
#include "shared_state.hpp"
#include "uart.hpp"
#include "ubx.hpp"

struct M10ParserContext {
  uint8_t cls;
  uint8_t id;
  uint16_t len;
  uint16_t payload_idx;
  uint8_t ck_a;
  uint8_t ck_b;
  checksum::Fletcher8 ck_calc;
  uint32_t pvt_itow_ms = 0;
  uint32_t pvt_rx_us = 0;

  static constexpr size_t kMaxPayloadSize = 120;
  uint8_t payload_buf[kMaxPayloadSize];

  M10PVTData &pvt_out;
  M10DOPData &dop_out;
  M10COVData &cov_out;
  bool &new_data_out;

  // Frames are stamped with the UART's idle-line time, not the parse time --
  // bytes queue for an unbounded spell before the poll reaches them, and the
  // epoch guard compares two such stamps against each other.
  Uart2 *uart = nullptr;

  bool epoch_ready = false;
  bool dop_ready = false;
  bool cov_ready = false;

  uint32_t checksum_failures = 0;

  M10ParserContext(M10PVTData &pvt, M10DOPData &dop, M10COVData &cov,
                   bool &flag)
      : pvt_out(pvt), dop_out(dop), cov_out(cov), new_data_out(flag) {}
};

// Where the parser sits in a UBX frame. Positions, not states with a
// lifecycle -- which is why this is an enum and not IState.
enum class M10Parse : uint8_t {
  kSync1,
  kSync2,
  kClass,
  kId,
  kLengthL,
  kLengthH,
  kPayload,
  kCkA,
  kCkB,
};

class M10Service {
 public:
  M10Service();

  void ProcessByte(uint8_t byte);

  const M10PVTData &GetData() const { return pvt_data_; }
  const M10DOPData &GetDOP() const { return dop_data_; }
  const M10COVData &GetCOV() const { return cov_data_; }

  // Drains the receiver, parses, and publishes a completed epoch to the
  // blackboard, where TelemetryPublisher tells a new fix from a re-read one by
  // its stamp.
  void Poll();

 private:
  friend class System;

  void Init(Uart2 &uart, SharedState &blackboard);
  GpsData BuildGpsData() const;
  void PublishIfNew();
  void DispatchFrame();

  Uart2 *uart_ = nullptr;
  SharedState *blackboard_ = nullptr;
  M10PVTData pvt_data_{};
  M10DOPData dop_data_{};
  M10COVData cov_data_{};
  bool new_data_ = false;

  M10ParserContext ctx_;
  M10Parse parse_ = M10Parse::kSync1;
};
