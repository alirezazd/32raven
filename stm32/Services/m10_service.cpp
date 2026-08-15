// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "m10_service.hpp"

#include "m10_reg.hpp"
#include "shared_state.hpp"
#include "uart.hpp"

namespace {

// Bytes drained per poll. A full epoch is ~150 bytes across four messages, and
// the line delivers 11.5 kB/s against a 1 kHz poll, so this is a bound on
// catch-up bursts rather than on steady flow. Cutting it below an epoch's
// worth would spread one fix across several passes for no gain.
constexpr uint32_t kRxByteBudget = 32;

static constexpr uint8_t kNavPvtValidDateBit = 1u << 0;
static constexpr uint8_t kNavPvtValidTimeBit = 1u << 1;
static constexpr uint8_t kNavPvtFullyResolvedBit = 1u << 2;

bool HasReliableGpsUtc(const M10PVTData &pvt) {
  const uint8_t required_bits =
      kNavPvtValidDateBit | kNavPvtValidTimeBit | kNavPvtFullyResolvedBit;
  return (pvt.valid & required_bits) == required_bits;
}

}  // namespace

void M10Service::ProcessByte(uint8_t byte) {
  M10ParserContext &ctx = ctx_;

  // Running checksum over class, id, length and payload -- every byte between
  // the sync pair and the checksum pair, in arrival order (UBX 8-bit Fletcher).
  const auto accumulate = [&ctx](uint8_t b) {
    ctx.ck_a_calc += b;
    ctx.ck_b_calc += ctx.ck_a_calc;
  };

  switch (parse_) {
    case M10Parse::kSync1:
      if (byte == UBX::kSync1) {
        parse_ = M10Parse::kSync2;
      }
      break;

    case M10Parse::kSync2:
      // A second 0xB5 is a fresh start, not a failure: the first was the tail
      // of noise and this one may open the real frame.
      if (byte == UBX::kSync2) {
        parse_ = M10Parse::kClass;
      } else if (byte != UBX::kSync1) {
        parse_ = M10Parse::kSync1;
      }
      break;

    case M10Parse::kClass:
      ctx.cls = byte;
      ctx.ck_a_calc = 0;
      ctx.ck_b_calc = 0;
      accumulate(byte);
      parse_ = M10Parse::kId;
      break;

    case M10Parse::kId:
      ctx.id = byte;
      accumulate(byte);
      parse_ = M10Parse::kLengthL;
      break;

    case M10Parse::kLengthL:
      ctx.len = byte;
      accumulate(byte);
      parse_ = M10Parse::kLengthH;
      break;

    case M10Parse::kLengthH:
      ctx.len |= static_cast<uint16_t>(static_cast<uint16_t>(byte) << 8);
      accumulate(byte);
      ctx.payload_idx = 0;
      if (ctx.len > M10ParserContext::kMaxPayloadSize) {
        ctx.oversize_len_count++;
        parse_ = M10Parse::kSync1;
      } else if (ctx.len == 0) {
        parse_ = M10Parse::kCkA;
      } else {
        parse_ = M10Parse::kPayload;
      }
      break;

    case M10Parse::kPayload:
      if (ctx.payload_idx < ctx.len) {
        ctx.payload_buf[ctx.payload_idx] = byte;
        accumulate(byte);
        ctx.payload_idx++;
      }
      if (ctx.payload_idx >= ctx.len) {
        parse_ = M10Parse::kCkA;
      }
      break;

    case M10Parse::kCkA:
      ctx.ck_a = byte;
      if (ctx.ck_a == ctx.ck_a_calc) {
        parse_ = M10Parse::kCkB;
      } else {
        ctx.checksum_fail_count++;
        parse_ = M10Parse::kSync1;
      }
      break;

    case M10Parse::kCkB:
      ctx.ck_b = byte;
      parse_ = M10Parse::kSync1;
      if (ctx.ck_b != ctx.ck_b_calc) {
        ctx.checksum_fail_count++;
        break;
      }
      DispatchFrame();
      break;
  }
}

// Checksum has passed; the frame is whole. Anything unrecognised still counts
// as received -- the rate is what says the link is healthy, not the mix.
void M10Service::DispatchFrame() {
  M10ParserContext &ctx = ctx_;
  ctx.frame_ok_count++;

  if (ctx.cls != UBX::kClsNav) {
    return;
  }

  if (ctx.id == UBX::kIdNavPvt && ctx.len == sizeof(M10PVTData)) {
    std::memcpy(&ctx.pvt_out, ctx.payload_buf, sizeof(M10PVTData));
    ctx.epoch_ready = true;
    ctx.pvt_itow_ms = ctx.pvt_out.iTOW;
    ctx.pvt_rx_us = ctx.uart->GetLastRxTime();
  } else if (ctx.id == UBX::kIdNavDop && ctx.len == sizeof(M10DOPData)) {
    std::memcpy(&ctx.dop_out, ctx.payload_buf, sizeof(M10DOPData));
    ctx.dop_ready = true;
  } else if (ctx.id == UBX::kIdNavCov && ctx.len == sizeof(M10COVData)) {
    std::memcpy(&ctx.cov_out, ctx.payload_buf, sizeof(M10COVData));
    ctx.cov_ready = true;
  } else if (ctx.id == UBX::kIdNavEoe && ctx.len == 4) {
    // End-of-epoch closes the set. The PVT it closes must be the same fix
    // (matching iTOW) and recent, or a stale one would publish as new.
    const uint32_t eoe_itow_ms =
        static_cast<uint32_t>(ctx.payload_buf[0]) |
        (static_cast<uint32_t>(ctx.payload_buf[1]) << 8) |
        (static_cast<uint32_t>(ctx.payload_buf[2]) << 16) |
        (static_cast<uint32_t>(ctx.payload_buf[3]) << 24);
    constexpr uint64_t max_age_us = 150000;
    const uint64_t now_us = ctx.uart->GetLastRxTime();

    if (ctx.epoch_ready && eoe_itow_ms == ctx.pvt_itow_ms &&
        (now_us - ctx.pvt_rx_us) < max_age_us) {
      ctx.new_data_out = true;
    }

    ctx.epoch_ready = false;
    ctx.dop_ready = false;
    ctx.cov_ready = false;
  }
}

M10Service::M10Service() : ctx_(pvt_data_, dop_data_, cov_data_, new_data_) {}

void M10Service::Init(Uart2 &uart, SharedState &blackboard) {
  uart_ = &uart;
  ctx_.uart = &uart;
  blackboard_ = &blackboard;
}

bool M10Service::Poll() {
  if (uart_ == nullptr) {
    return false;
  }

  uint8_t byte = 0;
  uint32_t budget = kRxByteBudget;
  while (budget > 0u && uart_->ReadByte(byte)) {
    ProcessByte(byte);
    budget--;
  }

  return PublishIfNew();
}

void M10Service::FillGpsData(GpsData &data) const {
  // When the fix arrived, not when it was collected. Stamping at the call
  // makes every fix look new however long it queued, which is exactly the
  // case a freshness check downstream exists to catch.
  data.timestamp_us = ctx_.pvt_rx_us;
  data.lat = pvt_data_.lat;
  data.lon = pvt_data_.lon;
  data.alt = pvt_data_.hMSL;
  data.vel = static_cast<uint16_t>(pvt_data_.gSpeed / 10);  // mm/s -> cm/s
  data.hdg =
      static_cast<uint16_t>(pvt_data_.headMot / 1000);  // 1e-5 deg -> cdeg
  data.num_sats = pvt_data_.numSV;
  data.fix_type = pvt_data_.fixType;
  if (HasReliableGpsUtc(pvt_data_)) {
    data.year = pvt_data_.year;
    data.month = pvt_data_.month;
    data.day = pvt_data_.day;
    data.hour = pvt_data_.hour;
    data.min = pvt_data_.min;
    data.sec = pvt_data_.sec;
  }
  data.hAcc = pvt_data_.hAcc;
  data.vAcc = pvt_data_.vAcc;
  data.gDOP = dop_data_.gDOP;
  data.pDOP = dop_data_.pDOP;
  data.hDOP = dop_data_.hDOP;
  data.vDOP = dop_data_.vDOP;

  data.posCovValid = cov_data_.posCovValid;
  data.velCovValid = cov_data_.velCovValid;
  data.posCovNN = cov_data_.posCovNN;
  data.posCovEE = cov_data_.posCovEE;
  data.posCovDD = cov_data_.posCovDD;
}

bool M10Service::PublishIfNew() {
  if (!new_data_ || blackboard_ == nullptr) {
    return false;
  }

  GpsData data{};
  FillGpsData(data);
  blackboard_->UpdateGps(data);
  new_data_ = false;
  return true;
}
