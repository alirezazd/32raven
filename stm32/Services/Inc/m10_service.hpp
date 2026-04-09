#pragma once

#include <cstdint>
#include <cstring>

#include "state_machine.hpp"

struct M10PVTData {
  uint32_t iTOW;
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t min;
  uint8_t sec;
  uint8_t valid;
  uint32_t tAcc;
  int32_t nano;
  uint8_t fixType;
  uint8_t flags;
  uint8_t flags2;
  uint8_t numSV;
  int32_t lon;
  int32_t lat;
  int32_t height;
  int32_t hMSL;
  uint32_t hAcc;
  uint32_t vAcc;
  int32_t velN;
  int32_t velE;
  int32_t velD;
  int32_t gSpeed;
  int32_t headMot;
  uint32_t sAcc;
  uint32_t headAcc;
  uint16_t pDOP;
  uint8_t reserved1[6];
  int32_t headVeh;
  int16_t magDec;
  uint16_t magAcc;
} __attribute__((packed));

static_assert(sizeof(M10PVTData) == 92,
              "M10PVTData size must match UBX NAV-PVT payload");

struct M10DOPData {
  uint32_t iTOW;
  uint16_t gDOP;
  uint16_t pDOP;
  uint16_t tDOP;
  uint16_t vDOP;
  uint16_t hDOP;
  uint16_t nDOP;
  uint16_t eDOP;
} __attribute__((packed));

static_assert(sizeof(M10DOPData) == 18,
              "M10DOPData size must match UBX NAV-DOP payload");

struct M10COVData {
  uint32_t iTOW;
  uint8_t version;
  uint8_t posCovValid;
  uint8_t velCovValid;
  uint8_t reserved1;
  float posCovNN;
  float posCovNE;
  float posCovND;
  float posCovEE;
  float posCovED;
  float posCovDD;
  float velCovNN;
  float velCovNE;
  float velCovND;
  float velCovEE;
  float velCovED;
  float velCovDD;
} __attribute__((packed));

static_assert(sizeof(M10COVData) == 56,
              "M10COVData size must match UBX NAV-COV payload");

struct M10ParserContext {
  uint8_t current_byte;

  uint8_t cls;
  uint8_t id;
  uint16_t len;
  uint16_t payload_idx;
  uint8_t ck_a;
  uint8_t ck_b;
  uint8_t ck_a_calc;
  uint8_t ck_b_calc;
  uint32_t pvt_itow_ms = 0;
  uint64_t pvt_rx_us = 0;

  static constexpr size_t kMaxPayloadSize = 120;
  uint8_t payload_buf[kMaxPayloadSize];

  M10PVTData &pvt_out;
  M10DOPData &dop_out;
  M10COVData &cov_out;
  bool &new_data_out;

  bool epoch_ready = false;
  bool dop_ready = false;
  bool cov_ready = false;

  uint32_t checksum_fail_count = 0;
  uint32_t oversize_len_count = 0;
  uint32_t frame_ok_count = 0;

  StateMachine<M10ParserContext> *sm = nullptr;

  M10ParserContext(M10PVTData &pvt, M10DOPData &dop, M10COVData &cov, bool &flag)
      : pvt_out(pvt), dop_out(dop), cov_out(cov), new_data_out(flag) {}
};

class M10Service {
 public:
  M10Service();

  void ProcessByte(uint8_t byte);

  const M10PVTData &GetData() const { return pvt_data_; }
  const M10DOPData &GetDOP() const { return dop_data_; }
  const M10COVData &GetCOV() const { return cov_data_; }
  bool NewDataAvailable() const { return new_data_; }
  void ClearNewDataFlag() { new_data_ = false; }

  uint32_t GetChecksumFailCount() const { return ctx_.checksum_fail_count; }
  uint32_t GetOversizeLenCount() const { return ctx_.oversize_len_count; }
  uint32_t GetFrameOkCount() const { return ctx_.frame_ok_count; }

 private:
  M10PVTData pvt_data_{};
  M10DOPData dop_data_{};
  M10COVData cov_data_{};
  bool new_data_ = false;

  M10ParserContext ctx_;
  StateMachine<M10ParserContext> sm_;
};
