#pragma once

#include "state_machine.hpp"
#include <cstdint>
#include <cstring>

struct PVTData {
  uint32_t iTOW; // 0 GPS Time of Week [ms]
  uint16_t year; // 4 Year
  uint8_t month; // 6 Month
  uint8_t day;   // 7 Day
  uint8_t hour;  // 8 Hour
  uint8_t min;   // 9 Minute
  uint8_t sec;   // 10 Second
  uint8_t valid; // 11 Validity flags

  uint32_t tAcc; // 12 Time accuracy estimate (UTC) [ns]
  int32_t nano;  // 16 Fractional part of iTOW [ns]

  uint8_t fixType; // 20 Fix type
  uint8_t flags;   // 21 Fix Status Flags
  uint8_t flags2;  // 22 Fix Status Flags 2
  uint8_t numSV;   // 23 Number of SVs used in Nav Solution

  int32_t lon;    // 24 Longitude [1e-7 deg]
  int32_t lat;    // 28 Latitude [1e-7 deg]
  int32_t height; // 32 Height above ellipsoid [mm]
  int32_t hMSL;   // 36 Height above mean sea level [mm]
  uint32_t hAcc;  // 40 Horizontal accuracy estimate [mm]
  uint32_t vAcc;  // 44 Vertical accuracy estimate [mm]

  int32_t velN;     // 48 NED north velocity [mm/s]
  int32_t velE;     // 52 NED east velocity [mm/s]
  int32_t velD;     // 56 NED down velocity [mm/s]
  int32_t gSpeed;   // 60 Ground Speed (2-D) [mm/s]
  int32_t headMot;  // 64 Heading of motion (2-D) [1e-5 deg]
  uint32_t sAcc;    // 68 Speed accuracy estimate [mm/s]
  uint32_t headAcc; // 72 Heading accuracy estimate [1e-5 deg]

  uint16_t pDOP;        // 76 Position DOP [0.01]
  uint8_t reserved1[6]; // 78..83 Reserved

  int32_t headVeh; // 84 Heading of vehicle (2-D) [1e-5 deg]
  int16_t magDec;  // 88 Magnetic Declination [1e-5 deg]
  uint16_t magAcc; // 90 Magnetic Declination accuracy [1e-5 deg]
} __attribute__((packed));

static_assert(sizeof(PVTData) == 92,
              "PVTData size must match UBX NAV-PVT payload");

// UBX-NAV-DOP: Dilution of Precision
struct DOPData {
  uint32_t iTOW;    // 0 GPS Time of Week [ms]
  uint16_t gDOP;    // 4 Geometric DOP [0.01]
  uint16_t pDOP;    // 6 Position DOP [0.01]
  uint16_t tDOP;    // 8 Time DOP [0.01]
  uint16_t vDOP;    // 10 Vertical DOP [0.01]
  uint16_t hDOP;    // 12 Horizontal DOP [0.01]
  uint16_t nDOP;    // 14 Northing DOP [0.01]
  uint16_t eDOP;    // 16 Easting DOP [0.01]
} __attribute__((packed));

static_assert(sizeof(DOPData) == 18,
              "DOPData size must match UBX NAV-DOP payload");

// UBX-NAV-COV: Covariance Matrix
struct COVData {
  uint32_t iTOW;       // 0 GPS Time of Week [ms]
  uint8_t version;     // 4 Message version (0x00)
  uint8_t posCovValid; // 5 Position covariance valid flag
  uint8_t velCovValid; // 6 Velocity covariance valid flag
  uint8_t reserved1;   // 7 Reserved
  float posCovNN;      // 8 Position covariance North-North [m²]
  float posCovNE;      // 12 Position covariance North-East [m²]
  float posCovND;      // 16 Position covariance North-Down [m²]
  float posCovEE;      // 20 Position covariance East-East [m²]
  float posCovED;      // 24 Position covariance East-Down [m²]
  float posCovDD;      // 28 Position covariance Down-Down [m²]
  float velCovNN;      // 32 Velocity covariance North-North [m²/s²]
  float velCovNE;      // 36 Velocity covariance North-East [m²/s²]
  float velCovND;      // 40 Velocity covariance North-Down [m²/s²]
  float velCovEE;      // 44 Velocity covariance East-East [m²/s²]
  float velCovED;      // 48 Velocity covariance East-Down [m²/s²]
  float velCovDD;      // 52 Velocity covariance Down-Down [m²/s²]
} __attribute__((packed));

static_assert(sizeof(COVData) == 56,
              "COVData size must match UBX NAV-COV payload");

// Context for the parser state machine
struct ParserContext {
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

  static constexpr size_t MAX_PAYLOAD_SIZE = 120;
  uint8_t payload_buf[MAX_PAYLOAD_SIZE];

  // Output hooks
  PVTData &pvt_out;
  DOPData &dop_out;
  COVData &cov_out;
  bool &new_data_out;

  // EOE (End of Epoch) marker - used to commit PVT data
  bool epoch_ready = false;
  bool dop_ready = false;
  bool cov_ready = false;

  // Health Stats
  uint32_t checksum_fail_count = 0;
  uint32_t oversize_len_count = 0;
  uint32_t frame_ok_count = 0;

  // Reference to the state machine to trigger transitions
  StateMachine<ParserContext> *sm = nullptr;

  ParserContext(PVTData &pvt, DOPData &dop, COVData &cov, bool &flag)
      : pvt_out(pvt), dop_out(dop), cov_out(cov), new_data_out(flag) {}
};

struct ParserContext;

class M9NService {
public:
  M9NService();

  // Feed a byte to the parser
  void ProcessByte(uint8_t byte);

  const PVTData &GetData() const { return pvt_data_; }
  const DOPData &GetDOP() const { return dop_data_; }
  const COVData &GetCOV() const { return cov_data_; }
  bool NewDataAvailable() const { return new_data_; }
  void ClearNewDataFlag() { new_data_ = false; }

  // Stats Access
  uint32_t GetChecksumFailCount() const { return ctx_.checksum_fail_count; }
  uint32_t GetOversizeLenCount() const { return ctx_.oversize_len_count; }
  uint32_t GetFrameOkCount() const { return ctx_.frame_ok_count; }

private:
  PVTData pvt_data_{};
  DOPData dop_data_{};
  COVData cov_data_{};
  bool new_data_ = false;

  ParserContext ctx_;
  StateMachine<ParserContext> sm_;
};
