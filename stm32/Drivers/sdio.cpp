// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "sdio.hpp"

#include <utility>

#include "error_code.hpp"
#include "panic.hpp"
#include "stm32f4xx.h"
#include "system.hpp"
#include "watchdog.hpp"

namespace {

// A single blocking wait can outlast the watchdog's ~700 ms window -- a
// card's program phase alone is specced to 250 ms. All of them run from the
// main loop and keep their own timeout, so a wedged loop still returns.
void FeedWatchdog() { Watchdog::GetInstance().Kick(); }

// For the paths that only ask whether the card answered: which way it failed
// is already in Stats by the time they see this.
bool Ok(Outcome o) { return o == Outcome::kOk; }

// 48 MHz kernel clock: /(118+2) = 400 kHz to identify, /(0+2) = 24 MHz for
// data. HWFC_EN stays off -- the F4 flow-control erratum corrupts data; DMA
// keeps the FIFO fed instead.
constexpr uint32_t kInitClkDiv = 118;
constexpr uint32_t kDataClkDiv = 0;
constexpr uint32_t kDataClockHz = 24000000;

constexpr uint32_t kCmdTimeoutUs = 10000;
constexpr uint32_t kAcmd41TimeoutUs = 1000000;
constexpr uint32_t kTransferTimeoutUs = 1000000;
// The SD spec caps a write's busy phase at 250 ms; real cards brush it.
constexpr uint32_t kProgramTimeoutUs = 500000;

constexpr uint32_t kIfCondCheckPattern = 0x1AA;
constexpr uint32_t kOcrHcs = 1u << 30;
constexpr uint32_t kOcrReady = 1u << 31;
constexpr uint32_t kOcrVoltageWindow = 0x00FF8000;

constexpr uint32_t kCardStatusReadyForData = 1u << 8;
constexpr uint32_t kCardStateTran = 4;

constexpr uint32_t kStaCmdFlags =
    SDIO_STA_CCRCFAIL | SDIO_STA_CTIMEOUT | SDIO_STA_CMDREND | SDIO_STA_CMDSENT;
constexpr uint32_t kStaDataErrors = SDIO_STA_DCRCFAIL | SDIO_STA_DTIMEOUT |
                                    SDIO_STA_TXUNDERR | SDIO_STA_RXOVERR;
constexpr uint32_t kIcrAll = 0x00C007FF;

// DBLOCKSIZE = log2(512).
constexpr uint32_t kDctrlBlock512 = 9u << SDIO_DCTRL_DBLOCKSIZE_Pos;

uint32_t Micros() { return System::GetInstance().Time().Micros(); }

bool Aligned4(const void *p) {
  return (reinterpret_cast<uintptr_t>(p) & 0x3u) == 0u;
}

}  // namespace

Sdio &Sdio::GetInstance() {
  static Sdio instance;
  return instance;
}

void Sdio::Init() {
  if (initialized_) {
    Panic(ErrorCode::Stm32::kSdioReinit);
  }

  RCC->APB2ENR |= RCC_APB2ENR_SDIOEN;
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

  present_ = ProbeCard();
  initialized_ = true;
}

bool Sdio::Reprobe() {
  if (!initialized_ || write_phase_ != WritePhase::kIdle) {
    return false;
  }
  present_ = ProbeCard();
  return present_;
}

void Sdio::PowerUpBus() {
  SDIO->POWER = 0;
  System::GetInstance().Time().DelayMicros(1000);
  SDIO->CLKCR = kInitClkDiv | SDIO_CLKCR_CLKEN;
  SDIO->POWER = SDIO_POWER_PWRCTRL;
  // The card needs 74 clocks (185 us at 400 kHz) before the first command.
  System::GetInstance().Time().DelayMicros(1000);
}

Outcome Sdio::SendCommand(Cmd cmd, uint32_t arg, Resp resp) {
  uint32_t waitresp = 0;
  if (resp == Resp::kShort || resp == Resp::kShortNoCrc) {
    waitresp = SDIO_CMD_WAITRESP_0;
  } else if (resp == Resp::kLong) {
    waitresp = SDIO_CMD_WAITRESP_0 | SDIO_CMD_WAITRESP_1;
  }

  SDIO->ICR = kStaCmdFlags;
  SDIO->ARG = arg;
  SDIO->CMD = std::to_underlying(cmd) | waitresp | SDIO_CMD_CPSMEN;

  const uint32_t start = Micros();
  while (true) {
    const uint32_t sta = SDIO->STA;
    if (resp == Resp::kNone) {
      if (sta & SDIO_STA_CMDSENT) {
        SDIO->ICR = SDIO_ICR_CMDSENTC;
        return Outcome::kOk;
      }
    } else {
      if (sta & SDIO_STA_CTIMEOUT) {
        SDIO->ICR = SDIO_ICR_CTIMEOUTC;
        ++stats_.cmd_timeouts;
        return Outcome::kTimeout;
      }
      if (sta & SDIO_STA_CCRCFAIL) {
        SDIO->ICR = SDIO_ICR_CCRCFAILC;
        if (resp == Resp::kShortNoCrc) {
          return Outcome::kOk;
        }
        ++stats_.cmd_crc_errors;
        return Outcome::kCorrupt;
      }
      if (sta & SDIO_STA_CMDREND) {
        SDIO->ICR = SDIO_ICR_CMDRENDC;
        return Outcome::kOk;
      }
    }
    if ((Micros() - start) > kCmdTimeoutUs) {
      ++stats_.cmd_timeouts;
      return Outcome::kTimeout;
    }
  }
}

Outcome Sdio::SendAppCommand(Acmd cmd, uint32_t arg, Resp resp) {
  const Outcome app =
      SendCommand(Cmd::kApp, static_cast<uint32_t>(rca_) << 16, Resp::kShort);
  if (app != Outcome::kOk) {
    return app;
  }
  return SendCommand(static_cast<Cmd>(std::to_underlying(cmd)), arg, resp);
}

bool Sdio::ProbeCard() {
  present_ = false;
  block_count_ = 0;
  rca_ = 0;
  write_phase_ = WritePhase::kIdle;

  PowerUpBus();

  if (!Ok(SendCommand(Cmd::kGoIdle, 0, Resp::kNone))) {
    return false;
  }

  // Only a card that answers CMD8 honours ACMD41's HCS bit; one that ignores
  // it is a V1 byte-addressed part this driver does not speak.
  if (!Ok(SendCommand(Cmd::kSendIfCond, kIfCondCheckPattern, Resp::kShort)) ||
      (SDIO->RESP1 & 0xFFFu) != kIfCondCheckPattern) {
    return false;
  }

  uint32_t ocr = 0;
  const uint32_t acmd41_start = Micros();
  while (true) {
    FeedWatchdog();
    if (!Ok(SendAppCommand(Acmd::kOpCond, kOcrHcs | kOcrVoltageWindow,
                            Resp::kShortNoCrc))) {
      return false;
    }
    ocr = SDIO->RESP1;
    if (ocr & kOcrReady) {
      break;
    }
    if ((Micros() - acmd41_start) > kAcmd41TimeoutUs) {
      return false;
    }
    System::GetInstance().Time().DelayMicros(1000);
  }
  if ((ocr & kOcrHcs) == 0u) {
    return false;
  }

  if (!Ok(SendCommand(Cmd::kAllSendCid, 0, Resp::kLong))) {
    return false;
  }
  if (!Ok(SendCommand(Cmd::kSendRelAddr, 0, Resp::kShort))) {
    return false;
  }
  rca_ = static_cast<uint16_t>(SDIO->RESP1 >> 16);

  if (!Ok(SendCommand(Cmd::kSendCsd, static_cast<uint32_t>(rca_) << 16,
                       Resp::kLong))) {
    return false;
  }
  // CSD v2: C_SIZE spans bits [69:48], and capacity = (C_SIZE + 1) * 512 KB.
  // RESP1 holds CSD[127:96], RESP2 [95:64], RESP3 [63:32].
  if ((SDIO->RESP1 >> 30) != 1u) {
    return false;
  }
  const uint32_t c_size = ((SDIO->RESP2 & 0x3Fu) << 16) | (SDIO->RESP3 >> 16);
  block_count_ = (c_size + 1u) * 1024u;

  if (!Ok(SendCommand(Cmd::kSelect, static_cast<uint32_t>(rca_) << 16,
                           Resp::kShort))) {
    return false;
  }
  if (!Ok(SendAppCommand(Acmd::kSetBusWidth, 2, Resp::kShort))) {
    return false;
  }

  SDIO->CLKCR = kDataClkDiv | SDIO_CLKCR_CLKEN | SDIO_CLKCR_WIDBUS_0;
  return WaitCardReady(kCmdTimeoutUs);
}

bool Sdio::WaitCardReady(uint32_t timeout_us) {
  const uint32_t start = Micros();
  while (true) {
    if (!Ok(SendCommand(Cmd::kSendStatus, static_cast<uint32_t>(rca_) << 16,
                             Resp::kShort))) {
      return false;
    }
    const uint32_t status = SDIO->RESP1;
    if ((status & kCardStatusReadyForData) &&
        ((status >> 9) & 0xFu) == kCardStateTran) {
      return true;
    }
    if ((Micros() - start) > timeout_us) {
      return false;
    }
    System::GetInstance().Time().DelayMicros(100);
  }
}

void Sdio::ConfigureDma(const uint8_t *buf, Dir dir) {
  DMA2_Stream3->CR &= ~DMA_SxCR_EN;
  while (DMA2_Stream3->CR & DMA_SxCR_EN) {
  }
  DMA2->LIFCR = DMA_LIFCR_CTCIF3 | DMA_LIFCR_CHTIF3 | DMA_LIFCR_CTEIF3 |
                DMA_LIFCR_CDMEIF3 | DMA_LIFCR_CFEIF3;

  // SDIO is DMA2 Stream3 Channel4 in the F407 request map. Under PFCTRL the
  // DPSM's DLEN ends the stream, so NDTR is written only for form. 4-beat
  // word bursts pair with the 16-byte FIFO threshold.
  DMA2_Stream3->CR = DMA_SxCR_CHSEL_2 | DMA_SxCR_MBURST_0 | DMA_SxCR_PBURST_0 |
                     DMA_SxCR_MSIZE_1 | DMA_SxCR_PSIZE_1 | DMA_SxCR_MINC |
                     DMA_SxCR_PFCTRL |
                     ((dir == Dir::kToCard) ? DMA_SxCR_DIR_0 : 0u);
  DMA2_Stream3->FCR = DMA_SxFCR_DMDIS | DMA_SxFCR_FTH;
  DMA2_Stream3->PAR = reinterpret_cast<uint32_t>(&SDIO->FIFO);
  DMA2_Stream3->M0AR = reinterpret_cast<uint32_t>(buf);
  DMA2_Stream3->NDTR = 0;
  DMA2_Stream3->CR |= DMA_SxCR_EN;
}

void Sdio::StopDma() {
  DMA2_Stream3->CR &= ~DMA_SxCR_EN;
  while (DMA2_Stream3->CR & DMA_SxCR_EN) {
  }
}

void Sdio::PrepareDataPath(uint32_t byte_len, Dir dir) {
  SDIO->ICR = kIcrAll;
  SDIO->DTIMER = kDataClockHz;
  SDIO->DLEN = byte_len;
  SDIO->DCTRL = kDctrlBlock512 | SDIO_DCTRL_DMAEN |
                ((dir == Dir::kFromCard) ? SDIO_DCTRL_DTDIR : 0u) |
                SDIO_DCTRL_DTEN;
}

void Sdio::AbortTransfer() {
  StopDma();
  SDIO->DCTRL = 0;
  SendCommand(Cmd::kStop, 0, Resp::kShort);
  SDIO->ICR = kIcrAll;
  write_phase_ = WritePhase::kIdle;
}

Outcome Sdio::ReadBlocks(uint32_t lba, std::span<uint8_t> dst) {
  // Split out of the guard below: a write in flight is the one refusal the
  // caller can wait out, where the rest need the request or the card fixed.
  if (write_phase_ != WritePhase::kIdle) {
    return Outcome::kRejected;
  }
  if (!initialized_ || !present_ || dst.empty() ||
      (dst.size() % kBlockBytes) != 0u) {
    return Outcome::kInvalid;
  }
  if (!Aligned4(dst.data())) {
    ++stats_.unaligned_refusals;
    return Outcome::kInvalid;
  }
  const bool multi = dst.size() > kBlockBytes;

  ConfigureDma(dst.data(), Dir::kFromCard);
  // Reads arm the DPSM before the command -- data can start the moment the
  // card sees it. Writes are the other way round.
  PrepareDataPath(static_cast<uint32_t>(dst.size()), Dir::kFromCard);

  const Outcome cmd =
      SendCommand(multi ? Cmd::kReadMulti : Cmd::kReadSingle, lba,
                  Resp::kShort);
  if (cmd != Outcome::kOk) {
    AbortTransfer();
    return cmd;
  }

  const uint32_t start = Micros();
  while (true) {
    FeedWatchdog();
    const uint32_t sta = SDIO->STA;
    if (sta & kStaDataErrors) {
      const bool crc = (sta & SDIO_STA_DCRCFAIL) != 0;
      if (crc) {
        ++stats_.data_crc_errors;
      } else {
        ++stats_.data_timeouts;
      }
      AbortTransfer();
      return crc ? Outcome::kCorrupt : Outcome::kTimeout;
    }
    if (sta & SDIO_STA_DATAEND) {
      break;
    }
    if ((Micros() - start) > kTransferTimeoutUs) {
      ++stats_.data_timeouts;
      AbortTransfer();
      return Outcome::kTimeout;
    }
  }

  // DATAEND only means the last byte entered the FIFO; the DMA is still
  // draining it. Stopping the stream now strands the tail in a FIFO the F4
  // cannot flush, from where it prefixes the next read and shifts a sector.
  while (!(DMA2->LISR & DMA_LISR_TCIF3)) {
    FeedWatchdog();
    if ((Micros() - start) > kTransferTimeoutUs) {
      ++stats_.data_timeouts;
      AbortTransfer();
      return Outcome::kTimeout;
    }
  }

  StopDma();
  SDIO->DCTRL = 0;
  Outcome stop = Outcome::kOk;
  if (multi) {
    stop = SendCommand(Cmd::kStop, 0, Resp::kShort);
  }
  SDIO->ICR = kIcrAll;
  return stop;
}

Outcome Sdio::StartWrite(uint32_t lba, std::span<const uint8_t> src) {
  // As in ReadBlocks: a write already in flight is the refusal worth retrying.
  if (write_phase_ != WritePhase::kIdle) {
    return Outcome::kRejected;
  }
  if (!initialized_ || !present_ || src.empty() ||
      (src.size() % kBlockBytes) != 0u) {
    return Outcome::kInvalid;
  }
  if (!Aligned4(src.data())) {
    ++stats_.unaligned_refusals;
    return Outcome::kInvalid;
  }
  if (!WaitCardReady(kCmdTimeoutUs)) {
    ++stats_.write_errors;
    return Outcome::kTimeout;
  }
  const bool multi = src.size() > kBlockBytes;

  ConfigureDma(src.data(), Dir::kToCard);
  const Outcome cmd = SendCommand(
      multi ? Cmd::kWriteMulti : Cmd::kWriteSingle, lba, Resp::kShort);
  if (cmd != Outcome::kOk) {
    StopDma();
    return cmd;
  }
  PrepareDataPath(static_cast<uint32_t>(src.size()), Dir::kToCard);

  write_multi_ = multi;
  write_deadline_us_ = Micros() + kTransferTimeoutUs;
  write_phase_ = WritePhase::kData;
  return Outcome::kOk;
}

Sdio::WriteStatus Sdio::PollWrite() {
  switch (write_phase_) {
    case WritePhase::kIdle:
      return WriteStatus::kIdle;

    case WritePhase::kData: {
      const uint32_t sta = SDIO->STA;
      if (sta & kStaDataErrors) {
        if (sta & SDIO_STA_DCRCFAIL) {
          ++stats_.data_crc_errors;
        } else {
          ++stats_.data_timeouts;
        }
        ++stats_.write_errors;
        AbortTransfer();
        return WriteStatus::kError;
      }
      if (sta & SDIO_STA_DATAEND) {
        StopDma();
        SDIO->DCTRL = 0;
        bool ok = true;
        if (write_multi_) {
          ok = Ok(SendCommand(Cmd::kStop, 0, Resp::kShort));
        }
        SDIO->ICR = kIcrAll;
        if (!ok) {
          ++stats_.write_errors;
          write_phase_ = WritePhase::kIdle;
          return WriteStatus::kError;
        }
        write_deadline_us_ = Micros() + kProgramTimeoutUs;
        write_phase_ = WritePhase::kProgramming;
        return WriteStatus::kBusy;
      }
      if (static_cast<int32_t>(Micros() - write_deadline_us_) > 0) {
        ++stats_.data_timeouts;
        ++stats_.write_errors;
        AbortTransfer();
        return WriteStatus::kError;
      }
      return WriteStatus::kBusy;
    }

    case WritePhase::kProgramming: {
      if (!Ok(SendCommand(Cmd::kSendStatus, static_cast<uint32_t>(rca_) << 16,
                               Resp::kShort))) {
        ++stats_.write_errors;
        write_phase_ = WritePhase::kIdle;
        return WriteStatus::kError;
      }
      const uint32_t status = SDIO->RESP1;
      if ((status & kCardStatusReadyForData) &&
          ((status >> 9) & 0xFu) == kCardStateTran) {
        write_phase_ = WritePhase::kIdle;
        return WriteStatus::kDone;
      }
      if (static_cast<int32_t>(Micros() - write_deadline_us_) > 0) {
        ++stats_.write_errors;
        write_phase_ = WritePhase::kIdle;
        return WriteStatus::kError;
      }
      return WriteStatus::kBusy;
    }
  }
  return WriteStatus::kIdle;
}

Outcome Sdio::WriteBlocks(uint32_t lba, std::span<const uint8_t> src) {
  const Outcome start = StartWrite(lba, src);
  if (start != Outcome::kOk) {
    return start;
  }
  while (true) {
    FeedWatchdog();
    const WriteStatus status = PollWrite();
    if (status == WriteStatus::kDone) {
      return Outcome::kOk;
    }
    // PollWrite reports progress, not cause; Stats carries which of the data
    // errors it counted on the way to kError.
    if (status != WriteStatus::kBusy) {
      return Outcome::kTimeout;
    }
  }
}
