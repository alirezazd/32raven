// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "usb_cdc.hpp"

#include <cstring>

#include "error_code.hpp"
#include "irq_priority.hpp"
#include "panic.hpp"
#include "stm32f4xx.h"
#include "system.hpp"

namespace {

// Core register windows
// The DWC2 core scatters its device registers across fixed offsets from the
// peripheral base rather than exposing one struct, so each block gets an
// accessor instead of a macro.

inline USB_OTG_GlobalTypeDef *Global() {
  return reinterpret_cast<USB_OTG_GlobalTypeDef *>(USB_OTG_FS_PERIPH_BASE);
}

inline USB_OTG_DeviceTypeDef *Device() {
  return reinterpret_cast<USB_OTG_DeviceTypeDef *>(USB_OTG_FS_PERIPH_BASE +
                                                   USB_OTG_DEVICE_BASE);
}

inline USB_OTG_INEndpointTypeDef *InEp(uint32_t n) {
  return reinterpret_cast<USB_OTG_INEndpointTypeDef *>(
      USB_OTG_FS_PERIPH_BASE + USB_OTG_IN_ENDPOINT_BASE +
      (n * USB_OTG_EP_REG_SIZE));
}

inline USB_OTG_OUTEndpointTypeDef *OutEp(uint32_t n) {
  return reinterpret_cast<USB_OTG_OUTEndpointTypeDef *>(
      USB_OTG_FS_PERIPH_BASE + USB_OTG_OUT_ENDPOINT_BASE +
      (n * USB_OTG_EP_REG_SIZE));
}

// Writes go to the target endpoint's window; every read comes out of the one
// shared RX FIFO, which is why ReadFifo always uses window 0.
inline volatile uint32_t &Fifo(uint32_t n) {
  return *reinterpret_cast<volatile uint32_t *>(
      USB_OTG_FS_PERIPH_BASE + USB_OTG_FIFO_BASE + (n * USB_OTG_FIFO_SIZE));
}

inline volatile uint32_t &PowerClockGate() {
  return *reinterpret_cast<volatile uint32_t *>(USB_OTG_FS_PERIPH_BASE +
                                                USB_OTG_PCGCCTL_BASE);
}

// Endpoints and FIFO partitioning

constexpr uint32_t kCtrlEp = 0u;
constexpr uint32_t kBulkEp = 1u;    // IN and OUT share the number
constexpr uint32_t kNotifyEp = 2u;  // IN only; CDC requires it to exist

constexpr uint16_t kEp0MaxPacket = stm32_limits::kUsbCdcEp0MaxPacketBytes;
constexpr uint16_t kBulkMaxPacket = stm32_limits::kUsbCdcBulkMaxPacketBytes;
constexpr uint16_t kNotifyMaxPacket = 8u;

// The descriptors below spell these out as literal bytes, which no expression
// can reach, so this is what catches the two drifting apart.
static_assert(kEp0MaxPacket == 0x40u);
static_assert(kBulkMaxPacket == 0x40u);

// OTG FS has 1.25 KB of FIFO RAM = 320 32-bit words, split by hand because the
// core has no allocator. The numbers must sum to <= 320 or transfers silently
// corrupt each other.
constexpr uint32_t kRxFifoWords = 128u;
constexpr uint32_t kTxFifo0Start = kRxFifoWords;
constexpr uint32_t kTxFifo0Words = 64u;
constexpr uint32_t kTxFifo1Start = kTxFifo0Start + kTxFifo0Words;
constexpr uint32_t kTxFifo1Words = 112u;
constexpr uint32_t kTxFifo2Start = kTxFifo1Start + kTxFifo1Words;
constexpr uint32_t kTxFifo2Words = 16u;
static_assert(kTxFifo2Start + kTxFifo2Words <= 320u,
              "OTG FS FIFO allocation exceeds the core's 1.25 KB of RAM");

// Protocol constants

constexpr uint8_t kReqTypeTypeMask = 0x60u;
constexpr uint8_t kReqTypeStandard = 0x00u;
constexpr uint8_t kReqTypeClass = 0x20u;

constexpr uint8_t kReqGetStatus = 0x00u;
constexpr uint8_t kReqClearFeature = 0x01u;
constexpr uint8_t kReqSetFeature = 0x03u;
constexpr uint8_t kReqSetAddress = 0x05u;
constexpr uint8_t kReqGetDescriptor = 0x06u;
constexpr uint8_t kReqGetConfiguration = 0x08u;
constexpr uint8_t kReqSetConfiguration = 0x09u;
constexpr uint8_t kReqGetInterface = 0x0Au;
constexpr uint8_t kReqSetInterface = 0x0Bu;

constexpr uint8_t kDescDevice = 0x01u;
constexpr uint8_t kDescConfiguration = 0x02u;
constexpr uint8_t kDescString = 0x03u;

constexpr uint8_t kCdcSetLineCoding = 0x20u;
constexpr uint8_t kCdcGetLineCoding = 0x21u;
constexpr uint8_t kCdcSetControlLineState = 0x22u;
constexpr uint16_t kCdcControlLineDtr = 0x0001u;

// GRXSTSP packet-status codes (RM0090 §34.16.4).
constexpr uint32_t kPktStsOutData = 0x02u;
constexpr uint32_t kPktStsSetupData = 0x06u;

// PHY turnaround, in PHY clocks. The reset default of 2 corrupts control
// transfers before enumeration finishes; ST's table gives 6 above 32 MHz.
constexpr uint32_t kTurnaroundTime = 6u;

constexpr uint32_t kResetSpinLimit = 200000u;

// These run inside the OTG ISR, so the boot-time limit would let a wedged core
// hold off the watchdog feed for tens of milliseconds.
constexpr uint32_t kFifoFlushSpinLimit = 2000u;
constexpr uint32_t kModeSettleUs = 50000u;

// Well above the 1 ms frame period, far below the UI's status cadence.
constexpr uint32_t kHostSilenceTimeoutUs = 50000u;

// Descriptors

// Device descriptor. VID/PID are patched in from UsbCdcConfig at Init.
uint8_t g_device_desc[] = {
    18u,   kDescDevice, 0x00u, 0x02u,  // bcdUSB 2.00
    0x02u,         // bDeviceClass: CDC, matching a plain ST virtual COM port
    0x00u, 0x00u,  // subclass, protocol
    0x40u,         // bMaxPacketSize0
    0x00u, 0x00u,  // idVendor  (patched)
    0x00u, 0x00u,  // idProduct (patched)
    0x00u, 0x02u,  // bcdDevice 2.00
    0x01u, 0x02u,       0x03u,  // iManufacturer, iProduct, iSerialNumber
    0x01u,                      // bNumConfigurations
};

// Single CDC-ACM configuration: a control interface plus a data interface.
const uint8_t kConfigDesc[] = {
    // Configuration: 67 bytes total, 2 interfaces, bus-powered, 250 mA
    9u,
    kDescConfiguration,
    67u,
    0x00u,
    2u,
    1u,
    0u,
    0x80u,
    125u,

    // Interface 0: CDC communications, ACM subclass, AT-command protocol
    9u,
    0x04u,
    0u,
    0u,
    1u,
    0x02u,
    0x02u,
    0x01u,
    0u,

    // CDC header functional, version 1.10
    5u,
    0x24u,
    0x00u,
    0x10u,
    0x01u,
    // CDC call management: no call management over the data interface
    5u,
    0x24u,
    0x01u,
    0x00u,
    0x01u,
    // CDC ACM functional: supports line coding / control line state
    4u,
    0x24u,
    0x02u,
    0x02u,
    // CDC union: interface 0 controls interface 1
    5u,
    0x24u,
    0x06u,
    0x00u,
    0x01u,

    // Never written to, but Linux refuses to bind without it.
    7u,
    0x05u,
    0x82u,
    0x03u,
    0x08u,
    0x00u,
    0x10u,

    // Interface 1: CDC data
    9u,
    0x04u,
    1u,
    0u,
    2u,
    0x0Au,
    0x00u,
    0x00u,
    0u,
    // Endpoint 1 OUT, bulk, 64 bytes
    7u,
    0x05u,
    0x01u,
    0x02u,
    0x40u,
    0x00u,
    0x00u,
    // Endpoint 1 IN, bulk, 64 bytes
    7u,
    0x05u,
    0x81u,
    0x02u,
    0x40u,
    0x00u,
    0x00u,
};
static_assert(sizeof(kConfigDesc) == 67u,
              "wTotalLength in the configuration descriptor must match its "
              "actual byte count");

const uint8_t kLangIdDesc[] = {4u, kDescString, 0x09u, 0x04u};  // en-US

char HexDigit(uint32_t nibble) {
  return static_cast<char>(nibble < 10u ? ('0' + nibble)
                                        : ('A' + nibble - 10u));
}

}  // namespace

UsbCdc &UsbCdc::GetInstance() {
  static UsbCdc instance;
  return instance;
}

void UsbCdc::Init(const UsbCdcConfig &config) {
  if (initialized_) {
    Panic(ErrorCode::Stm32::kUsbReinit);
  }

  cfg_ = config;
  g_device_desc[8] = static_cast<uint8_t>(cfg_.vendor_id);
  g_device_desc[9] = static_cast<uint8_t>(cfg_.vendor_id >> 8);
  g_device_desc[10] = static_cast<uint8_t>(cfg_.product_id);
  g_device_desc[11] = static_cast<uint8_t>(cfg_.product_id >> 8);

  CoreInit();
  DeviceInit();

  NVIC_SetPriority(OTG_FS_IRQn, irq_priority::kUsbOtgFs);
  NVIC_EnableIRQ(OTG_FS_IRQn);

  // Pull-up stays off; SetAttached is what presents us to a host.
  initialized_ = true;
}

void UsbCdc::Poll(uint32_t now_us) {
  const uint32_t frame =
      (Device()->DSTS & USB_OTG_DSTS_FNSOF_Msk) >> USB_OTG_DSTS_FNSOF_Pos;

  if (frame != last_frame_number_) {
    last_frame_number_ = frame;
    last_frame_change_us_ = now_us;
  }

  // Back-to-back polls legitimately land inside one 1 ms frame.
  host_present_ = (now_us - last_frame_change_us_) <= kHostSilenceTimeoutUs;
}

// Software level plug or unplug by switching the core's built-in D+ pull-up.
void UsbCdc::SetAttached(bool attached) {
  if (attached == attached_) {
    return;
  }
  attached_ = attached;

  if (attached) {
    Device()->DCTL &= ~USB_OTG_DCTL_SDIS;
    return;
  }

  Device()->DCTL |= USB_OTG_DCTL_SDIS;
  // The ISR owns tx_ring_'s tail and rx_ring_'s head, so a Clear() landing
  // mid-PumpTx would put tail behind head and stream the old session's bytes.
  NVIC_DisableIRQ(OTG_FS_IRQn);
  // The next attach must look like a fresh plug-in, not a resumed session.
  configured_ = false;
  dtr_ = false;
  tx_in_flight_ = false;
  tx_zlp_pending_ = false;
  ctrl_out_is_line_coding_ = false;
  current_configuration_ = 0;
  bulk_out_stalled_ = false;
  rx_ring_.Clear();
  tx_ring_.Clear();
  NVIC_EnableIRQ(OTG_FS_IRQn);
}

// Clocks the OTG core, resets it, and forces it into device mode.
void UsbCdc::CoreInit() {
  RCC->AHB2ENR |= RCC_AHB2ENR_OTGFSEN;
  // The enable takes a few bus cycles to land; reading it back covers the gap.
  (void)RCC->AHB2ENR;

  // Selecting the PHY before the reset is required — the reset is what latches
  // the choice into the core.
  Global()->GUSBCFG |= USB_OTG_GUSBCFG_PHYSEL;

  // The core only accepts a reset once its AHB master is idle.
  uint32_t guard = kResetSpinLimit;
  while ((Global()->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL) == 0u) {
    if (--guard == 0u) {
      Panic(ErrorCode::Stm32::kUsbInitFailed);
    }
  }
  Global()->GRSTCTL |= USB_OTG_GRSTCTL_CSRST;
  guard = kFifoFlushSpinLimit;
  while ((Global()->GRSTCTL & USB_OTG_GRSTCTL_CSRST) != 0u) {
    if (--guard == 0u) {
      Panic(ErrorCode::Stm32::kUsbInitFailed);
    }
  }

  // PA9 (VBUS) and PA10 (ID) belong to USART1, so NOVBUSSENS stops the core
  // waiting for a session it can never see, and device mode replaces ID.
  // PWRDWN reads backwards: setting it powers the transceiver up.
  Global()->GCCFG = USB_OTG_GCCFG_PWRDWN | USB_OTG_GCCFG_NOVBUSSENS;
  Global()->GUSBCFG = (Global()->GUSBCFG & ~USB_OTG_GUSBCFG_TRDT) |
                      (kTurnaroundTime << USB_OTG_GUSBCFG_TRDT_Pos);
  Global()->GUSBCFG |= USB_OTG_GUSBCFG_FDMOD;
  // Device registers read stale host-mode values until the role change settles.
  System::GetInstance().Time().DelayMicros(kModeSettleUs);
}

// Configures the device role and arms the interrupt that drives everything
// after it: speed, FIFO partitioning, endpoint reset, then interrupts on.
void UsbCdc::DeviceInit() {
  PowerClockGate() = 0u;

  // Pull-up held off until configured, so no host enumerates a half-built
  // endpoint set.
  Device()->DCFG |= USB_OTG_DCFG_DSPD;
  Device()->DCTL |= USB_OTG_DCTL_SDIS;

  // Encoding is (size_in_words << 16) | start_in_words; the regions themselves
  // are laid out by the kRxFifo/kTxFifo constants at the top of the file.
  Global()->GRXFSIZ = kRxFifoWords;
  Global()->DIEPTXF0_HNPTXFSIZ = (kTxFifo0Words << 16) | kTxFifo0Start;
  Global()->DIEPTXF[0] = (kTxFifo1Words << 16) | kTxFifo1Start;
  Global()->DIEPTXF[1] = (kTxFifo2Words << 16) | kTxFifo2Start;

  FlushTxFifo(0x10u);  // 0x10 = all transmit FIFOs
  FlushRxFifo();

  // OpenDataEndpoints re-enables only what each endpoint needs.
  Device()->DIEPMSK = 0u;
  Device()->DOEPMSK = 0u;
  Device()->DAINTMSK = 0u;

  // A warm reset leaves transfer sizes and interrupt flags from the last
  // session behind, which would corrupt the first new transfer.
  for (uint32_t ep = 0u; ep <= 3u; ++ep) {
    InEp(ep)->DIEPCTL = 0u;
    InEp(ep)->DIEPTSIZ = 0u;
    InEp(ep)->DIEPINT = 0xFB7Fu;  // write-1-to-clear, every defined flag
    OutEp(ep)->DOEPCTL = 0u;
    OutEp(ep)->DOEPTSIZ = 0u;
    OutEp(ep)->DOEPINT = 0xFB7Fu;
  }

  // Not 0xFFFFFFFF: bit 30 is read-only.
  Global()->GINTSTS = 0xBFFFFFFFu;
  Global()->GINTMSK = USB_OTG_GINTMSK_USBRST | USB_OTG_GINTMSK_ENUMDNEM |
                      USB_OTG_GINTMSK_RXFLVLM | USB_OTG_GINTMSK_IEPINT |
                      USB_OTG_GINTMSK_OEPINT | USB_OTG_GINTMSK_USBSUSPM |
                      USB_OTG_GINTMSK_WUIM;
  // Master switch: until this, the flags above set but no interrupt fires.
  Global()->GAHBCFG |= USB_OTG_GAHBCFG_GINT;
}

// Inside the ISR, so a wedged core is abandoned rather than waited out: the
// following bus reset clears the dirty FIFO anyway.
void UsbCdc::FlushFifo(uint32_t rstctl_write, uint32_t busy_mask) {
  uint32_t guard = kFifoFlushSpinLimit;
  while ((Global()->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL) == 0u) {
    if (--guard == 0u) {
      return;
    }
  }
  Global()->GRSTCTL = rstctl_write;
  guard = kFifoFlushSpinLimit;
  while ((Global()->GRSTCTL & busy_mask) != 0u) {
    if (--guard == 0u) {
      return;
    }
  }
}

void UsbCdc::FlushTxFifo(uint32_t fifo_num) {
  FlushFifo(USB_OTG_GRSTCTL_TXFFLSH | (fifo_num << USB_OTG_GRSTCTL_TXFNUM_Pos),
            USB_OTG_GRSTCTL_TXFFLSH);
}

void UsbCdc::FlushRxFifo() {
  FlushFifo(USB_OTG_GRSTCTL_RXFFLSH, USB_OTG_GRSTCTL_RXFFLSH);
}

// Interrupt dispatch

void UsbCdc::IrqHandler() {
  const uint32_t status = Global()->GINTSTS & Global()->GINTMSK;
  if (status == 0u) {
    return;
  }

  if (status & USB_OTG_GINTSTS_RXFLVL) {
    OnRxFifoLevel();
  }
  if (status & USB_OTG_GINTSTS_USBRST) {
    Global()->GINTSTS = USB_OTG_GINTSTS_USBRST;
    OnReset();
  }
  if (status & USB_OTG_GINTSTS_ENUMDNE) {
    Global()->GINTSTS = USB_OTG_GINTSTS_ENUMDNE;
    OnEnumDone();
  }
  if (status & USB_OTG_GINTSTS_OEPINT) {
    OnOutEndpoint();
  }
  if (status & USB_OTG_GINTSTS_IEPINT) {
    OnInEndpoint();
  }
  // Acknowledged but not recorded: host presence comes from the frame counter,
  // and dtr_ deliberately survives a suspend -- a host that parks an idle port
  // has not closed it, and will not re-send SET_CONTROL_LINE_STATE on resume.
  if (status & USB_OTG_GINTSTS_USBSUSP) {
    Global()->GINTSTS = USB_OTG_GINTSTS_USBSUSP;
  }
  if (status & USB_OTG_GINTSTS_WKUINT) {
    Global()->GINTSTS = USB_OTG_GINTSTS_WKUINT;
  }
}

void UsbCdc::OnReset() {
  ++reset_count_;
  ctrl_out_is_line_coding_ = false;
  configured_ = false;
  dtr_ = false;
  tx_in_flight_ = false;
  tx_zlp_pending_ = false;

  Device()->DCTL &= ~USB_OTG_DCTL_RWUSIG;
  FlushTxFifo(0x10u);

  for (uint32_t ep = 0u; ep <= 3u; ++ep) {
    InEp(ep)->DIEPINT = 0xFB7Fu;
    OutEp(ep)->DOEPINT = 0xFB7Fu;
  }

  Device()->DAINTMSK = (1u << kCtrlEp) | (1u << (kCtrlEp + 16u));
  Device()->DOEPMSK =
      USB_OTG_DOEPMSK_STUPM | USB_OTG_DOEPMSK_XFRCM | USB_OTG_DOEPMSK_EPDM;
  Device()->DIEPMSK =
      USB_OTG_DIEPMSK_XFRCM | USB_OTG_DIEPMSK_TOM | USB_OTG_DIEPMSK_EPDM;

  Device()->DCFG &= ~USB_OTG_DCFG_DAD;
  PrepareEp0Setup();
}

void UsbCdc::OnEnumDone() {
  InEp(kCtrlEp)->DIEPCTL &= ~USB_OTG_DIEPCTL_MPSIZ;  // MPSIZ 0 = 64 bytes
  Device()->DCTL |= USB_OTG_DCTL_CGINAK;
}

void UsbCdc::OnRxFifoLevel() {
  Global()->GINTMSK &= ~USB_OTG_GINTMSK_RXFLVLM;

  const uint32_t status = Global()->GRXSTSP;
  const uint32_t ep = status & USB_OTG_GRXSTSP_EPNUM;
  const uint16_t count =
      static_cast<uint16_t>((status & USB_OTG_GRXSTSP_BCNT) >> 4);
  const uint32_t pktsts =
      (status & USB_OTG_GRXSTSP_PKTSTS) >> USB_OTG_GRXSTSP_PKTSTS_Pos;

  if (pktsts == kPktStsSetupData && count == sizeof(SetupPacket)) {
    ReadFifo(reinterpret_cast<uint8_t *>(&setup_), count);
  } else if (pktsts == kPktStsOutData && count > 0u) {
    if (ep == kCtrlEp) {
      const uint16_t len =
          count > sizeof(ctrl_out_buf_) ? sizeof(ctrl_out_buf_) : count;
      ReadFifo(ctrl_out_buf_, len);
    } else {
      const uint16_t len =
          count > sizeof(bulk_out_buf_) ? sizeof(bulk_out_buf_) : count;
      ReadFifo(bulk_out_buf_, len);
      rx_ring_.PushBlock(bulk_out_buf_, len);
    }
  } else if (count > 0u) {
    // Status-only entries carry no payload; anything else with a byte count
    // must still be drained or the FIFO read pointer desynchronises.
    uint8_t discard[4];
    for (uint16_t i = 0u; i < count; i = static_cast<uint16_t>(i + 4u)) {
      const uint16_t left = static_cast<uint16_t>(count - i);
      ReadFifo(discard, left > 4u ? 4u : left);
    }
  }

  Global()->GINTMSK |= USB_OTG_GINTMSK_RXFLVLM;
}

void UsbCdc::OnOutEndpoint() {
  uint32_t pending = (Device()->DAINT & Device()->DAINTMSK) >> 16;
  for (uint32_t ep = 0u; pending != 0u && ep <= 3u; ++ep, pending >>= 1) {
    if ((pending & 1u) == 0u) {
      continue;
    }
    const uint32_t flags = OutEp(ep)->DOEPINT;
    // Same reasoning as the IN side: unacknowledged bits latch the level-driven
    // OEPINT and wedge the interrupt on.
    OutEp(ep)->DOEPINT = flags;

    if (flags & USB_OTG_DOEPINT_STUP) {
      HandleSetup();
    }
    if (flags & USB_OTG_DOEPINT_XFRC) {
      if (ep == kCtrlEp) {
        if (ctrl_out_is_line_coding_) {
          ctrl_out_is_line_coding_ = false;
          std::memcpy(&line_coding_, ctrl_out_buf_, sizeof(line_coding_));
          ControlSendZlp();
        }
        PrepareEp0Setup();
      } else {
        ArmBulkOut();
      }
    }
  }
}

void UsbCdc::OnInEndpoint() {
  uint32_t pending = Device()->DAINT & Device()->DAINTMSK & 0xFFFFu;
  for (uint32_t ep = 0u; pending != 0u && ep <= 3u; ++ep, pending >>= 1) {
    if ((pending & 1u) == 0u) {
      continue;
    }
    const uint32_t flags = InEp(ep)->DIEPINT;
    // Acknowledge everything up front, including bits we take no action on.
    // DAINT and GINTSTS.IEPINT are level flags derived from DIEPINT, so a bit
    // left standing (a timeout on an abandoned control transfer, say) re-enters
    // this handler forever and starves thread mode into a watchdog reset.
    InEp(ep)->DIEPINT = flags;

    if (flags & USB_OTG_DIEPINT_XFRC) {
      if (ep == kCtrlEp) {
        if (ctrl_in_remaining_ > 0u || ctrl_in_zlp_) {
          if (ctrl_in_remaining_ == 0u) {
            ctrl_in_zlp_ = false;
          }
          ControlInPacket();
        } else {
          PrepareEp0Setup();
        }
      } else if (ep == kBulkEp) {
        tx_in_flight_ = false;
        // A maximum-size packet does not end the host's bulk transfer; without
        // a following zero-length packet the data can sit in the host's buffer
        // until the next write happens to arrive.
        if (last_tx_len_ == kBulkMaxPacket && tx_ring_.IsEmpty()) {
          tx_zlp_pending_ = true;
        }
        PumpTx();
      }
    }
  }
}

// FIFO access

void UsbCdc::ReadFifo(uint8_t *dest, uint16_t len) {
  const uint16_t words = static_cast<uint16_t>(len / 4u);
  for (uint16_t i = 0u; i < words; ++i) {
    const uint32_t value = Fifo(0u);
    dest[0] = static_cast<uint8_t>(value);
    dest[1] = static_cast<uint8_t>(value >> 8);
    dest[2] = static_cast<uint8_t>(value >> 16);
    dest[3] = static_cast<uint8_t>(value >> 24);
    dest += 4;
  }
  const uint16_t tail = static_cast<uint16_t>(len % 4u);
  if (tail != 0u) {
    uint32_t value = Fifo(0u);
    for (uint16_t i = 0u; i < tail; ++i) {
      dest[i] = static_cast<uint8_t>(value);
      value >>= 8;
    }
  }
}

void UsbCdc::WriteFifo(uint8_t ep_num, const uint8_t *src, uint16_t len) {
  const uint16_t words = static_cast<uint16_t>((len + 3u) / 4u);
  for (uint16_t i = 0u; i < words; ++i) {
    uint32_t value = 0u;
    const uint16_t offset = static_cast<uint16_t>(i * 4u);
    const uint16_t left = static_cast<uint16_t>(len - offset);
    const uint16_t chunk = left > 4u ? 4u : left;
    for (uint16_t b = 0u; b < chunk; ++b) {
      value |= static_cast<uint32_t>(src[offset + b]) << (8u * b);
    }
    Fifo(ep_num) = value;
  }
}

// Control transfers

void UsbCdc::PrepareEp0Setup() {
  OutEp(kCtrlEp)->DOEPTSIZ = (1u << USB_OTG_DOEPTSIZ_PKTCNT_Pos) |
                             (3u * sizeof(SetupPacket)) |
                             USB_OTG_DOEPTSIZ_STUPCNT;
  // Hardware accepts SETUP packets unconditionally, but the status stage that
  // ends a control transfer is an ordinary OUT packet: without enabling the
  // endpoint and clearing NAK here, every transfer is answered and then hangs
  // on its status stage until the host gives up and resets.
  OutEp(kCtrlEp)->DOEPCTL |= USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_CNAK;
}

void UsbCdc::PrepareEp0Out(uint16_t len) {
  OutEp(kCtrlEp)->DOEPTSIZ =
      (1u << USB_OTG_DOEPTSIZ_PKTCNT_Pos) | static_cast<uint32_t>(len);
  OutEp(kCtrlEp)->DOEPCTL |= USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_CNAK;
}

void UsbCdc::ControlInPacket() {
  const uint16_t chunk =
      ctrl_in_remaining_ > kEp0MaxPacket ? kEp0MaxPacket : ctrl_in_remaining_;
  InEp(kCtrlEp)->DIEPTSIZ =
      (1u << USB_OTG_DIEPTSIZ_PKTCNT_Pos) | static_cast<uint32_t>(chunk);
  InEp(kCtrlEp)->DIEPCTL |= USB_OTG_DIEPCTL_EPENA | USB_OTG_DIEPCTL_CNAK;
  if (chunk > 0u) {
    WriteFifo(kCtrlEp, ctrl_in_ptr_, chunk);
    ctrl_in_ptr_ += chunk;
    ctrl_in_remaining_ = static_cast<uint16_t>(ctrl_in_remaining_ - chunk);
  }
}

void UsbCdc::ControlSend(const uint8_t *data, uint16_t len) {
  if (len > setup_.w_length) {
    len = setup_.w_length;
  }
  ctrl_in_ptr_ = data;
  ctrl_in_remaining_ = len;
  // A reply shorter than the host asked for must end on a short packet. When
  // the truncated length lands exactly on a packet boundary, an explicit
  // zero-length packet is the only way to signal the end.
  ctrl_in_zlp_ =
      (len < setup_.w_length) && (len != 0u) && ((len % kEp0MaxPacket) == 0u);
  ControlInPacket();
}

void UsbCdc::ControlSendZlp() {
  ctrl_in_ptr_ = nullptr;
  ctrl_in_remaining_ = 0u;
  ctrl_in_zlp_ = false;
  InEp(kCtrlEp)->DIEPTSIZ = (1u << USB_OTG_DIEPTSIZ_PKTCNT_Pos);
  InEp(kCtrlEp)->DIEPCTL |= USB_OTG_DIEPCTL_EPENA | USB_OTG_DIEPCTL_CNAK;
}

void UsbCdc::ControlStall() {
  InEp(kCtrlEp)->DIEPCTL |= USB_OTG_DIEPCTL_STALL;
  OutEp(kCtrlEp)->DOEPCTL |= USB_OTG_DOEPCTL_STALL;
  PrepareEp0Setup();
}

void UsbCdc::HandleSetup() {
  const SetupPacket setup = setup_;
  const uint8_t type = setup.bm_request_type & kReqTypeTypeMask;

  bool handled = false;
  if (type == kReqTypeStandard) {
    handled = HandleStandardRequest(setup);
  } else if (type == kReqTypeClass) {
    handled = HandleClassRequest(setup);
  }

  if (!handled) {
    ControlStall();
  }
}

bool UsbCdc::HandleStandardRequest(const SetupPacket &setup) {
  switch (setup.b_request) {
    case kReqGetDescriptor: {
      const uint8_t desc_type = static_cast<uint8_t>(setup.w_value >> 8);
      const uint8_t desc_index = static_cast<uint8_t>(setup.w_value);
      if (desc_type == kDescDevice) {
        ControlSend(g_device_desc, sizeof(g_device_desc));
        return true;
      }
      if (desc_type == kDescConfiguration) {
        ControlSend(kConfigDesc, sizeof(kConfigDesc));
        return true;
      }
      if (desc_type == kDescString) {
        if (desc_index == 0u) {
          ControlSend(kLangIdDesc, sizeof(kLangIdDesc));
          return true;
        }
        const uint16_t len = BuildStringDescriptor(desc_index);
        if (len == 0u) {
          return false;
        }
        ControlSend(string_desc_buf_, len);
        return true;
      }
      // Device qualifier and anything else: this is a full-speed-only device,
      // and stalling is how the host is told so.
      return false;
    }

    case kReqSetAddress:
      if (setup.w_value > 127u) {
        return false;
      }
      Device()->DCFG =
          (Device()->DCFG & ~USB_OTG_DCFG_DAD) |
          (static_cast<uint32_t>(setup.w_value) << USB_OTG_DCFG_DAD_Pos);
      ControlSendZlp();
      return true;

    case kReqSetConfiguration:
      if (setup.w_value == 0u) {
        configured_ = false;
        CloseDataEndpoints();
      } else {
        OpenDataEndpoints();
        configured_ = true;
      }
      ControlSendZlp();
      return true;

    case kReqGetConfiguration: {
      current_configuration_ = configured_ ? 1u : 0u;
      ControlSend(&current_configuration_, 1u);
      return true;
    }

    case kReqGetStatus: {
      // Bus-powered, no remote wakeup — a constant for every recipient we
      // expose.
      static const uint8_t kDeviceStatus[2] = {0u, 0u};
      ControlSend(kDeviceStatus, sizeof(kDeviceStatus));
      return true;
    }

    case kReqGetInterface: {
      static const uint8_t kAltSetting = 0u;
      ControlSend(&kAltSetting, 1u);
      return true;
    }

    case kReqSetInterface:
      ControlSendZlp();
      return true;

    case kReqClearFeature:
    case kReqSetFeature:
      ControlSendZlp();
      return true;

    default:
      return false;
  }
}

bool UsbCdc::HandleClassRequest(const SetupPacket &setup) {
  switch (setup.b_request) {
    case kCdcSetLineCoding:
      if (setup.w_length != sizeof(LineCoding)) {
        return false;
      }
      ctrl_out_is_line_coding_ = true;
      PrepareEp0Out(sizeof(LineCoding));
      return true;

    case kCdcGetLineCoding:
      ControlSend(reinterpret_cast<const uint8_t *>(&line_coding_),
                  sizeof(line_coding_));
      return true;

    case kCdcSetControlLineState:
      dtr_ = (setup.w_value & kCdcControlLineDtr) != 0u;
      ControlSendZlp();
      return true;

    default:
      return false;
  }
}

uint16_t UsbCdc::BuildStringDescriptor(uint8_t index) {
  const char *text = nullptr;
  char serial[25];

  if (index == 1u) {
    text = cfg_.manufacturer;
  } else if (index == 2u) {
    text = cfg_.product;
  } else if (index == 3u) {
    const volatile uint32_t *uid =
        reinterpret_cast<const volatile uint32_t *>(UID_BASE);
    size_t pos = 0u;
    for (uint32_t word = 0u; word < 3u; ++word) {
      const uint32_t value = uid[word];
      for (int32_t nibble = 7; nibble >= 0; --nibble) {
        serial[pos++] = HexDigit((value >> (nibble * 4)) & 0xFu);
      }
    }
    serial[pos] = '\0';
    text = serial;
  }

  if (text == nullptr) {
    return 0u;
  }

  size_t len = std::strlen(text);
  const size_t max_chars = (sizeof(string_desc_buf_) - 2u) / 2u;
  if (len > max_chars) {
    len = max_chars;
  }

  string_desc_buf_[0] = static_cast<uint8_t>(2u + (len * 2u));
  string_desc_buf_[1] = kDescString;
  for (size_t i = 0u; i < len; ++i) {
    string_desc_buf_[2u + (i * 2u)] = static_cast<uint8_t>(text[i]);
    string_desc_buf_[3u + (i * 2u)] = 0u;
  }
  return string_desc_buf_[0];
}

// Data endpoints

void UsbCdc::OpenDataEndpoints() {
  InEp(kBulkEp)->DIEPCTL = USB_OTG_DIEPCTL_USBAEP |
                           (2u << USB_OTG_DIEPCTL_EPTYP_Pos) |  // bulk
                           (kBulkEp << USB_OTG_DIEPCTL_TXFNUM_Pos) |
                           USB_OTG_DIEPCTL_SD0PID_SEVNFRM | kBulkMaxPacket;

  InEp(kNotifyEp)->DIEPCTL = USB_OTG_DIEPCTL_USBAEP |
                             (3u << USB_OTG_DIEPCTL_EPTYP_Pos) |  // interrupt
                             (kNotifyEp << USB_OTG_DIEPCTL_TXFNUM_Pos) |
                             USB_OTG_DIEPCTL_SD0PID_SEVNFRM | kNotifyMaxPacket;

  OutEp(kBulkEp)->DOEPCTL = USB_OTG_DOEPCTL_USBAEP |
                            (2u << USB_OTG_DOEPCTL_EPTYP_Pos) |  // bulk
                            USB_OTG_DOEPCTL_SD0PID_SEVNFRM | kBulkMaxPacket;

  Device()->DAINTMSK |=
      (1u << kBulkEp) | (1u << kNotifyEp) | (1u << (kBulkEp + 16u));

  tx_in_flight_ = false;
  tx_zlp_pending_ = false;
  ArmBulkOut();
}

void UsbCdc::CloseDataEndpoints() {
  Device()->DAINTMSK &=
      ~((1u << kBulkEp) | (1u << kNotifyEp) | (1u << (kBulkEp + 16u)));
  InEp(kBulkEp)->DIEPCTL &= ~USB_OTG_DIEPCTL_USBAEP;
  InEp(kNotifyEp)->DIEPCTL &= ~USB_OTG_DIEPCTL_USBAEP;
  OutEp(kBulkEp)->DOEPCTL &= ~USB_OTG_DOEPCTL_USBAEP;
}

// Re-arming unconditionally would make the device ACK packets it has nowhere to
// put, punching holes into the middle of a frame that the parser then reads as
// payload. Leaving the endpoint unarmed NAKs instead, so the host retries and
// the sender sees real back-pressure.
void UsbCdc::ArmBulkOut() {
  if ((rx_ring_.Capacity() - rx_ring_.Available()) < kBulkMaxPacket) {
    bulk_out_stalled_ = true;
    return;
  }
  bulk_out_stalled_ = false;
  OutEp(kBulkEp)->DOEPTSIZ =
      (1u << USB_OTG_DOEPTSIZ_PKTCNT_Pos) | kBulkMaxPacket;
  OutEp(kBulkEp)->DOEPCTL |= USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_CNAK;
}

void UsbCdc::PumpTx() {
  if (!configured_ || tx_in_flight_) {
    return;
  }

  if (tx_zlp_pending_) {
    tx_zlp_pending_ = false;
    tx_in_flight_ = true;
    last_tx_len_ = 0u;
    InEp(kBulkEp)->DIEPTSIZ = (1u << USB_OTG_DIEPTSIZ_PKTCNT_Pos);
    InEp(kBulkEp)->DIEPCTL |= USB_OTG_DIEPCTL_EPENA | USB_OTG_DIEPCTL_CNAK;
    return;
  }

  const uint8_t *src = nullptr;
  size_t avail = tx_ring_.ContiguousReadable(src);
  if (avail == 0u) {
    return;
  }
  if (avail > kBulkMaxPacket) {
    avail = kBulkMaxPacket;
  }

  const uint32_t words_needed = (static_cast<uint32_t>(avail) + 3u) / 4u;
  if ((InEp(kBulkEp)->DTXFSTS & USB_OTG_DTXFSTS_INEPTFSAV) < words_needed) {
    return;
  }

  tx_in_flight_ = true;
  last_tx_len_ = static_cast<uint16_t>(avail);
  InEp(kBulkEp)->DIEPTSIZ =
      (1u << USB_OTG_DIEPTSIZ_PKTCNT_Pos) | static_cast<uint32_t>(avail);
  InEp(kBulkEp)->DIEPCTL |= USB_OTG_DIEPCTL_EPENA | USB_OTG_DIEPCTL_CNAK;
  WriteFifo(kBulkEp, src, static_cast<uint16_t>(avail));
  tx_ring_.Consume(avail);
}

// Public data path

size_t UsbCdc::Send(const uint8_t *data, size_t len) {
  if (data == nullptr || len == 0u || !configured_) {
    return 0u;
  }
  const size_t written = tx_ring_.PushBlock(data, len);

  NVIC_DisableIRQ(OTG_FS_IRQn);
  PumpTx();
  NVIC_EnableIRQ(OTG_FS_IRQn);

  return written;
}

bool UsbCdc::Read(uint8_t &out) {
  const bool got = rx_ring_.Pop(out);
  // Space only reappears here, so this is where a NAKing endpoint gets revived.
  if (got && bulk_out_stalled_ && configured_) {
    NVIC_DisableIRQ(OTG_FS_IRQn);
    ArmBulkOut();
    NVIC_EnableIRQ(OTG_FS_IRQn);
  }
  return got;
}

extern "C" void UsbCdcOnIrq(void) { UsbCdc::GetInstance().IrqHandler(); }
