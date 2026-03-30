#include "ee.hpp"

#include <cstring>

#include "error_code.hpp"
#include "panic.hpp"
#include "stm32f4xx_hal.h"

namespace {

constexpr uint32_t kFlashEraseNoError = 0xFFFFFFFFu;
constexpr uint32_t kSlotMagic = 0x31524545u;  // "EER1"

struct SlotHeader {
  uint32_t magic;
  uint32_t sequence;
  uint32_t crc32;
  uint32_t reserved;
};

static_assert(sizeof(SlotHeader) == EE::kHeaderSize);

struct SlotDesc {
  uint32_t address;
  uint32_t sector;
};

struct SlotState {
  SlotDesc desc;
  SlotHeader header;
  bool valid;
};

constexpr SlotDesc kSlot1 = {
    .address = EE::kSector1Address,
    .sector = FLASH_SECTOR_1,
};

constexpr SlotDesc kSlot2 = {
    .address = EE::kSector2Address,
    .sector = FLASH_SECTOR_2,
};

const uint8_t *FlashBytes(uint32_t address) {
  return reinterpret_cast<const uint8_t *>(address);
}

uint32_t UpdateCrc32(uint32_t crc, uint8_t byte) {
  crc ^= byte;
  for (int i = 0; i < 8; ++i) {
    const uint32_t mask = -(crc & 1u);
    crc = (crc >> 1) ^ (0xEDB88320u & mask);
  }
  return crc;
}

uint32_t Crc32(const uint8_t *data, uint32_t len) {
  uint32_t crc = 0xFFFFFFFFu;
  for (uint32_t i = 0; i < len; ++i) {
    crc = UpdateCrc32(crc, data[i]);
  }
  return ~crc;
}

bool EraseSlot(const SlotDesc &slot) {
  FLASH_EraseInitTypeDef erase{};
  uint32_t erase_error = kFlashEraseNoError;

#if defined(STM32F405xx) || defined(STM32F407xx)
  erase.TypeErase = FLASH_TYPEERASE_SECTORS;
  erase.Sector = slot.sector;
  erase.NbSectors = 1;
#else
#error "Unsupported STM32 family for EE driver"
#endif

#if defined(FLASH_BANK_1) || defined(FLASH_BANK_2)
  erase.Banks = FLASH_BANK_1;
#endif
#if defined(FLASH_VOLTAGE_RANGE_3)
  erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
#endif

  return HAL_FLASHEx_Erase(&erase, &erase_error) == HAL_OK &&
         erase_error == kFlashEraseNoError;
}

uint32_t PayloadAddress(const SlotDesc &slot) {
  return slot.address + sizeof(SlotHeader);
}

bool ProgramBytes(uint32_t address, const uint8_t *src, uint32_t len) {
  for (uint32_t i = 0; i < len; ++i) {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, address + i, src[i]) !=
        HAL_OK) {
      return false;
    }
  }
  return true;
}

SlotHeader ReadHeader(const SlotDesc &slot) {
  SlotHeader header{};
  std::memcpy(&header, FlashBytes(slot.address), sizeof(header));
  return header;
}

bool HeaderIsBlank(const SlotHeader &header) {
  return header.magic == 0xFFFFFFFFu && header.sequence == 0xFFFFFFFFu &&
         header.crc32 == 0xFFFFFFFFu && header.reserved == 0xFFFFFFFFu;
}

bool ValidateSlot(const SlotDesc &slot, const SlotHeader &header) {
  if (HeaderIsBlank(header)) {
    return false;
  }
  if (header.magic != kSlotMagic) {
    return false;
  }
  return header.crc32 == Crc32(FlashBytes(PayloadAddress(slot)), EE::kCapacity);
}

SlotState ReadSlotState(const SlotDesc &slot) {
  const SlotHeader header = ReadHeader(slot);
  return {
      .desc = slot,
      .header = header,
      .valid = ValidateSlot(slot, header),
  };
}

const SlotState &SelectNewestSlot(const SlotState &a, const SlotState &b) {
  return (a.header.sequence >= b.header.sequence) ? a : b;
}

const SlotState *FindActiveSlot(const SlotState &a, const SlotState &b) {
  if (a.valid && b.valid) {
    return &SelectNewestSlot(a, b);
  }
  if (a.valid) {
    return &a;
  }
  if (b.valid) {
    return &b;
  }
  return nullptr;
}

const SlotDesc &InactiveSlot(const SlotState *active) {
  if (active == nullptr) {
    return kSlot1;
  }
  return (active->desc.address == kSlot1.address) ? kSlot2 : kSlot1;
}

bool VerifySlotPayload(const SlotDesc &slot, const uint8_t *expected) {
  return std::memcmp(FlashBytes(PayloadAddress(slot)), expected,
                     EE::kCapacity) == 0;
}

bool BufferIsErased(const uint8_t *data, size_t len) {
  for (size_t i = 0; i < len; ++i) {
    if (data[i] != 0xFFu) {
      return false;
    }
  }
  return true;
}

const SlotState *FindActiveSlotOrPanic(const SlotState &a, const SlotState &b) {
  const SlotState *active = FindActiveSlot(a, b);
  if (active != nullptr) {
    return active;
  }
  // If both slots are blank, callers will treat EEPROM as empty and
  // initialize defaults on first write.
  //
  // If both slots are present but invalid/corrupt, also fall back to the
  // "empty EEPROM" path so configuration can self-heal instead of hard
  // panicking during bring-up.
  return nullptr;
}

}  // namespace

void EE::Init() {
  if (initialized_) {
    Panic(ErrorCode::kEepromReinit);
  }
  initialized_ = true;
}

uint32_t EE::ActiveSectorAddress() const {
  const SlotState slot1 = ReadSlotState(kSlot1);
  const SlotState slot2 = ReadSlotState(kSlot2);
  const SlotState *active = FindActiveSlotOrPanic(slot1, slot2);
  return (active != nullptr) ? active->desc.address : 0u;
}

uint32_t EE::ActiveSectorNumber() const {
  const uint32_t active_address = ActiveSectorAddress();
  if (active_address == kSector1Address) {
    return 1u;
  }
  if (active_address == kSector2Address) {
    return 2u;
  }
  return 0u;
}

void EE::Format() {
  if (!initialized_) {
    Panic(ErrorCode::kEepromNotInitialized);
  }

  if (HAL_FLASH_Unlock() != HAL_OK) {
    Panic(ErrorCode::kEepromFormatFailed);
  }

  const bool ok = EraseSlot(kSlot1) && EraseSlot(kSlot2);
  HAL_FLASH_Lock();
  if (!ok) {
    Panic(ErrorCode::kEepromFormatFailed);
  }
}

bool EE::Read(void *dst, size_t len, size_t offset) const {
  if (dst == nullptr || !CheckRange(len, offset)) {
    return false;
  }

  const SlotState slot1 = ReadSlotState(kSlot1);
  const SlotState slot2 = ReadSlotState(kSlot2);
  const SlotState *active = FindActiveSlotOrPanic(slot1, slot2);
  if (active == nullptr) {
    std::memset(dst, 0xFF, len);
    return true;
  }

  std::memcpy(dst, FlashBytes(PayloadAddress(active->desc) + offset), len);
  return true;
}

bool EE::Write(const void *src, size_t len, size_t offset) {
  if (src == nullptr || !CheckRange(len, offset)) {
    return false;
  }

  const uint8_t *src_bytes = static_cast<const uint8_t *>(src);
  const SlotState slot1 = ReadSlotState(kSlot1);
  const SlotState slot2 = ReadSlotState(kSlot2);
  const SlotState *active = FindActiveSlotOrPanic(slot1, slot2);
  const SlotDesc &inactive = InactiveSlot(active);

  if (active != nullptr) {
    const uint8_t *flash_bytes =
        FlashBytes(PayloadAddress(active->desc) + offset);
    if (std::memcmp(flash_bytes, src_bytes, len) == 0) {
      return true;
    }
  } else if (BufferIsErased(src_bytes, len)) {
    return true;
  }

  if (HAL_FLASH_Unlock() != HAL_OK) {
    return false;
  }

  if (!EraseSlot(inactive)) {
    HAL_FLASH_Lock();
    return false;
  }

  uint32_t crc = 0xFFFFFFFFu;
  const uint8_t *active_payload =
      (active != nullptr) ? FlashBytes(PayloadAddress(active->desc)) : nullptr;
  uint8_t byte = 0xFFu;
  for (uint32_t i = 0; i < kCapacity; ++i) {
    if (i >= offset && i < offset + len) {
      byte = src_bytes[i - offset];
    } else if (active_payload != nullptr) {
      byte = active_payload[i];
    } else {
      byte = 0xFFu;
    }

    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, PayloadAddress(inactive) + i,
                          byte) != HAL_OK) {
      HAL_FLASH_Lock();
      return false;
    }
    crc = UpdateCrc32(crc, byte);
  }
  crc = ~crc;

  const SlotHeader header = {
      .magic = kSlotMagic,
      .sequence = (active != nullptr) ? (active->header.sequence + 1u) : 1u,
      .crc32 = crc,
      .reserved = 0xFFFFFFFFu,
  };

  const bool ok =
      ProgramBytes(inactive.address, reinterpret_cast<const uint8_t *>(&header),
                   sizeof(header)) &&
      VerifySlotPayload(inactive, FlashBytes(PayloadAddress(inactive))) &&
      ValidateSlot(inactive, ReadHeader(inactive));
  HAL_FLASH_Lock();
  return ok;
}

bool EE::CheckRange(size_t len, size_t offset) const {
  if (!initialized_ || offset > kCapacity) {
    return false;
  }
  return len <= (kCapacity - offset);
}
