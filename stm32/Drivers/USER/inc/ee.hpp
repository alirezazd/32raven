#pragma once

#include <cstddef>
#include <cstdint>
#include <type_traits>

class EE {
public:
  static constexpr uint32_t kSectorSize = 16u * 1024u;
  static constexpr uint32_t kHeaderSize = 16u;
  static constexpr uint32_t kCapacity = kSectorSize - kHeaderSize;
  static constexpr uint32_t kSector1Address = 0x08004000u;
  static constexpr uint32_t kSector2Address = 0x08008000u;

  static EE &GetInstance() {
    static EE instance;
    return instance;
  }

  void Init();

  uint32_t Capacity() const { return kCapacity; }
  bool IsInitialized() const { return initialized_; }
  uint32_t ActiveSectorAddress() const;
  uint32_t ActiveSectorNumber() const;

  void Format();
  bool Read(void *dst, size_t len, size_t offset = 0) const;
  bool Write(const void *src, size_t len, size_t offset = 0);

  template <typename T> bool ReadObject(T &dst, size_t offset = 0) const {
    static_assert(std::is_trivially_copyable_v<T>);
    return Read(&dst, sizeof(T), offset);
  }

  template <typename T> bool WriteObject(const T &src, size_t offset = 0) {
    static_assert(std::is_trivially_copyable_v<T>);
    return Write(&src, sizeof(T), offset);
  }

private:
  friend class System;

  EE() = default;
  ~EE() = default;

  EE(const EE &) = delete;
  EE &operator=(const EE &) = delete;

  bool CheckRange(size_t len, size_t offset) const;

  bool initialized_ = false;
};
