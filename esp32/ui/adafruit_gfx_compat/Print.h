#pragma once

#include <cstring>

#include "Arduino.h"

class Print {
 public:
  virtual ~Print() = default;
  virtual size_t write(uint8_t value) = 0;

  size_t write(const uint8_t *buffer, size_t size) {
    if (buffer == nullptr) {
      return 0;
    }

    size_t written = 0;
    for (; written < size; ++written) {
      write(buffer[written]);
    }
    return written;
  }

  size_t write(const char *str) {
    if (str == nullptr) {
      return 0;
    }

    return write(reinterpret_cast<const uint8_t *>(str), std::strlen(str));
  }

  size_t print(const char *str) { return write(str); }

  size_t print(const String &str) {
    return write(reinterpret_cast<const uint8_t *>(str.c_str()), str.size());
  }
};
