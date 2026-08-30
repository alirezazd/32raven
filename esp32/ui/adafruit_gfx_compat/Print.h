#pragma once

// NOLINTBEGIN(readability-identifier-naming)
// A compatibility shim: the names are Adafruit_GFX's own, and the library
// compiles against them by spelling. Renaming any of them is renaming their
// API, which is the one thing this file exists not to do.

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

// NOLINTEND(readability-identifier-naming)
