#pragma once

// NOLINTBEGIN(readability-identifier-naming)
// A compatibility shim: the names are Adafruit_GFX's own, and the library
// compiles against them by spelling. Renaming any of them is renaming their
// API, which is the one thing this file exists not to do.

#include <cstddef>
#include <cstdint>
#include <string>

using String = std::string;
using boolean = bool;
using byte = uint8_t;

class __FlashStringHelper {};

#ifndef PROGMEM
#define PROGMEM
#endif

// NOLINTEND(readability-identifier-naming)
