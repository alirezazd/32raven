#pragma once

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
