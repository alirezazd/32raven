#pragma once

#include "error_code.hpp"

[[noreturn]] void Panic(ErrorCode code);
