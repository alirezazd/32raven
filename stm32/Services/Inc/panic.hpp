#pragma once

#include <cstdint>

#include "error_code.hpp"

// Panic handler with zero dependencies on System class
// Disables interrupts, flashes LED, and sends error code via UART
void Panic(ErrorCode code);
