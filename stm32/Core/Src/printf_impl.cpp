// printf_impl.cpp — replaces newlib's vsnprintf with nanoprintf.
//
// newlib-nano vsprintf pulls in _malloc_r / _free_r / _sbrk to support
// unbounded %s and float formats; the linker can't prove our format
// strings never allocate, so the heap stays reachable.
//
// nanoprintf is a single-header, allocation-free printf (0BSD). Compiled
// once here, then routed in via `-Wl,--wrap=vsnprintf`. The newlib
// vsprintf object goes unreferenced → _malloc_r / _free_r / _sbrk drop
// out of the binary.

// Feature flags. All must be #defined per nanoprintf.h's error gates.
// Enable only what the firmware uses today.
#define NANOPRINTF_USE_FIELD_WIDTH_FORMAT_SPECIFIERS 1
#define NANOPRINTF_USE_PRECISION_FORMAT_SPECIFIERS 1
#define NANOPRINTF_USE_FLOAT_FORMAT_SPECIFIERS \
  1                                               // %f / %e / %g (nanoprintf's
                                                  // own conversion: no heap,
                                                  // no newlib float printf)
#define NANOPRINTF_USE_LARGE_FORMAT_SPECIFIERS 1  // %lld, %lu, %zu, …
#define NANOPRINTF_USE_SMALL_FORMAT_SPECIFIERS 1  // %hd, %hhd
#define NANOPRINTF_USE_WRITEBACK_FORMAT_SPECIFIERS 0  // no %n
#define NANOPRINTF_USE_BINARY_FORMAT_SPECIFIERS 0     // no %b
#define NANOPRINTF_USE_ALT_FORM_FLAG 1

#define NANOPRINTF_IMPLEMENTATION
#include <cstdarg>
#include <cstddef>

#include "nanoprintf.h"

// Linker wrap stubs. `-Wl,--wrap=vsnprintf` redirects calls to vsnprintf
// into __wrap_vsnprintf; the original is reachable as __real_vsnprintf
// (unused here).
// NOLINTBEGIN(readability-identifier-naming)
// The `__wrap_<sym>` prefix is fixed by GNU ld's --wrap mechanism;
// renaming breaks the wrap.
extern "C" int __wrap_vsnprintf(char *buffer, size_t bufsz, const char *format,
                                va_list args) {
  return npf_vsnprintf(buffer, bufsz, format, args);
}

extern "C" int __wrap_snprintf(char *buffer, size_t bufsz, const char *format,
                               ...) {
  va_list args;
  va_start(args, format);
  const int rv = npf_vsnprintf(buffer, bufsz, format, args);
  va_end(args);
  return rv;
}
// NOLINTEND(readability-identifier-naming)
