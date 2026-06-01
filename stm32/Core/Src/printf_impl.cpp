// printf_impl.cpp — replaces newlib's vsnprintf with nanoprintf.
//
// Why: newlib-nano's vsprintf implementation pulls in `_malloc_r`,
// `_free_r`, and `_sbrk` to support unbounded %s and float formats.
// Even though our format strings never trigger an allocation in
// practice, the linker can't prove that — the symbols stay live and
// the call graph keeps a heap reachable.
//
// nanoprintf is a single-header, allocation-free printf implementation
// (Charles Nicholson / 0BSD). We compile it once here with the feature
// set the firmware actually uses, then wrap `vsnprintf` at the linker
// (`-Wl,--wrap=vsnprintf`) so existing callers route through nanoprintf
// without source-level changes. The newlib vsprintf object is no longer
// referenced → _malloc_r / _free_r / _sbrk leave the binary.

// Feature flags. All must be #defined per nanoprintf.h's error gates.
// Enable only what the firmware uses today; adjust if SendLog format
// strings grow new conversions later.
#define NANOPRINTF_USE_FIELD_WIDTH_FORMAT_SPECIFIERS 1
#define NANOPRINTF_USE_PRECISION_FORMAT_SPECIFIERS 1
#define NANOPRINTF_USE_FLOAT_FORMAT_SPECIFIERS 0      // no %f / %e / %g
#define NANOPRINTF_USE_LARGE_FORMAT_SPECIFIERS 1      // %lld, %lu, %zu, …
#define NANOPRINTF_USE_SMALL_FORMAT_SPECIFIERS 1      // %hd, %hhd
#define NANOPRINTF_USE_WRITEBACK_FORMAT_SPECIFIERS 0  // no %n
#define NANOPRINTF_USE_BINARY_FORMAT_SPECIFIERS 0     // no %b
#define NANOPRINTF_USE_ALT_FORM_FLAG 1

#define NANOPRINTF_IMPLEMENTATION
#include <cstdarg>
#include <cstddef>

#include "nanoprintf.h"

// Linker wrap stubs. `-Wl,--wrap=vsnprintf` redirects every call to
// vsnprintf into __wrap_vsnprintf; the original libc vsnprintf is
// available as __real_vsnprintf (we don't call it). Same wrap pattern
// is what newlib's nano variant uses internally.
// NOLINTBEGIN(readability-identifier-naming)
// The `__wrap_<sym>` naming is fixed by GNU ld's --wrap mechanism.
// The linker pattern-matches on exactly this prefix; renaming breaks
// the wrap.
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
