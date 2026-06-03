// cxa_atexit_stub.cpp — no-op replacements for the C++ runtime's
// destructor-registration machinery.
//
// Per the Itanium ABI, static/global dtors register via `__cxa_atexit` at
// construction and run via `__cxa_finalize` at exit. Newlib's `__cxa_atexit`
// mallocs a list node on first registration, pulling `_malloc_r`/`_free_r`/
// `_sbrk` into the binary. Firmware never exits, so the chain never runs;
// these no-op stubs drop the registration and its heap dependency.
//
// `__dso_handle` is provided by the toolchain's crtbegin.o for static
// linkage, so it's not defined here.
//
// Trade-off: function-local static dtors never run — correct for a device
// that never shuts down. Shutdown code must be called explicitly from a
// panic/reboot path, not via global dtors.

// NOLINTBEGIN(readability-identifier-naming)
// ABI-fixed Itanium C++ ABI names; the compiler emits calls to these exact
// symbols.
extern "C" {

int __cxa_atexit(void (*)(void *), void *, void *) { return 0; }

void __cxa_finalize(void *) {}

}  // extern "C"
// NOLINTEND(readability-identifier-naming)

// ── C++ allocator stubs ──────────────────────────────────────────────
// Classes with virtual dtors emit vtables whose deleting-destructor slot
// references `operator delete`, even with no `new`/`delete` in firmware.
// libstdc++'s default `operator delete` calls `free`, pulling in `_malloc_r`/
// `_sbrk`. These overrides satisfy the vtable reference without the heap.
// Bodies trap because they must never run (no heap; allocation forbidden by
// check_forbidden.sh) — a trap surfaces as an obvious hard fault.

#include <cstddef>

void operator delete(void *) noexcept { __builtin_trap(); }
void operator delete(void *, std::size_t) noexcept { __builtin_trap(); }
void operator delete[](void *) noexcept { __builtin_trap(); }
void operator delete[](void *, std::size_t) noexcept { __builtin_trap(); }

// ── newlib reentrant allocator overrides ─────────────────────────────
// Newlib's stdio internals (vfprintf, fvwrite, ...) reference `_malloc_r`/
// `_free_r` in their call graphs even on paths never taken here. Strong
// trapping stubs satisfy those references so the real newlib allocators
// don't link. Traps must never fire (forbidden by check_forbidden.sh,
// EIGEN_NO_MALLOC, and the operator delete stubs above).
//
// `_sbrk` is defined separately in sysmem.c (also patched to trap); not
// redefined here to avoid a multiple-definition error.

// NOLINTBEGIN(readability-identifier-naming)
// Newlib reentrant allocator names; must match exactly for the linker to
// pick these stubs over the library implementations.
extern "C" {

void *_malloc_r(void *, std::size_t) { __builtin_trap(); }
void _free_r(void *, void *) { __builtin_trap(); }
void *_calloc_r(void *, std::size_t, std::size_t) { __builtin_trap(); }
void *_realloc_r(void *, void *, std::size_t) { __builtin_trap(); }
void *_sbrk_r(void *, int) { __builtin_trap(); }

}  // extern "C"
// NOLINTEND(readability-identifier-naming)
