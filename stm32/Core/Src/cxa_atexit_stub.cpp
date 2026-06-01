// cxa_atexit_stub.cpp — no-op replacements for the C++ runtime's
// destructor-registration machinery.
//
// The C++ ABI specifies that destructors for function local `static` objects
// (Meyers singletons) and global non-trivial objects are registered via
// `__cxa_atexit(dtor, object, dso_handle)` at construction time, and run via
// `__cxa_finalize()` when the program exits. Newlib's `__cxa_atexit` maintains
// a linked list of pending destructors, and the first registration mallocs a
// node — which is what was pulling `_malloc_r` / `_free_r` / `_sbrk` into the
// binary even after we replaced vsnprintf.
//
// In firmware, `main()` never returns. The destructor chain never runs.
// The registration itself is wasted work and a heap dependency we don't
// need. These stubs make registration a no-op:
//   - `__cxa_atexit` returns 0 (success) without recording anything.
//   - `__cxa_finalize` does nothing.
// `__dso_handle` is already provided by the toolchain's crtbegin.o for
// static linkage so we don't define it here.
//
// Trade-off: function-local statics' destructors never run. For all
// the singletons in System / drivers / services, that's correct —
// the firmware doesn't "shut down," and destruction at exit on a
// device that never exits is meaningless. If you ever want a piece
// of code to run on shutdown, you'll do it explicitly via a sentinel
// function called from a panic / reboot path, not via global dtors.

// NOLINTBEGIN(readability-identifier-naming)
// `__cxa_atexit` and `__cxa_finalize` are ABI-fixed names from the
// Itanium C++ ABI; the compiler emits calls to exactly these symbols.
extern "C" {

int __cxa_atexit(void (*)(void *), void *, void *) { return 0; }

void __cxa_finalize(void *) {}

}  // extern "C"
// NOLINTEND(readability-identifier-naming)

// ── C++ allocator stubs ──────────────────────────────────────────────
// Even with -fno-exceptions / -fno-rtti and zero `new`/`delete` expressions
// in the firmware, classes with `virtual` destructors (IState, IFastTick-
// State, ...) emit vtables whose "deleting destructor" slot calls
// `operator delete`. The default `operator delete` lives in libstdc++'s
// `del_op.o`, which calls `free`, which pulls newlib's `_malloc_r` and
// `_sbrk` into the link.
//
// Providing our own `operator delete` satisfies the vtable reference
// without dragging the heap into the binary. The bodies use
// `__builtin_trap()` because they MUST never execute on this firmware
// (every dynamic allocation site is forbidden by `check_forbidden.sh`
// and there is no heap to free into). If a trap ever fires, it's a
// hard fault that's trivial to spot in a debugger.

#include <cstddef>

void operator delete(void *) noexcept { __builtin_trap(); }
void operator delete(void *, std::size_t) noexcept { __builtin_trap(); }
void operator delete[](void *) noexcept { __builtin_trap(); }
void operator delete[](void *, std::size_t) noexcept { __builtin_trap(); }

// ── newlib reentrant allocator overrides ─────────────────────────────
// Even with no `new` / `delete` in firmware code, newlib's stdio
// internals (fvwrite, vfprintf, etc.) reference `_malloc_r` / `_free_r`
// in their call graphs — pulled in transitively despite never being
// called on our paths. Providing strong stubs here satisfies the
// references with `__builtin_trap()` bodies, so the multi-hundred-byte
// newlib implementations don't get linked. Hard runtime fault if any
// allocator path ever fires, which it shouldn't (forbidden by
// check_forbidden.sh, EIGEN_NO_MALLOC, and the operator delete stubs
// above).
//
// `_sbrk` itself is defined in stm32/Core/Src/sysmem.c — that file is
// edited separately to trap instead of allocate; we don't touch it here
// to avoid a multiple-definition.

// NOLINTBEGIN(readability-identifier-naming)
// `_malloc_r` / `_free_r` / `_sbrk_r` etc. are newlib's reentrant
// allocator names; they must match exactly for the linker to pick
// these stubs over the library implementations.
extern "C" {

void *_malloc_r(void *, std::size_t) { __builtin_trap(); }
void _free_r(void *, void *) { __builtin_trap(); }
void *_calloc_r(void *, std::size_t, std::size_t) { __builtin_trap(); }
void *_realloc_r(void *, void *, std::size_t) { __builtin_trap(); }
void *_sbrk_r(void *, int) { __builtin_trap(); }

}  // extern "C"
// NOLINTEND(readability-identifier-naming)
