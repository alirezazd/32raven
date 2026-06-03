set(CMAKE_SYSTEM_NAME               Generic)
set(CMAKE_SYSTEM_PROCESSOR          arm)

set(CMAKE_C_COMPILER_ID GNU)
set(CMAKE_CXX_COMPILER_ID GNU)

# Some default GCC settings
# arm-none-eabi- must be part of path environment
set(TOOLCHAIN_PREFIX                arm-none-eabi-)

set(CMAKE_C_COMPILER                ${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_ASM_COMPILER              ${CMAKE_C_COMPILER})
set(CMAKE_CXX_COMPILER              ${TOOLCHAIN_PREFIX}g++)
set(CMAKE_LINKER                    ${TOOLCHAIN_PREFIX}g++)
set(CMAKE_OBJCOPY                   ${TOOLCHAIN_PREFIX}objcopy)
set(CMAKE_SIZE                      ${TOOLCHAIN_PREFIX}size)

set(CMAKE_EXECUTABLE_SUFFIX_ASM     ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_C       ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX     ".elf")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# MCU specific flags
set(TARGET_FLAGS "-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard ")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${TARGET_FLAGS}")
set(CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp -MMD -MP")
# Float codegen knobs applied to ALL configs so Debug and Release stay
# numerically identical:
#   -ffp-contract=fast  -> let the M4 FPU fuse a*b+c into a single VFMA (one
#                          rounding; faster and more accurate in the control math)
#   -fno-math-errno     -> math funcs don't set errno, so sqrtf/fabsf/... inline
#                          to single hardware instructions (VSQRT.F32, VABS)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -fdata-sections -ffunction-sections -ffp-contract=fast -fno-math-errno")

# Debug: no optimization, full symbols. Release: -O2 (the Cortex-M4 speed sweet
# spot) + LTO so the many small cross-TU methods (singleton accessors, inline
# driver ops) collapse at link. -O3 is a drop-in swap to A/B if you want to
# chase the last few %.
set(CMAKE_C_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_C_FLAGS_RELEASE "-O2 -g0 -flto=auto")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -g0 -flto=auto")

set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -fno-rtti -fno-exceptions -fno-threadsafe-statics")

set(CMAKE_EXE_LINKER_FLAGS "${TARGET_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T \"${CMAKE_SOURCE_DIR}/STM32F407XX_FLASH.ld\"")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --specs=nano.specs")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${CMAKE_PROJECT_NAME}.map -Wl,--gc-sections")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--print-memory-usage")

# Release links with LTO too: the final optimization (LTRANS) pass runs at link,
# so it needs the same -O2 -flto=auto as the compile step (Debug links normally).
#   -flto=auto -> run LTRANS in parallel across cores (silences GCC's "serial
#                 compilation of N LTRANS jobs" note and speeds up the link)
#   -Wno-psabi -> codegen happens here under LTO, so the Eigen "parameter passing
#                 for struct Matrix ... changed in GCC 10.1" psabi notes are
#                 emitted at link; suppress them here as well as at compile.
set(CMAKE_EXE_LINKER_FLAGS_RELEASE "-O2 -flto=auto -Wno-psabi")
set(TOOLCHAIN_LINK_LIBRARIES "m")
