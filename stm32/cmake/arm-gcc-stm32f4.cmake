set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(TOOLCHAIN_PREFIX arm-none-eabi)

set(CMAKE_C_COMPILER   ${TOOLCHAIN_PREFIX}-gcc)
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PREFIX}-g++)
set(CMAKE_ASM_COMPILER ${TOOLCHAIN_PREFIX}-gcc)

# Cortex-M4F (STM32F407)
set(MCPU   "-mcpu=cortex-m4")
set(MTHUMB "-mthumb")
set(MFPU   "-mfpu=fpv4-sp-d16")
set(MFLOAT "-mfloat-abi=hard")

add_compile_options(
  ${MCPU} ${MTHUMB} ${MFPU} ${MFLOAT}
  -ffunction-sections -fdata-sections
)

add_link_options(
  ${MCPU} ${MTHUMB} ${MFPU} ${MFLOAT}
  -Wl,--gc-sections
)
