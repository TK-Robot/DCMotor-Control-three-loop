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
set(TARGET_FLAGS "-mcpu=cortex-m0plus ")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${TARGET_FLAGS}")
set(CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp -MMD -MP")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -fdata-sections -ffunction-sections")

# Code size switch for CLion / CMake.
# STM32G030F6P6 has 32 KiB Flash, and this project reserves the last 2 KiB
# for NVM parameters, so the application linker region is 30 KiB.
#
# Change only TK_SIZE_PROFILE when you need a different size/debug tradeoff:
#   "debug_full"  : -O0 -g3, easiest source-level debug, usually too large.
#   "debug"       : -Og -g3, usable source debugging with bounded code size.
#   "debug_small" : -Os -g3, symbols retained but values may be optimized out.
#   "min_size"    : -Oz -g0, smallest firmware.
set(TK_SIZE_PROFILE "debug")

if(TK_SIZE_PROFILE STREQUAL "debug_full")
    set(TK_DEBUG_FLAGS "-O0 -g3")
elseif(TK_SIZE_PROFILE STREQUAL "min_size")
    set(TK_DEBUG_FLAGS "-Oz -g0")
elseif(TK_SIZE_PROFILE STREQUAL "debug_small")
    set(TK_DEBUG_FLAGS "-Os -g3")
else()
    set(TK_DEBUG_FLAGS "-Og -g3 -fno-omit-frame-pointer")
endif()

set(CMAKE_C_FLAGS_DEBUG "${TK_DEBUG_FLAGS}")
set(CMAKE_C_FLAGS_RELEASE "-Oz -g0 -flto -fno-fat-lto-objects")
set(CMAKE_CXX_FLAGS_DEBUG "${TK_DEBUG_FLAGS}")
set(CMAKE_CXX_FLAGS_RELEASE "-Oz -g0 -flto -fno-fat-lto-objects")

set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -fno-rtti -fno-exceptions -fno-threadsafe-statics")

set(CMAKE_EXE_LINKER_FLAGS "${TARGET_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -nostartfiles")
set(CMAKE_EXE_LINKER_FLAGS_RELEASE "-flto")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T \"${CMAKE_SOURCE_DIR}/STM32G030XX_FLASH.ld\"")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --specs=nano.specs")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${CMAKE_PROJECT_NAME}.map -Wl,--gc-sections")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--print-memory-usage")
set(TOOLCHAIN_LINK_LIBRARIES "m")
