# ports/chip/imxrt/arm_gcc_cortex_m7.cmake — the cross toolchain for i.MX RT10xx.
#
# Installed as CMAKE_TOOLCHAIN_FILE by prelude.cmake before project(), unless
# the caller named one. The GNU Arm Embedded toolchain (arm-none-eabi-gcc)
# must be on PATH, or ARM_GCC_ROOT (CMake or environment variable) must name
# its install prefix.
#
# Code generation: Cortex-M7 with its double-precision FPU (FPv5-D16), hard
# float ABI, Thumb-2. Every function and object goes in its own section so
# --gc-sections drops what an image does not reach. C++ is built without
# exceptions or RTTI, as on the rp2 port. The full newlib, not newlib-nano,
# backs the C library, so printf() handles every format Rosette uses
# (64-bit integers among them) at a cost only in flash, of which an i.MX RT
# board has megabytes; the nosys stubs fill in the system calls the chip port
# does not supply (imxrt_newlib_syscalls.c).
#
# References: Arm GNU Toolchain documentation, "Arm options" (-mcpu, -mfpu,
# -mfloat-abi); i.MX RT1060 Reference Manual §1.3 (Arm Cortex-M7 core, FPU).

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR cortex-m7)

if(NOT DEFINED ARM_GCC_ROOT AND DEFINED ENV{ARM_GCC_ROOT})
  set(ARM_GCC_ROOT "$ENV{ARM_GCC_ROOT}")
endif()

if(ARM_GCC_ROOT)
  set(_arm_gcc_prefix "${ARM_GCC_ROOT}/bin/arm-none-eabi-")
else()
  set(_arm_gcc_prefix "arm-none-eabi-")
endif()

set(CMAKE_C_COMPILER   "${_arm_gcc_prefix}gcc")
set(CMAKE_CXX_COMPILER "${_arm_gcc_prefix}g++")
set(CMAKE_ASM_COMPILER "${_arm_gcc_prefix}gcc")
set(CMAKE_OBJCOPY      "${_arm_gcc_prefix}objcopy" CACHE FILEPATH "objcopy for image outputs")
set(CMAKE_SIZE         "${_arm_gcc_prefix}size" CACHE FILEPATH "size for image reports")

# Compiler checks link a static library: a test executable has no startup or
# linker script yet.
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(_imxrt_cpu_flags "-mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard")
set(CMAKE_C_FLAGS_INIT   "${_imxrt_cpu_flags} -ffunction-sections -fdata-sections")
set(CMAKE_CXX_FLAGS_INIT
    "${_imxrt_cpu_flags} -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti")
set(CMAKE_ASM_FLAGS_INIT "${_imxrt_cpu_flags} -x assembler-with-cpp")
set(CMAKE_EXE_LINKER_FLAGS_INIT
    "${_imxrt_cpu_flags} --specs=nosys.specs -Wl,--gc-sections")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
