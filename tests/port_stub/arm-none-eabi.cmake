# Cross toolchain for tests/port_stub: bare Arm Cortex-M with newlib, no SDK.
#
# newlib supplies the C and C++ libraries and nothing else: no sockets, no
# threads, no Pico SDK. nosys.specs stubs its system calls, so an image links
# as long as Rosette itself defines everything it calls.
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)

# Cortex-M0+ is the smallest core Rosette targets (RP2040). No exceptions or
# RTTI, as in the Pico SDK's defaults.
set(CMAKE_C_FLAGS_INIT "-mcpu=cortex-m0plus -mthumb")
set(CMAKE_CXX_FLAGS_INIT "-mcpu=cortex-m0plus -mthumb -fno-exceptions -fno-rtti")
set(CMAKE_EXE_LINKER_FLAGS_INIT "--specs=nosys.specs")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
