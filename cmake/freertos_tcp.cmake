# cmake/freertos_tcp.cmake
#
# Locates or fetches FreeRTOS-Kernel and FreeRTOS-Plus-TCP and exposes
# them as CMake targets: freertos_kernel, freertos_tcp.
#
# Paths
# -----
# Set via environment or -D flag:
#   FREERTOS_KERNEL_PATH   — path to FreeRTOS-Kernel repo root
#   FREERTOS_TCP_PATH      — path to FreeRTOS-Plus-TCP repo root
#
# If not set, CMake FetchContent pulls them from GitHub.

include(FetchContent)

# ── FreeRTOS Kernel ───────────────────────────────────────────────────────────

if(DEFINED ENV{FREERTOS_KERNEL_PATH})
    set(FREERTOS_KERNEL_PATH "$ENV{FREERTOS_KERNEL_PATH}")
endif()

if(DEFINED FREERTOS_KERNEL_PATH AND EXISTS "${FREERTOS_KERNEL_PATH}")
    message(STATUS "Using local FreeRTOS-Kernel: ${FREERTOS_KERNEL_PATH}")
    add_subdirectory(${FREERTOS_KERNEL_PATH} freertos_kernel EXCLUDE_FROM_ALL)
else()
    FetchContent_Declare(freertos_kernel
        GIT_REPOSITORY https://github.com/FreeRTOS/FreeRTOS-Kernel.git
        GIT_TAG        V11.1.0
        GIT_SHALLOW    TRUE
    )
    FetchContent_MakeAvailable(freertos_kernel)
endif()

# Port selection: RP2040 and RP2350 both use the ARM_CM33_NTZ/non_secure
# port on RP2350 and the ARM_CM0 port on RP2040.
if(RTPS_PROCESSOR STREQUAL "RP2350")
    set(FREERTOS_PORT GCC_ARM_CM33_NTZ_NONSECURE CACHE STRING "" FORCE)
else()
    set(FREERTOS_PORT GCC_ARM_CM0 CACHE STRING "" FORCE)
endif()

# FreeRTOSConfig.h is provided by the project config/ directory.
target_include_directories(freertos_kernel PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/config
)

# ── FreeRTOS+TCP ──────────────────────────────────────────────────────────────

if(DEFINED ENV{FREERTOS_TCP_PATH})
    set(FREERTOS_TCP_PATH "$ENV{FREERTOS_TCP_PATH}")
endif()

if(DEFINED FREERTOS_TCP_PATH AND EXISTS "${FREERTOS_TCP_PATH}")
    message(STATUS "Using local FreeRTOS+TCP: ${FREERTOS_TCP_PATH}")
    add_subdirectory(${FREERTOS_TCP_PATH} freertos_tcp EXCLUDE_FROM_ALL)
else()
    FetchContent_Declare(freertos_tcp
        GIT_REPOSITORY https://github.com/FreeRTOS/FreeRTOS-Plus-TCP.git
        GIT_TAG        V4.2.2
        GIT_SHALLOW    TRUE
    )
    FetchContent_MakeAvailable(freertos_tcp)
endif()

# FreeRTOS+TCP needs to find FreeRTOS_IP_ConfigDefaults.h which lives in
# the library itself, plus our FreeRTOSIPConfig.h in config/.
target_include_directories(freertos_tcp PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/config
)

# Network interface: our MACRAW driver is compiled as part of rtps_bsp.
# Tell FreeRTOS+TCP not to compile its own interface.
set(FREERTOS_TCP_NETWORK_IF "CUSTOM" CACHE STRING "" FORCE)
