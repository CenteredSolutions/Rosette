# cmake/freertos_tcp.cmake
#
# Locates or fetches FreeRTOS-Kernel and FreeRTOS-Plus-TCP and exposes them as
# the CMake targets `freertos_kernel` and `freertos_tcp`.
#
# Set FREERTOS_KERNEL_PATH / FREERTOS_TCP_PATH (env or -D) to use local
# checkouts; otherwise FetchContent pulls them from GitHub.
#
# FreeRTOS-Kernel V11 requires FREERTOS_PORT and a `freertos_config` INTERFACE
# target to exist *before* the kernel is added, so those come first here.

include(FetchContent)
set(FETCHCONTENT_QUIET OFF)

# ── Port + heap (must precede the kernel) ─────────────────────────────────────
if(ROSETTE_PROCESSOR STREQUAL "RP2350")
    set(FREERTOS_PORT GCC_ARM_CM33_NTZ_NONSECURE CACHE STRING "" FORCE)
else()
    set(FREERTOS_PORT GCC_ARM_CM0 CACHE STRING "" FORCE)
endif()
set(FREERTOS_HEAP 4 CACHE STRING "" FORCE)

# ── freertos_config: where FreeRTOSConfig.h / FreeRTOSIPConfig.h live ─────────
if(NOT TARGET freertos_config)
    add_library(freertos_config INTERFACE)
    target_include_directories(freertos_config SYSTEM INTERFACE
        "${ROSETTE_SOURCE_DIR}/config"
        "${ROSETTE_BOARD_DIR}")            # board_config.h -> BOARD_IS_RP2350
    if(TARGET pico_base_headers)
        # FreeRTOSConfig.h pulls in SystemCoreClock / pico headers.
        target_link_libraries(freertos_config INTERFACE pico_base_headers)
    endif()
endif()

# ── FreeRTOS Kernel ─────────────────────────────────────────────────────────
if(DEFINED ENV{FREERTOS_KERNEL_PATH})
    set(FREERTOS_KERNEL_PATH "$ENV{FREERTOS_KERNEL_PATH}")
endif()

if(DEFINED FREERTOS_KERNEL_PATH AND EXISTS "${FREERTOS_KERNEL_PATH}")
    message(STATUS "Using local FreeRTOS-Kernel: ${FREERTOS_KERNEL_PATH}")
    add_subdirectory(${FREERTOS_KERNEL_PATH} freertos_kernel EXCLUDE_FROM_ALL)
else()
    message(STATUS "Fetching FreeRTOS-Kernel V11.1.0 — first run only")
    FetchContent_Declare(freertos_kernel
        GIT_REPOSITORY https://github.com/FreeRTOS/FreeRTOS-Kernel.git
        GIT_TAG        V11.1.0
        GIT_SHALLOW    TRUE
        GIT_PROGRESS   TRUE
        GIT_SUBMODULES ""
    )
    FetchContent_MakeAvailable(freertos_kernel)
endif()

# ── FreeRTOS+TCP ────────────────────────────────────────────────────────────
if(DEFINED ENV{FREERTOS_TCP_PATH})
    set(FREERTOS_TCP_PATH "$ENV{FREERTOS_TCP_PATH}")
endif()

if(DEFINED FREERTOS_TCP_PATH AND EXISTS "${FREERTOS_TCP_PATH}")
    message(STATUS "Using local FreeRTOS+TCP: ${FREERTOS_TCP_PATH}")
    add_subdirectory(${FREERTOS_TCP_PATH} freertos_tcp EXCLUDE_FROM_ALL)
else()
    message(STATUS "Fetching FreeRTOS-Plus-TCP V4.2.2 — first run only")
    FetchContent_Declare(freertos_tcp
        GIT_REPOSITORY https://github.com/FreeRTOS/FreeRTOS-Plus-TCP.git
        GIT_TAG        V4.2.2
        GIT_SHALLOW    TRUE
        GIT_PROGRESS   TRUE
        GIT_SUBMODULES ""
    )
    FetchContent_MakeAvailable(freertos_tcp)
endif()

target_include_directories(freertos_tcp PUBLIC "${ROSETTE_SOURCE_DIR}/config")
