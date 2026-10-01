# ports/chip/rp2/prelude.cmake — the rp2 port's setup before project().
#
# Included by cmake/rosette_prelude.cmake, before the first project() call, so
# that the Pico SDK can install its cross toolchain. Locates or fetches the
# SDK and includes its pico_sdk_init.cmake; port.cmake beside this file calls
# pico_sdk_init() after project().
#
# First configure with no local SDK does a git clone (~40 MB shallow) plus the
# one submodule this build needs (tinyusb, for USB stdio). Point PICO_SDK_PATH
# at an existing checkout to skip it.

include_guard(GLOBAL)
include(FetchContent)

# Show clone/download progress — a fresh fetch is otherwise silent for minutes.
set(FETCHCONTENT_QUIET OFF)

if(DEFINED ENV{PICO_SDK_PATH})
    set(PICO_SDK_PATH "$ENV{PICO_SDK_PATH}")
endif()

if(DEFINED PICO_SDK_PATH AND EXISTS "${PICO_SDK_PATH}/pico_sdk_init.cmake")
    message(STATUS "Using local Pico SDK: ${PICO_SDK_PATH}")
else()
    message(STATUS "")
    message(STATUS "Rosette: first configure for '${ROSETTE_BOARD}' fetches the Pico SDK")
    message(STATUS "  (a few minutes; progress shown below). Set PICO_SDK_PATH to use an")
    message(STATUS "  existing checkout. A micro-ROS library is cross-built later, during")
    message(STATUS "  'cmake --build' (~20-30 min, one time).")
    message(STATUS "")
    message(STATUS "Fetching the Pico SDK (raspberrypi/pico-sdk 2.2.0) — first run only, a few minutes")
    FetchContent_Declare(pico_sdk
        GIT_REPOSITORY https://github.com/raspberrypi/pico-sdk.git
        GIT_TAG        2.2.0
        GIT_SHALLOW    TRUE
        GIT_PROGRESS   TRUE
        # Only tinyusb is needed here; skip mbedtls / cyw43-driver / btstack / etc.
        GIT_SUBMODULES "lib/tinyusb"
        GIT_SUBMODULES_RECURSE OFF
    )
    FetchContent_MakeAvailable(pico_sdk)
    set(PICO_SDK_PATH "${pico_sdk_SOURCE_DIR}" CACHE STRING "Path to the Pico SDK" FORCE)
    message(STATUS "Fetched Pico SDK: ${PICO_SDK_PATH}")
endif()

include("${PICO_SDK_PATH}/pico_sdk_init.cmake")

# Examples that call the Pico SDK themselves build only on this chip.
set(ROSETTE_BOARD_ENABLE_PICO_SDK ON)
