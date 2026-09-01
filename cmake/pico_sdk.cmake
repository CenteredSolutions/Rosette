# cmake/pico_sdk.cmake
#
# Locates or fetches the Raspberry Pi Pico SDK and makes its
# pico_sdk_init.cmake available to the top-level build.
#
# First configure with no local SDK does a git clone (~40 MB shallow) plus the
# one submodule this build needs (tinyusb, for USB stdio). Point PICO_SDK_PATH
# at an existing checkout to skip it.

include(FetchContent)

# Show clone/download progress — a fresh fetch is otherwise silent for minutes.
set(FETCHCONTENT_QUIET OFF)

if(DEFINED ENV{PICO_SDK_PATH})
    set(PICO_SDK_PATH "$ENV{PICO_SDK_PATH}")
endif()

if(DEFINED PICO_SDK_PATH AND EXISTS "${PICO_SDK_PATH}/pico_sdk_init.cmake")
    message(STATUS "Using local Pico SDK: ${PICO_SDK_PATH}")
else()
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
