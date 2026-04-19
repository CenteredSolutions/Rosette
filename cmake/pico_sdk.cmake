# cmake/pico_sdk.cmake
#
# Locates or fetches the Raspberry Pi Pico SDK and makes its
# pico_sdk_init.cmake available to the top-level build.

include(FetchContent)

if(DEFINED ENV{PICO_SDK_PATH})
    set(PICO_SDK_PATH "$ENV{PICO_SDK_PATH}")
endif()

if(DEFINED PICO_SDK_PATH AND EXISTS "${PICO_SDK_PATH}/pico_sdk_init.cmake")
    message(STATUS "Using local Pico SDK: ${PICO_SDK_PATH}")
else()
    FetchContent_Declare(pico_sdk
        GIT_REPOSITORY https://github.com/raspberrypi/pico-sdk.git
        GIT_TAG        2.2.0
        GIT_SHALLOW    TRUE
    )
    FetchContent_GetProperties(pico_sdk)
    if(NOT pico_sdk_POPULATED)
        FetchContent_Populate(pico_sdk)
    endif()
    set(PICO_SDK_PATH "${pico_sdk_SOURCE_DIR}" CACHE STRING "Path to the Pico SDK" FORCE)
    message(STATUS "Using fetched Pico SDK: ${PICO_SDK_PATH}")
endif()

include("${PICO_SDK_PATH}/pico_sdk_init.cmake")
