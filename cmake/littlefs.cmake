# cmake/littlefs.cmake
#
# Locates or fetches LittleFS and exports:
#   RTPS_LITTLEFS_DIR   - path containing lfs.h, lfs.c, lfs_util.c
#   RTPS_HAVE_LITTLEFS  - ON when the required files are available

include(FetchContent)

set(RTPS_HAVE_LITTLEFS OFF)

if(DEFINED ENV{LITTLEFS_PATH})
    set(LITTLEFS_PATH "$ENV{LITTLEFS_PATH}")
endif()

# Backward-compatible local checkout fallback.
set(_rtps_default_littlefs_dir "${CMAKE_CURRENT_SOURCE_DIR}/../littlefs")

if(DEFINED LITTLEFS_PATH AND EXISTS "${LITTLEFS_PATH}/lfs.c")
    set(RTPS_LITTLEFS_DIR "${LITTLEFS_PATH}")
    message(STATUS "Using local LittleFS: ${RTPS_LITTLEFS_DIR}")
elseif(EXISTS "${_rtps_default_littlefs_dir}/lfs.c")
    set(RTPS_LITTLEFS_DIR "${_rtps_default_littlefs_dir}")
    message(STATUS "Using local LittleFS: ${RTPS_LITTLEFS_DIR}")
else()
    FetchContent_Declare(littlefs
        GIT_REPOSITORY https://github.com/littlefs-project/littlefs.git
        GIT_TAG        v2.11.1
        GIT_SHALLOW    TRUE
    )
    FetchContent_GetProperties(littlefs)
    if(NOT littlefs_POPULATED)
        FetchContent_Populate(littlefs)
    endif()
    set(RTPS_LITTLEFS_DIR "${littlefs_SOURCE_DIR}")
    message(STATUS "Using fetched LittleFS: ${RTPS_LITTLEFS_DIR}")
endif()

if(EXISTS "${RTPS_LITTLEFS_DIR}/lfs.c" AND
   EXISTS "${RTPS_LITTLEFS_DIR}/lfs_util.c" AND
   EXISTS "${RTPS_LITTLEFS_DIR}/lfs.h")
    set(RTPS_HAVE_LITTLEFS ON)
else()
    message(FATAL_ERROR
        "LittleFS sources are incomplete in '${RTPS_LITTLEFS_DIR}'")
endif()
