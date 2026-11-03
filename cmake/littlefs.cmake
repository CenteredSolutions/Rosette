# SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
# SPDX-License-Identifier: Apache-2.0

# cmake/littlefs.cmake
#
# Locates or fetches LittleFS and exports:
#   ROSETTE_LITTLEFS_DIR   - path containing lfs.h, lfs.c, lfs_util.c
#   ROSETTE_HAVE_LITTLEFS  - ON when the required files are available

include(FetchContent)
set(FETCHCONTENT_QUIET OFF)

set(ROSETTE_HAVE_LITTLEFS OFF)

if(DEFINED ENV{LITTLEFS_PATH})
    set(LITTLEFS_PATH "$ENV{LITTLEFS_PATH}")
endif()

# Backward-compatible local checkout fallback.
set(_rosette_default_littlefs_dir "${CMAKE_CURRENT_SOURCE_DIR}/../littlefs")

if(DEFINED LITTLEFS_PATH AND EXISTS "${LITTLEFS_PATH}/lfs.c")
    set(ROSETTE_LITTLEFS_DIR "${LITTLEFS_PATH}")
    message(STATUS "Using local LittleFS: ${ROSETTE_LITTLEFS_DIR}")
elseif(EXISTS "${_rosette_default_littlefs_dir}/lfs.c")
    set(ROSETTE_LITTLEFS_DIR "${_rosette_default_littlefs_dir}")
    message(STATUS "Using local LittleFS: ${ROSETTE_LITTLEFS_DIR}")
else()
    message(STATUS "Fetching LittleFS v2.11.1 — first run only")
    FetchContent_Declare(littlefs
        GIT_REPOSITORY https://github.com/littlefs-project/littlefs.git
        GIT_TAG        v2.11.1
        GIT_SHALLOW    TRUE
        GIT_PROGRESS   TRUE
    )
    FetchContent_MakeAvailable(littlefs)
    set(ROSETTE_LITTLEFS_DIR "${littlefs_SOURCE_DIR}")
    message(STATUS "Fetched LittleFS: ${ROSETTE_LITTLEFS_DIR}")
endif()

if(EXISTS "${ROSETTE_LITTLEFS_DIR}/lfs.c" AND
   EXISTS "${ROSETTE_LITTLEFS_DIR}/lfs_util.c" AND
   EXISTS "${ROSETTE_LITTLEFS_DIR}/lfs.h")
    set(ROSETTE_HAVE_LITTLEFS ON)
else()
    message(FATAL_ERROR
        "LittleFS sources are incomplete in '${ROSETTE_LITTLEFS_DIR}'")
endif()
