# SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
# SPDX-License-Identifier: Apache-2.0

# MicroPython user module integration for Rosette.
#
# Expected usage:
#   set(USER_C_MODULES /abs/path/to/Rosette/bindings/micropython)
#   include(${MICROPY_DIR}/py/usermod.cmake)
#
# micro-ROS auto-setup
# --------------------
# By default this module enables ROSETTE_ENABLE_MICRO_ROS so that
# micro_ros.cmake will auto-fetch, configure, and build the micro-ROS static
# library as part of the normal CMake build.  You may override the behaviour
# with the same cache variables documented in cmake/micro_ros.cmake:
#
#   ROSETTE_ENABLE_MICRO_ROS          – set OFF to skip micro-ROS entirely
#   ROSETTE_MICRO_ROS_INCLUDE_DIR     – pre-built include tree (skips bootstrap)
#   ROSETTE_MICRO_ROS_LIBRARY         – pre-built libmicroros.a  (skips bootstrap)

set(ROSETTE_BINDINGS_DIR ${CMAKE_CURRENT_LIST_DIR})
get_filename_component(ROSETTE_REPO_DIR ${ROSETTE_BINDINGS_DIR}/../.. ABSOLUTE)

# ── transport default for MicroPython builds ──────────────────────────────────
# The micro_ros.cmake transport cache variable is named ROSETTE_MICRO_ROS_TRANSPORT
# (values: auto | udp | custom).  Default to udp for MicroPython targets unless
# the caller has already set a value.
if(NOT DEFINED ROSETTE_MICRO_ROS_TRANSPORT)
    set(ROSETTE_MICRO_ROS_TRANSPORT "udp" CACHE STRING
        "Rosette MicroPython backend transport mode: auto | udp | custom")
    set_property(CACHE ROSETTE_MICRO_ROS_TRANSPORT PROPERTY STRINGS auto udp custom)
endif()

# ── pre-built paths (optional override) ──────────────────────────────────────
set(ROSETTE_MICRO_ROS_INCLUDE_DIR "" CACHE PATH
    "Include directory for a prebuilt micro-ROS generate_lib output")
set(ROSETTE_MICRO_ROS_LIBRARY "" CACHE FILEPATH
    "Path to a prebuilt libmicroros.a for the MicroPython Rosette binding")

# ── legacy: pull paths from a neighbouring build-micro-ros CMakeCache.txt ────
set(_rosette_micro_ros_cache_file ${ROSETTE_REPO_DIR}/build-micro-ros/CMakeCache.txt)
if(EXISTS ${_rosette_micro_ros_cache_file})
    if(ROSETTE_MICRO_ROS_INCLUDE_DIR STREQUAL "")
        file(STRINGS ${_rosette_micro_ros_cache_file}
            _rosette_micro_ros_include_line
            REGEX "^ROSETTE_MICRO_ROS_INCLUDE_DIR:PATH=")
        if(_rosette_micro_ros_include_line)
            string(REGEX REPLACE "^ROSETTE_MICRO_ROS_INCLUDE_DIR:PATH=" ""
                ROSETTE_MICRO_ROS_INCLUDE_DIR "${_rosette_micro_ros_include_line}")
        endif()
    endif()
    if(ROSETTE_MICRO_ROS_LIBRARY STREQUAL "")
        file(STRINGS ${_rosette_micro_ros_cache_file}
            _rosette_micro_ros_library_line
            REGEX "^ROSETTE_MICRO_ROS_LIBRARY:FILEPATH=")
        if(_rosette_micro_ros_library_line)
            string(REGEX REPLACE "^ROSETTE_MICRO_ROS_LIBRARY:FILEPATH=" ""
                ROSETTE_MICRO_ROS_LIBRARY "${_rosette_micro_ros_library_line}")
        endif()
    endif()
endif()

# ── auto-setup via micro_ros.cmake ────────────────────────────────────────────
# If neither a pre-built library nor include tree has been found yet, enable
# the micro_ros.cmake bootstrap so the workspace is created and built
# automatically.  If the target micro_ros_prebuilt already exists (because the
# parent project already included micro_ros.cmake) we skip the include to avoid
# re-running the module.
if(NOT TARGET micro_ros_prebuilt)
    set(_rosette_need_bootstrap TRUE)
    if(ROSETTE_MICRO_ROS_INCLUDE_DIR STREQUAL "" OR
       ROSETTE_MICRO_ROS_LIBRARY STREQUAL "" OR
       NOT EXISTS "${ROSETTE_MICRO_ROS_INCLUDE_DIR}/rcl/rcl.h" OR
       NOT EXISTS "${ROSETTE_MICRO_ROS_LIBRARY}")
        set(_rosette_need_bootstrap TRUE)
    endif()

    if(_rosette_need_bootstrap)
        # Enable the bootstrap module and use the current binary dir as workspace
        # unless the caller has already configured a workspace location.
        if(NOT DEFINED ROSETTE_ENABLE_MICRO_ROS)
            set(ROSETTE_ENABLE_MICRO_ROS ON CACHE BOOL
                "Enable micro-ROS bootstrap for the MicroPython Rosette binding" FORCE)
        endif()
        if(NOT DEFINED ROSETTE_MICRO_ROS_WORKSPACE_DIR)
            set(ROSETTE_MICRO_ROS_WORKSPACE_DIR
                "${CMAKE_BINARY_DIR}/micro_ros_ws" CACHE PATH
                "Workspace used to build the generated micro-ROS static library" FORCE)
        endif()
        # Inside a MicroPython firmware build the mp_hal_time_ns / mp_hal_ticks_us
        # symbols are available and provide properly calibrated wall-clock and
        # monotonic time.  Tell micro_ros.cmake to patch rcutils accordingly.
        if(NOT DEFINED ROSETTE_MICRO_ROS_USE_MPHAL_TIME)
            set(ROSETTE_MICRO_ROS_USE_MPHAL_TIME ON CACHE BOOL
                "Use MicroPython HAL time functions in the rcutils time patch" FORCE)
        endif()
        include(${ROSETTE_REPO_DIR}/cmake/micro_ros.cmake)
    endif()
endif()

# ── resolve final include / library paths ────────────────────────────────────
# After the bootstrap include the ROSETTE_MICRO_ROS_LIBRARY and
# ROSETTE_MICRO_ROS_INCLUDE_DIR cache variables are populated (FORCE) by
# micro_ros.cmake, so re-read them here.
set(ROSETTE_MICROPY_HAS_MICRO_ROS 1)
if(NOT EXISTS "${ROSETTE_MICRO_ROS_INCLUDE_DIR}/rcl/rcl.h" OR
   NOT EXISTS "${ROSETTE_MICRO_ROS_LIBRARY}")
    if(NOT TARGET micro_ros_prebuilt)
        message(FATAL_ERROR
            "Micro-ROS backend is required to publish messages from MicroPython.\n"
            "Either:\n"
            "  • Set ROSETTE_ENABLE_MICRO_ROS=ON so the workspace is built automatically, or\n"
            "  • Provide ROSETTE_MICRO_ROS_INCLUDE_DIR and ROSETTE_MICRO_ROS_LIBRARY pointing\n"
            "    to a pre-built micro-ROS generate_lib output.")
    endif()
    # The library will be produced at build time via the bootstrap target;
    # allow the configure step to succeed so the dependency chain is established.
endif()

set(ROSETTE_MICROPY_BACKEND_TRANSPORT_CUSTOM 0)
if(ROSETTE_MICRO_ROS_TRANSPORT STREQUAL "custom")
    set(ROSETTE_MICROPY_BACKEND_TRANSPORT_CUSTOM 1)
endif()

add_library(usermod_rosette INTERFACE)
target_sources(usermod_rosette INTERFACE
    ${ROSETTE_BINDINGS_DIR}/modrosette.c
    ${ROSETTE_BINDINGS_DIR}/rosette_mpy_obj_defs.c
    ${ROSETTE_BINDINGS_DIR}/rosette_mpy_backend_runtime.cpp
    ${ROSETTE_BINDINGS_DIR}/rosette_mpy_context.cpp
    ${ROSETTE_BINDINGS_DIR}/rosette_mpy_node.cpp
    ${ROSETTE_BINDINGS_DIR}/rosette_mpy_pubsub.cpp
    ${ROSETTE_BINDINGS_DIR}/rosette_mpy_schema.cpp
    ${ROSETTE_BINDINGS_DIR}/rosette_mpy_transport.cpp
    ${ROSETTE_BINDINGS_DIR}/rosette_mpy_stream.cpp)

target_include_directories(usermod_rosette INTERFACE
    ${ROSETTE_BINDINGS_DIR}
    ${ROSETTE_REPO_DIR}
    ${ROSETTE_REPO_DIR}/include)

if(ROSETTE_MICROPY_HAS_MICRO_ROS)
    if(TARGET micro_ros_prebuilt)
        # Use the imported target so CMake knows the library is produced by the
        # bootstrap custom command and will wait for it before linking.
        target_include_directories(usermod_rosette INTERFACE
            $<TARGET_PROPERTY:micro_ros_prebuilt,INTERFACE_INCLUDE_DIRECTORIES>)
        target_link_libraries(usermod_rosette INTERFACE micro_ros_prebuilt)
        add_dependencies(usermod_rosette micro_ros_prebuilt)
    else()
        # Pre-built paths supplied directly by the caller.
        target_include_directories(usermod_rosette INTERFACE
            ${ROSETTE_MICRO_ROS_INCLUDE_DIR})
        target_link_libraries(usermod_rosette INTERFACE
            ${ROSETTE_MICRO_ROS_LIBRARY})
    endif()
endif()

target_compile_options(usermod_rosette INTERFACE
    $<$<COMPILE_LANGUAGE:CXX>:-std=gnu++17>)
target_compile_definitions(usermod_rosette INTERFACE
    ROSETTE_MICROPY_HAS_MICRO_ROS=${ROSETTE_MICROPY_HAS_MICRO_ROS}
    ROSETTE_MICROPY_BACKEND_TRANSPORT_CUSTOM=${ROSETTE_MICROPY_BACKEND_TRANSPORT_CUSTOM}
    ROSETTE_MICRO_ROS_TRANSPORT_CUSTOM=${ROSETTE_MICROPY_BACKEND_TRANSPORT_CUSTOM})
target_link_libraries(usermod INTERFACE usermod_rosette)
