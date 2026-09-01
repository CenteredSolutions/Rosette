# cmake/micro_ros.cmake
#
# Bootstraps a micro-ROS static library build using the official
# `micro_ros_setup` generate_lib workflow.
#
# Why this module exists
# ----------------------
# micro-ROS is not a single CMake package that can be linked with one
# `FetchContent_MakeAvailable()` call. The supported upstream path for custom
# build systems is the `micro_ros_setup` "generate_lib" flow, which downloads
# the required ROS 2 / micro-ROS packages and produces:
#
#   firmware/build/libmicroros.a
#   firmware/build/include/
#
# This module integrates that workflow into Rosette's top-level CMake build so
# the repository can auto-fetch and build a platform-matched micro-ROS bundle
# when requested.

include(FetchContent)

option(ROSETTE_ENABLE_MICRO_ROS
    "Enable micro-ROS bootstrap support in this build"
    OFF)

if(NOT ROSETTE_ENABLE_MICRO_ROS)
    return()
endif()

option(ROSETTE_MICRO_ROS_BOOTSTRAP_ON_BUILD
    "Build the micro-ROS static library automatically as part of the default build"
    ON)
option(ROSETTE_BUILD_MICRO_ROS_BOARD_SMOKE
    "Build the Rosette micro-ROS board smoke executable"
    ON)
option(ROSETTE_MICRO_ROS_FETCH_SETUP
    "Fetch the official micro_ros_setup repository when no local path is provided"
    ON)
option(ROSETTE_MICRO_ROS_RUN_ROSDEP
    "Run rosdep install while bootstrapping the micro-ROS workspace"
    OFF)

set(ROSETTE_MICRO_ROS_DISTRO "humble" CACHE STRING
    "ROS 2 distro / micro_ros_setup branch to use")
set_property(CACHE ROSETTE_MICRO_ROS_DISTRO PROPERTY STRINGS
    humble jazzy kilted rolling)

set(ROSETTE_MICRO_ROS_TRANSPORT "auto" CACHE STRING
    "micro-ROS transport profile to build: auto | custom | udp")
set_property(CACHE ROSETTE_MICRO_ROS_TRANSPORT PROPERTY STRINGS
    auto custom udp)

set(ROSETTE_MICRO_ROS_SETUP_PATH "" CACHE PATH
    "Path to a local micro_ros_setup checkout")
set(ROSETTE_MICRO_ROS_WORKSPACE_DIR "${CMAKE_BINARY_DIR}/micro_ros_ws" CACHE PATH
    "Workspace used to build the generated micro-ROS static library")
set(ROSETTE_MICRO_ROS_ROS_SETUP
    "/opt/ros/${ROSETTE_MICRO_ROS_DISTRO}/setup.bash" CACHE FILEPATH
    "ROS 2 setup script sourced before building micro_ros_setup")

set(ROSETTE_MICRO_ROS_MAX_NODES 2 CACHE STRING "rmw_microxrcedds max nodes")
set(ROSETTE_MICRO_ROS_MAX_PUBLISHERS 8 CACHE STRING "rmw_microxrcedds max publishers")
set(ROSETTE_MICRO_ROS_MAX_SUBSCRIPTIONS 8 CACHE STRING "rmw_microxrcedds max subscriptions")
set(ROSETTE_MICRO_ROS_MAX_SERVICES 4 CACHE STRING "rmw_microxrcedds max services")
set(ROSETTE_MICRO_ROS_MAX_CLIENTS 4 CACHE STRING "rmw_microxrcedds max clients")
set(ROSETTE_MICRO_ROS_MAX_HISTORY 8 CACHE STRING "rmw_microxrcedds max history depth")
set(ROSETTE_MICRO_ROS_EXTRA_C_FLAGS "" CACHE STRING
    "Extra C compiler flags appended to the generated micro-ROS toolchain")
set(ROSETTE_MICRO_ROS_EXTRA_CXX_FLAGS "" CACHE STRING
    "Extra C++ compiler flags appended to the generated micro-ROS toolchain")
set(ROSETTE_MICRO_ROS_AGENT_IP "" CACHE STRING
    "Optional agent IPv4 address used by Rosette board smoke targets")
set(ROSETTE_MICRO_ROS_AGENT_PORT "8888" CACHE STRING
    "Agent UDP port used by Rosette board smoke targets")
set(ROSETTE_MICRO_ROS_DISCOVERY_GROUP "239.255.0.2" CACHE STRING
    "IPv4 multicast group used by the upstream-compatible XRCE Agent locator")
set(ROSETTE_MICRO_ROS_DISCOVERY_PORT "7400" CACHE STRING
    "UDP port used by the upstream-compatible XRCE Agent locator")
set(ROSETTE_MICRO_ROS_DISCOVERY_TIMEOUT_MS "750" CACHE STRING
    "Per-attempt receive timeout for the Rosette Agent locator")
set(ROSETTE_MICRO_ROS_DISCOVERY_ATTEMPTS "3" CACHE STRING
    "Number of multicast probe attempts for the Rosette Agent locator")

if(DEFINED ENV{MICRO_ROS_SETUP_PATH} AND ROSETTE_MICRO_ROS_SETUP_PATH STREQUAL "")
    set(ROSETTE_MICRO_ROS_SETUP_PATH "$ENV{MICRO_ROS_SETUP_PATH}")
endif()

if(DEFINED ROSETTE_MICRO_ROS_SETUP_PATH AND
   NOT ROSETTE_MICRO_ROS_SETUP_PATH STREQUAL "" AND
   EXISTS "${ROSETTE_MICRO_ROS_SETUP_PATH}/CMakeLists.txt")
    set(_rosette_micro_ros_setup_source_dir "${ROSETTE_MICRO_ROS_SETUP_PATH}")
    message(STATUS "Using local micro_ros_setup: ${_rosette_micro_ros_setup_source_dir}")
elseif(ROSETTE_MICRO_ROS_FETCH_SETUP)
    FetchContent_Declare(micro_ros_setup
        GIT_REPOSITORY https://github.com/micro-ROS/micro_ros_setup.git
        GIT_TAG        ${ROSETTE_MICRO_ROS_DISTRO}
        GIT_SHALLOW    TRUE
    )
    FetchContent_GetProperties(micro_ros_setup)
    if(NOT micro_ros_setup_POPULATED)
        FetchContent_Populate(micro_ros_setup)
    endif()
    set(_rosette_micro_ros_setup_source_dir "${micro_ros_setup_SOURCE_DIR}")
    message(STATUS "Using fetched micro_ros_setup: ${_rosette_micro_ros_setup_source_dir}")
else()
    message(FATAL_ERROR
        "ROSETTE_ENABLE_MICRO_ROS=ON but no micro_ros_setup source was provided. "
        "Set ROSETTE_MICRO_ROS_SETUP_PATH or enable ROSETTE_MICRO_ROS_FETCH_SETUP.")
endif()

set(_rosette_micro_ros_transport "${ROSETTE_MICRO_ROS_TRANSPORT}")
if(_rosette_micro_ros_transport STREQUAL "auto")
    if(ROSETTE_BOARD_ENABLE_FREERTOS)
        set(_rosette_micro_ros_transport "custom")
    else()
        set(_rosette_micro_ros_transport "udp")
    endif()
endif()

set(_rosette_micro_ros_arch_flags "")
set(ROSETTE_MICRO_ROS_USE_PICO_TIME OFF)
if(ROSETTE_PROCESSOR STREQUAL "RP2040")
    set(_rosette_micro_ros_arch_flags "-mcpu=cortex-m0plus -mthumb")
    set(ROSETTE_MICRO_ROS_USE_PICO_TIME ON)
elseif(ROSETTE_PROCESSOR STREQUAL "RP2350")
    set(_rosette_micro_ros_arch_flags "-mcpu=cortex-m33 -mthumb")
    set(ROSETTE_MICRO_ROS_USE_PICO_TIME ON)
endif()

set(_rosette_micro_ros_common_c_flags
    "${_rosette_micro_ros_arch_flags} -ffunction-sections -fdata-sections ${ROSETTE_MICRO_ROS_EXTRA_C_FLAGS}")
set(_rosette_micro_ros_common_cxx_flags
    "${_rosette_micro_ros_arch_flags} -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti ${ROSETTE_MICRO_ROS_EXTRA_CXX_FLAGS}")

set(_rosette_micro_ros_toolchain_file
    "${CMAKE_BINARY_DIR}/generated/micro_ros/micro_ros_toolchain.cmake")
set(_rosette_micro_ros_colcon_meta_file
    "${CMAKE_BINARY_DIR}/generated/micro_ros/micro_ros_colcon.meta")
set(_rosette_micro_ros_bootstrap_script
    "${CMAKE_BINARY_DIR}/generated/micro_ros/bootstrap_micro_ros.sh")
file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/generated/micro_ros")

set(ROSETTE_MICRO_ROS_TOOLCHAIN_SYSTEM_NAME "Generic")
if(ROSETTE_PROCESSOR STREQUAL "HOST")
    set(ROSETTE_MICRO_ROS_TOOLCHAIN_SYSTEM_NAME "${CMAKE_SYSTEM_NAME}")
endif()

set(ROSETTE_MICRO_ROS_USE_CUSTOM_TRANSPORT OFF)
set(ROSETTE_MICRO_ROS_USE_UDP_TRANSPORT OFF)
set(ROSETTE_MICRO_ROS_USE_DISCOVERY OFF)
if(_rosette_micro_ros_transport STREQUAL "custom")
    set(ROSETTE_MICRO_ROS_USE_CUSTOM_TRANSPORT ON)
elseif(_rosette_micro_ros_transport STREQUAL "udp")
    set(ROSETTE_MICRO_ROS_USE_UDP_TRANSPORT ON)
    set(ROSETTE_MICRO_ROS_USE_DISCOVERY ON)
else()
    message(FATAL_ERROR
        "Unsupported ROSETTE_MICRO_ROS_TRANSPORT='${_rosette_micro_ros_transport}'. "
        "Expected custom or udp.")
endif()

set(ROSETTE_MICRO_ROS_SETUP_SOURCE_DIR "${_rosette_micro_ros_setup_source_dir}")
set(ROSETTE_MICRO_ROS_COMMON_C_FLAGS "${_rosette_micro_ros_common_c_flags}")
set(ROSETTE_MICRO_ROS_COMMON_CXX_FLAGS "${_rosette_micro_ros_common_cxx_flags}")
set(ROSETTE_MICRO_ROS_WORKSPACE_DIR_ESCAPED "${ROSETTE_MICRO_ROS_WORKSPACE_DIR}")
set(ROSETTE_MICRO_ROS_RUN_ROSDEP_TEXT "OFF")
if(ROSETTE_MICRO_ROS_RUN_ROSDEP)
    set(ROSETTE_MICRO_ROS_RUN_ROSDEP_TEXT "ON")
endif()
set(ROSETTE_MICRO_ROS_USE_PICO_TIME_TEXT "OFF")
if(ROSETTE_MICRO_ROS_USE_PICO_TIME)
    set(ROSETTE_MICRO_ROS_USE_PICO_TIME_TEXT "ON")
endif()

configure_file(
    "${CMAKE_CURRENT_LIST_DIR}/micro_ros_toolchain.cmake.in"
    "${_rosette_micro_ros_toolchain_file}"
    @ONLY)
configure_file(
    "${CMAKE_CURRENT_LIST_DIR}/micro_ros_colcon.meta.in"
    "${_rosette_micro_ros_colcon_meta_file}"
    @ONLY)
configure_file(
    "${CMAKE_CURRENT_LIST_DIR}/micro_ros_bootstrap.sh.in"
    "${_rosette_micro_ros_bootstrap_script}"
    @ONLY)

set(_rosette_micro_ros_stamp
    "${ROSETTE_MICRO_ROS_WORKSPACE_DIR}/.rosette_micro_ros_built")
set(_rosette_micro_ros_lib
    "${ROSETTE_MICRO_ROS_WORKSPACE_DIR}/firmware/build/libmicroros.a")
set(_rosette_micro_ros_include
    "${ROSETTE_MICRO_ROS_WORKSPACE_DIR}/firmware/build/include")

add_custom_command(
    OUTPUT "${_rosette_micro_ros_stamp}"
    COMMAND /bin/bash "${_rosette_micro_ros_bootstrap_script}"
    COMMAND "${CMAKE_COMMAND}" -E touch "${_rosette_micro_ros_stamp}"
    DEPENDS
        "${_rosette_micro_ros_toolchain_file}"
        "${_rosette_micro_ros_colcon_meta_file}"
        "${_rosette_micro_ros_bootstrap_script}"
    BYPRODUCTS
        "${_rosette_micro_ros_lib}"
    USES_TERMINAL
    COMMENT "Bootstrapping micro-ROS static library (${_rosette_micro_ros_transport})"
    VERBATIM)

if(ROSETTE_MICRO_ROS_BOOTSTRAP_ON_BUILD)
    add_custom_target(rosette_micro_ros_bootstrap ALL
        DEPENDS "${_rosette_micro_ros_stamp}")
else()
    add_custom_target(rosette_micro_ros_bootstrap
        DEPENDS "${_rosette_micro_ros_stamp}")
endif()

add_library(micro_ros_prebuilt STATIC IMPORTED GLOBAL)
set_target_properties(micro_ros_prebuilt PROPERTIES
    IMPORTED_LOCATION "${_rosette_micro_ros_lib}"
    INTERFACE_INCLUDE_DIRECTORIES "${_rosette_micro_ros_include}")
add_dependencies(micro_ros_prebuilt rosette_micro_ros_bootstrap)

add_library(micro_ros_vendor INTERFACE)
target_link_libraries(micro_ros_vendor INTERFACE micro_ros_prebuilt)
add_dependencies(micro_ros_vendor rosette_micro_ros_bootstrap)
target_compile_definitions(micro_ros_vendor INTERFACE
    ROSETTE_MICRO_ROS_TRANSPORT_CUSTOM=$<BOOL:${ROSETTE_MICRO_ROS_USE_CUSTOM_TRANSPORT}>
    ROSETTE_MICRO_ROS_TRANSPORT_UDP=$<BOOL:${ROSETTE_MICRO_ROS_USE_UDP_TRANSPORT}>
    ROSETTE_MICRO_ROS_AGENT_IP="${ROSETTE_MICRO_ROS_AGENT_IP}"
    ROSETTE_MICRO_ROS_AGENT_PORT=${ROSETTE_MICRO_ROS_AGENT_PORT}
    ROSETTE_MICRO_ROS_DISCOVERY_GROUP="${ROSETTE_MICRO_ROS_DISCOVERY_GROUP}"
    ROSETTE_MICRO_ROS_DISCOVERY_PORT="${ROSETTE_MICRO_ROS_DISCOVERY_PORT}"
    ROSETTE_MICRO_ROS_DISCOVERY_TIMEOUT_MS=${ROSETTE_MICRO_ROS_DISCOVERY_TIMEOUT_MS}
    ROSETTE_MICRO_ROS_DISCOVERY_ATTEMPTS=${ROSETTE_MICRO_ROS_DISCOVERY_ATTEMPTS})

set(ROSETTE_MICRO_ROS_LIBRARY "${_rosette_micro_ros_lib}" CACHE FILEPATH
    "Path to the generated micro-ROS static library" FORCE)
set(ROSETTE_MICRO_ROS_INCLUDE_DIR "${_rosette_micro_ros_include}" CACHE PATH
    "Path to the generated micro-ROS include tree" FORCE)

message(STATUS "micro-ROS bootstrap enabled")
message(STATUS "  Distro     : ${ROSETTE_MICRO_ROS_DISTRO}")
message(STATUS "  Transport  : ${_rosette_micro_ros_transport}")
message(STATUS "  Workspace  : ${ROSETTE_MICRO_ROS_WORKSPACE_DIR}")
message(STATUS "  Toolchain  : ${_rosette_micro_ros_toolchain_file}")
