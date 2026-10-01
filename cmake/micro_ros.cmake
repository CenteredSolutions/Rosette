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
option(ROSETTE_MICRO_ROS_FETCH_SETUP
    "Fetch the official micro_ros_setup repository when no local path is provided"
    ON)
option(ROSETTE_MICRO_ROS_RUN_ROSDEP
    "Run rosdep install while bootstrapping the micro-ROS workspace"
    OFF)
option(ROSETTE_MICRO_ROS_RMW_ERROR_HANDLING
    "Enable rmw_microxrcedds internal error reporting so rmw_get_error_string() is populated on init failures"
    ON)

set(ROSETTE_MICRO_ROS_DISTRO "humble" CACHE STRING
    "ROS 2 distro / micro_ros_setup branch to use")
set_property(CACHE ROSETTE_MICRO_ROS_DISTRO PROPERTY STRINGS
    humble jazzy kilted rolling)

set(ROSETTE_MICRO_ROS_SETUP_PATH "" CACHE PATH
    "Path to a local micro_ros_setup checkout")
set(ROSETTE_MICRO_ROS_EXTRA_INTERFACE_PACKAGES "" CACHE STRING
    "Semicolon-separated list of extra ROS 2 interface package directories \
(each containing package.xml) to compile into libmicroros, in addition to any \
found under examples/")
set(ROSETTE_MICRO_ROS_WORKSPACE_DIR "${CMAKE_BINARY_DIR}/micro_ros_ws" CACHE PATH
    "Workspace used to build the generated micro-ROS static library")
set(ROSETTE_MICRO_ROS_ROS_SETUP
    "/opt/ros/${ROSETTE_MICRO_ROS_DISTRO}/setup.bash" CACHE FILEPATH
    "ROS 2 setup script sourced before building micro_ros_setup")

set(ROSETTE_MICRO_ROS_MAX_NODES 2 CACHE STRING "rmw_microxrcedds max nodes")
set(ROSETTE_MICRO_ROS_MAX_PUBLISHERS 10 CACHE STRING "rmw_microxrcedds max publishers")
set(ROSETTE_MICRO_ROS_MAX_SUBSCRIPTIONS 10 CACHE STRING "rmw_microxrcedds max subscriptions")
set(ROSETTE_MICRO_ROS_MAX_SERVICES 8 CACHE STRING "rmw_microxrcedds max services")
set(ROSETTE_MICRO_ROS_MAX_CLIENTS 6 CACHE STRING "rmw_microxrcedds max clients")
set(ROSETTE_MICRO_ROS_MAX_HISTORY 8 CACHE STRING "rmw_microxrcedds max history depth")

# XRCE serialization buffers. The file-management service carries multi-KiB
# string payloads; these must be large enough to hold the biggest request or
# response, and the transport MTU large enough to keep an Ethernet datagram
# unfragmented. They are shared across every entity of the same kind.
set(ROSETTE_MICRO_ROS_MAX_INPUT_BUFFER_SIZE 8192 CACHE STRING
    "rmw_microxrcedds max input (deserialization) buffer size")
set(ROSETTE_MICRO_ROS_MAX_OUTPUT_BUFFER_SIZE 8192 CACHE STRING
    "rmw_microxrcedds max output (serialization) buffer size")
set(ROSETTE_MICRO_ROS_MAX_TRANSPORT_MTU 1400 CACHE STRING
    "Micro XRCE-DDS transport MTU (bytes per datagram)")
set(ROSETTE_MICRO_ROS_STREAM_HISTORY 8 CACHE STRING
    "Micro XRCE-DDS reliable stream fragment-buffer depth")
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

# --- Agent location for the agent examples (examples/led_files) --------------
# "mdns"  -> MdnsAgentLocator: browse a DNS-SD service or resolve a host name.
# "xrce"  -> MulticastAgentLocator: the upstream-compatible XRCE GET_INFO probe.
set(ROSETTE_MICRO_ROS_DISCOVERY "mdns" CACHE STRING
    "Agent autodiscovery mechanism: mdns or xrce")
set(ROSETTE_MICRO_ROS_MDNS_SERVICE "_microrosagent._udp.local" CACHE STRING
    "DNS-SD service type browsed by the mDNS Agent locator")
set(ROSETTE_MICRO_ROS_MDNS_HOSTNAME "" CACHE STRING
    "When set, the mDNS Agent locator resolves this host name instead of browsing a service")
set(ROSETTE_MICRO_ROS_MDNS_TIMEOUT_MS "1000" CACHE STRING
    "Per-attempt receive timeout for the mDNS Agent locator")
set(ROSETTE_MICRO_ROS_MDNS_ATTEMPTS "3" CACHE STRING
    "Number of mDNS query attempts for the mDNS Agent locator")

if(ROSETTE_MICRO_ROS_DISCOVERY STREQUAL "mdns")
    set(_rosette_discovery_mdns 1)
else()
    set(_rosette_discovery_mdns 0)
endif()

# Agent location settings, compiled into every target linking micro_ros_vendor
# (the agent examples pick their AgentLocator from these).
set(_rosette_micro_ros_agent_definitions
    ROSETTE_MICRO_ROS_AGENT_IP="${ROSETTE_MICRO_ROS_AGENT_IP}"
    ROSETTE_MICRO_ROS_AGENT_PORT=${ROSETTE_MICRO_ROS_AGENT_PORT}
    ROSETTE_MICRO_ROS_DISCOVERY_GROUP="${ROSETTE_MICRO_ROS_DISCOVERY_GROUP}"
    ROSETTE_MICRO_ROS_DISCOVERY_PORT="${ROSETTE_MICRO_ROS_DISCOVERY_PORT}"
    ROSETTE_MICRO_ROS_DISCOVERY_TIMEOUT_MS=${ROSETTE_MICRO_ROS_DISCOVERY_TIMEOUT_MS}
    ROSETTE_MICRO_ROS_DISCOVERY_ATTEMPTS=${ROSETTE_MICRO_ROS_DISCOVERY_ATTEMPTS}
    ROSETTE_MICRO_ROS_DISCOVERY_MDNS=${_rosette_discovery_mdns}
    ROSETTE_MICRO_ROS_MDNS_SERVICE="${ROSETTE_MICRO_ROS_MDNS_SERVICE}"
    ROSETTE_MICRO_ROS_MDNS_HOSTNAME="${ROSETTE_MICRO_ROS_MDNS_HOSTNAME}"
    ROSETTE_MICRO_ROS_MDNS_TIMEOUT_MS=${ROSETTE_MICRO_ROS_MDNS_TIMEOUT_MS}
    ROSETTE_MICRO_ROS_MDNS_ATTEMPTS=${ROSETTE_MICRO_ROS_MDNS_ATTEMPTS}
)

# The XRCE client always speaks through a Rosette socket: the custom transport
# on every platform, host included, so agent location (static, mDNS, XRCE
# multicast) and the socket path are the same code the board runs. See
# include/rosette/ros/agent_session_gate.hpp. The profile feeds both the
# prebuilt and the bootstrap paths, and micro_ros_colcon.meta.in.
set(_rosette_micro_ros_transport "custom")
set(ROSETTE_MICRO_ROS_USE_CUSTOM_TRANSPORT ON)
set(ROSETTE_MICRO_ROS_UCLIENT_UDP OFF)

set(_rosette_micro_ros_lib
    "${ROSETTE_MICRO_ROS_WORKSPACE_DIR}/firmware/build/libmicroros.a")
set(_rosette_micro_ros_include
    "${ROSETTE_MICRO_ROS_WORKSPACE_DIR}/firmware/build/include")

# ------------------------------------------------------------
# Fast path: use an already-built libmicroros.a in place.
#
# Point ROSETTE_MICRO_ROS_WORKSPACE_DIR at a prior generate_lib output and pass
# -DROSETTE_MICRO_ROS_BOOTSTRAP_ON_BUILD=OFF. This is the CI and offline path
# and skips micro_ros_setup entirely.
# ------------------------------------------------------------
if(NOT ROSETTE_MICRO_ROS_BOOTSTRAP_ON_BUILD AND
   EXISTS "${_rosette_micro_ros_lib}" AND
   EXISTS "${_rosette_micro_ros_include}")

    add_library(micro_ros_prebuilt STATIC IMPORTED GLOBAL)
    set_target_properties(micro_ros_prebuilt PROPERTIES
        IMPORTED_LOCATION "${_rosette_micro_ros_lib}"
        INTERFACE_INCLUDE_DIRECTORIES "${_rosette_micro_ros_include}")

    add_library(micro_ros_vendor INTERFACE)
    target_link_libraries(micro_ros_vendor INTERFACE micro_ros_prebuilt)
    target_compile_definitions(micro_ros_vendor INTERFACE ${_rosette_micro_ros_agent_definitions})

    add_custom_target(rosette_micro_ros_bootstrap
        COMMENT "micro-ROS: using prebuilt ${_rosette_micro_ros_lib}")

    set(ROSETTE_MICRO_ROS_LIBRARY "${_rosette_micro_ros_lib}" CACHE FILEPATH
        "Path to the generated micro-ROS static library" FORCE)
    set(ROSETTE_MICRO_ROS_INCLUDE_DIR "${_rosette_micro_ros_include}" CACHE PATH
        "Path to the generated micro-ROS include tree" FORCE)

    message(STATUS "micro-ROS: using prebuilt library")
    message(STATUS "  Library   : ${_rosette_micro_ros_lib}")
    message(STATUS "  Transport : ${_rosette_micro_ros_transport}")
    return()
endif()

if(DEFINED ENV{MICRO_ROS_SETUP_PATH} AND ROSETTE_MICRO_ROS_SETUP_PATH STREQUAL "")
    set(ROSETTE_MICRO_ROS_SETUP_PATH "$ENV{MICRO_ROS_SETUP_PATH}")
endif()

if(DEFINED ROSETTE_MICRO_ROS_SETUP_PATH AND
   NOT ROSETTE_MICRO_ROS_SETUP_PATH STREQUAL "" AND
   EXISTS "${ROSETTE_MICRO_ROS_SETUP_PATH}/CMakeLists.txt")
    set(_rosette_micro_ros_setup_source_dir "${ROSETTE_MICRO_ROS_SETUP_PATH}")
    message(STATUS "Using local micro_ros_setup: ${_rosette_micro_ros_setup_source_dir}")
elseif(ROSETTE_MICRO_ROS_FETCH_SETUP)
    set(FETCHCONTENT_QUIET OFF)
    message(STATUS "Fetching micro_ros_setup (${ROSETTE_MICRO_ROS_DISTRO}) — first run only")
    FetchContent_Declare(micro_ros_setup
        GIT_REPOSITORY https://github.com/micro-ROS/micro_ros_setup.git
        GIT_TAG        ${ROSETTE_MICRO_ROS_DISTRO}
        GIT_SHALLOW    TRUE
        GIT_PROGRESS   TRUE
        # micro_ros_setup is an ament package; download it, don't configure it
        # (the bootstrap script builds it with colcon).
        SOURCE_SUBDIR  _rosette_not_a_cmake_dir
    )
    FetchContent_MakeAvailable(micro_ros_setup)
    set(_rosette_micro_ros_setup_source_dir "${micro_ros_setup_SOURCE_DIR}")
    message(STATUS "Fetched micro_ros_setup: ${_rosette_micro_ros_setup_source_dir}")
else()
    message(FATAL_ERROR
        "ROSETTE_ENABLE_MICRO_ROS=ON but no micro_ros_setup source was provided. "
        "Set ROSETTE_MICRO_ROS_SETUP_PATH or enable ROSETTE_MICRO_ROS_FETCH_SETUP.")
endif()

set(_rosette_micro_ros_arch_flags "")
set(ROSETTE_MICRO_ROS_USE_PICO_TIME OFF)
if(RTPS_PROCESSOR STREQUAL "RP2040" OR PICO_PLATFORM STREQUAL "rp2040")
    set(_rosette_micro_ros_arch_flags "-mcpu=cortex-m0plus -mthumb")
    set(ROSETTE_MICRO_ROS_USE_PICO_TIME ON)
elseif(RTPS_PROCESSOR STREQUAL "RP2350" OR PICO_PLATFORM MATCHES "^rp2350(-arm-s)?$")
    set(_rosette_micro_ros_arch_flags "-mcpu=cortex-m33 -mthumb")
    set(ROSETTE_MICRO_ROS_USE_PICO_TIME ON)
endif()

# Allow callers (e.g. micropython.cmake) to tell us which time API is
# available in the final link.  When building inside MicroPython the
# mp_hal_time_ns / mp_hal_ticks_us symbols are available; for bare Pico SDK
# builds we fall back to time_us_64 directly.
option(ROSETTE_MICRO_ROS_USE_MPHAL_TIME
    "Use MicroPython HAL time functions (mp_hal_time_ns / mp_hal_ticks_us) in the rcutils time patch"
    OFF)

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
set(ROSETTE_MICRO_ROS_USE_MPHAL_TIME_TEXT "OFF")
if(ROSETTE_MICRO_ROS_USE_MPHAL_TIME)
    set(ROSETTE_MICRO_ROS_USE_MPHAL_TIME_TEXT "ON")
endif()
set(ROSETTE_MICRO_ROS_RMW_ERROR_HANDLING_TEXT "OFF")
if(ROSETTE_MICRO_ROS_RMW_ERROR_HANDLING)
    set(ROSETTE_MICRO_ROS_RMW_ERROR_HANDLING_TEXT "ON")
endif()

# Extra interface packages compiled into libmicroros: an interface package that
# an example carries lives at examples/<name>/<pkg>/ or
# examples/<name>/interfaces/<pkg>/. Deliberately NOT a recursive glob — that
# would pick up package.xml files inside build directories (the micro_ros_setup
# config tree, other vendored packages, ...). Plus anything explicitly listed
# in ROSETTE_MICRO_ROS_EXTRA_INTERFACE_PACKAGES.
file(GLOB _rosette_micro_ros_example_pkg_xmls
    "${CMAKE_CURRENT_SOURCE_DIR}/examples/*/*/package.xml"
    "${CMAKE_CURRENT_SOURCE_DIR}/examples/*/interfaces/*/package.xml")
set(_rosette_micro_ros_interface_packages "${ROSETTE_MICRO_ROS_EXTRA_INTERFACE_PACKAGES}")
set(_rosette_micro_ros_interface_files "")
foreach(_xml IN LISTS _rosette_micro_ros_example_pkg_xmls)
    get_filename_component(_pkg_dir "${_xml}" DIRECTORY)
    if(EXISTS "${_pkg_dir}/CMakeLists.txt")
        list(APPEND _rosette_micro_ros_interface_packages "${_pkg_dir}")
        file(GLOB _pkg_iface_files CONFIGURE_DEPENDS
            "${_pkg_dir}/package.xml" "${_pkg_dir}/CMakeLists.txt"
            "${_pkg_dir}/msg/*" "${_pkg_dir}/srv/*" "${_pkg_dir}/action/*")
        list(APPEND _rosette_micro_ros_interface_files ${_pkg_iface_files})
    endif()
endforeach()
list(REMOVE_DUPLICATES _rosette_micro_ros_interface_packages)
string(REPLACE ";" "\n" ROSETTE_MICRO_ROS_INTERFACE_PACKAGES
    "${_rosette_micro_ros_interface_packages}")

if(_rosette_micro_ros_interface_packages)
    message(STATUS "micro-ROS: extra interface packages:")
    foreach(_p IN LISTS _rosette_micro_ros_interface_packages)
        message(STATUS "    ${_p}")
    endforeach()
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

# The prebuilt fast path returns earlier; reaching here means the bootstrap
# script runs as a build step and produces libmicroros.a.
add_custom_command(
    OUTPUT "${_rosette_micro_ros_stamp}"
    COMMAND /bin/bash "${_rosette_micro_ros_bootstrap_script}"
    COMMAND "${CMAKE_COMMAND}" -E touch "${_rosette_micro_ros_stamp}"
    DEPENDS
        ${_rosette_micro_ros_interface_files}
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
target_compile_definitions(micro_ros_vendor INTERFACE ${_rosette_micro_ros_agent_definitions})

set(ROSETTE_MICRO_ROS_LIBRARY "${_rosette_micro_ros_lib}" CACHE FILEPATH
    "Path to the generated micro-ROS static library" FORCE)
set(ROSETTE_MICRO_ROS_INCLUDE_DIR "${_rosette_micro_ros_include}" CACHE PATH
    "Path to the generated micro-ROS include tree" FORCE)

message(STATUS "micro-ROS bootstrap enabled")
message(STATUS "  Distro        : ${ROSETTE_MICRO_ROS_DISTRO}")
message(STATUS "  Transport     : ${_rosette_micro_ros_transport} (Rosette sockets)")
message(STATUS "  Workspace     : ${ROSETTE_MICRO_ROS_WORKSPACE_DIR}")
message(STATUS "  Toolchain     : ${_rosette_micro_ros_toolchain_file}")
