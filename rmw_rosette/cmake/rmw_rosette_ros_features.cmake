# rmw_rosette_ros_features: what the ROS 2 headers rmw_rosette builds against provide.
#
# rmw_rosette builds from one source tree against every supported ROS 2
# distribution (Humble, Jazzy) and against the micro-ROS headers a firmware
# image compiles with. Where those headers differ, the code branches on a
# feature macro, never on a distribution name: each macro names the API it
# guards, so a reviewer can check the fork against the header that defines it,
# and a distribution that backports a feature picks it up without a change here.
#
# Differences that are only a type or a size (rmw_request_id_t::writer_guid,
# RMW_GID_STORAGE_SIZE) need no probe; the code takes them from the header
# with decltype or the macro itself.
#
# Each probe compiles a few lines of C against the target's own include
# directories. Compiling as a static library, without linking, keeps the probes
# valid under a cross compiler (arm-none-eabi) that cannot link test programs.
# A declaration probe takes the function's address rather than calling it: an
# undeclared identifier there is an error in every C dialect, where a call
# would only warn under C89 implicit-declaration rules.
#
# The results land in a generated header, rmw_rosette/ros_features.h, whose
# macros are always defined to 0 or 1 (#cmakedefine01). Code tests them with
# `#if`, and on host builds -Wundef turns a misspelled or unincluded macro into
# a diagnostic instead of a silent 0. Entry points that exist only on some
# distributions live in their own source files, which the probe adds to the
# target.
#
# Usage:
#   include(rmw_rosette/cmake/rmw_rosette_ros_features.cmake)
#   rmw_rosette_probe_ros_features(<target> [INCLUDE_DIRS <dir>...]
#                                  [LIBRARIES <imported target>...])
#
# LIBRARIES takes imported targets (rmw::rmw, rosidl_runtime_c::...) whose
# include directories the probes compile with; INCLUDE_DIRS takes plain paths,
# as a firmware build that only has an install tree provides.

include(CheckCSourceCompiles)

set(_rmw_rosette_ros_features_dir ${CMAKE_CURRENT_LIST_DIR})

# Compiles SOURCE against the probe include directories and sets RESULT_VAR to
# 1 or 0 in the caller. check_c_source_compiles caches the outcome, so a
# reconfigure against the same headers does not repeat the probe.
function(_rmw_rosette_probe RESULT_VAR SOURCE)
  set(CMAKE_REQUIRED_INCLUDES ${_RMW_ROSETTE_PROBE_INCLUDES})
  set(CMAKE_REQUIRED_LIBRARIES ${_RMW_ROSETTE_PROBE_LIBRARIES})
  set(CMAKE_REQUIRED_QUIET ON)
  set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
  check_c_source_compiles("${SOURCE}" ${RESULT_VAR})
  if(${RESULT_VAR})
    set(${RESULT_VAR} 1 PARENT_SCOPE)
  else()
    set(${RESULT_VAR} 0 PARENT_SCOPE)
  endif()
endfunction()

# Probes the ROS 2 headers reachable from INCLUDE_DIRS and LIBRARIES, writes
# rmw_rosette/ros_features.h under the current binary directory, adds that
# directory to TARGET's private include path, compiles TARGET's C++ with -Wundef
# on host builds, and adds the source files of the entry points those headers
# declare.
function(rmw_rosette_probe_ros_features TARGET)
  cmake_parse_arguments(ARG "" "" "INCLUDE_DIRS;LIBRARIES" ${ARGN})
  set(_RMW_ROSETTE_PROBE_INCLUDES ${ARG_INCLUDE_DIRS})
  set(_RMW_ROSETTE_PROBE_LIBRARIES ${ARG_LIBRARIES})

  # rmw/discovery_options.h (Iron+): rmw_init_options_t carries
  # discovery_options (automatic discovery range, static peers) next to the
  # deprecated localhost_only.
  _rmw_rosette_probe(RMW_ROSETTE_HAS_DISCOVERY_OPTIONS [[
#include <rmw/init_options.h>
int probe(rmw_init_options_t * o) { return (int)o->discovery_options.automatic_discovery_range; }
]])

  # rosidl_runtime_c/type_hash.h (Iron+): message type supports carry the
  # RIHS01 hash of their type description.
  _rmw_rosette_probe(RMW_ROSETTE_HAS_TYPE_HASH [[
#include <rosidl_runtime_c/message_type_support_struct.h>
const void * probe(const rosidl_message_type_support_t * t) { return (const void *)t->get_type_hash_func; }
]])

  # rmw/rmw.h (Jazzy+): rmw_count_clients and rmw_count_services.
  _rmw_rosette_probe(RMW_ROSETTE_HAS_SERVICE_ENDPOINT_COUNTS [[
#include <rmw/rmw.h>
const void * probe(void) { return (const void *)&rmw_count_clients; }
]])

  # rmw/rmw.h (Iron+): rmw_get_gid_for_client.
  _rmw_rosette_probe(RMW_ROSETTE_HAS_CLIENT_GID [[
#include <rmw/rmw.h>
const void * probe(void) { return (const void *)&rmw_get_gid_for_client; }
]])

  # rmw/event.h (Iron+): rmw_event_type_is_supported.
  _rmw_rosette_probe(RMW_ROSETTE_HAS_EVENT_TYPE_QUERY [[
#include <rmw/event.h>
const void * probe(void) { return (const void *)&rmw_event_type_is_supported; }
]])

  # rmw/dynamic_message_type_support.h (Jazzy+): runtime (dynamic) type
  # support, rmw_take_dynamic_message and rmw_serialization_support_init.
  _rmw_rosette_probe(RMW_ROSETTE_HAS_DYNAMIC_TYPESUPPORT [[
#include <rmw/dynamic_message_type_support.h>
const void * probe(void) { return (const void *)&rmw_take_dynamic_message; }
]])

  foreach(_feature
      RMW_ROSETTE_HAS_DISCOVERY_OPTIONS
      RMW_ROSETTE_HAS_TYPE_HASH
      RMW_ROSETTE_HAS_SERVICE_ENDPOINT_COUNTS
      RMW_ROSETTE_HAS_CLIENT_GID
      RMW_ROSETTE_HAS_EVENT_TYPE_QUERY
      RMW_ROSETTE_HAS_DYNAMIC_TYPESUPPORT)
    message(STATUS "rmw_rosette ROS feature ${_feature}=${${_feature}}")
  endforeach()

  set(_out_dir ${CMAKE_CURRENT_BINARY_DIR}/rmw_rosette_generated)
  configure_file(${_rmw_rosette_ros_features_dir}/ros_features.h.in
                 ${_out_dir}/rmw_rosette/ros_features.h @ONLY)
  target_include_directories(${TARGET} PRIVATE ${_out_dir})
  # -Wundef applies to host builds only, and to their C++ alone. A firmware
  # target compiles board SDK and RTOS headers (Pico SDK, FreeRTOS portmacro.h)
  # whose #if tests rely on undefined macros, and its probes are all 0, the same
  # branches a Humble host build compiles; host builds on each distribution
  # compile both sides of every feature test, so a misspelled macro is caught there.
  if(NOT CMAKE_CROSSCOMPILING)
    target_compile_options(${TARGET} PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-Wundef>)
  endif()

  set(_src_dir ${_rmw_rosette_ros_features_dir}/../src)
  if(RMW_ROSETTE_HAS_DYNAMIC_TYPESUPPORT)
    target_sources(${TARGET} PRIVATE ${_src_dir}/rmw_dynamic_message.cpp)
  endif()
endfunction()
