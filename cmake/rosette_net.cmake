# cmake/rosette_net.cmake
#
# Defines `rosette_net`: the portable network core (rosette::net sockets,
# address management, SNTP) and the IP stack the board selected.
#
# Inputs:
#   ROSETTE_NET_STACK  a network port: wiznet | freertos_tcp | posix | ... A name
#                      is a port when ports/net/<name>/port.cmake exists here or
#                      under ROSETTE_PORT_DIRS (cmake/rosette_ports.cmake)
#   ROSETTE_ROOT       repository root (defaults to this file's parent directory)
#   rosette_platform   the platform target (cmake/rosette_platform.cmake), which
#                      must exist already
#
# The port's port.cmake runs with ROSETTE_PORT_DIR set to its directory, and
# adds its sources, include directories, and libraries to `rosette_net`.
#
# Firmware on the micro-ROS agent backend takes its time from the agent, so
# SNTP defaults to off there (net/sntp_client.hpp). ROSETTE_ROS_BACKEND_AGENT
# goes on rosette_net PUBLIC, so the whole firmware sees one value; it is on
# with ROSETTE_ENABLE_MICRO_ROS, or when a build that links the agent backend
# some other way asks for it.

if(TARGET rosette_net)
  return()
endif()

if(NOT DEFINED ROSETTE_ROOT)
  get_filename_component(ROSETTE_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." REALPATH)
endif()

if(NOT TARGET rosette_platform)
  message(FATAL_ERROR "rosette_net.cmake: include cmake/rosette_platform.cmake first")
endif()

include(${CMAKE_CURRENT_LIST_DIR}/rosette_ports.cmake)
rosette_require_port(_rn_port net "${ROSETTE_NET_STACK}" ROSETTE_NET_STACK)

option(ROSETTE_ROS_BACKEND_AGENT
  "The firmware uses the micro-ROS agent backend (the agent is the time source; SNTP off by default)"
  OFF)

add_library(rosette_net STATIC
  ${ROSETTE_ROOT}/src/net/socket.cpp
  ${ROSETTE_ROOT}/src/net/sntp_client.cpp
  ${ROSETTE_ROOT}/src/net/sntp_wire.cpp
  ${ROSETTE_ROOT}/src/net/address_candidate.cpp
  ${ROSETTE_ROOT}/src/net/address_list.cpp
  ${ROSETTE_ROOT}/src/net/address_manager.cpp
  ${ROSETTE_ROOT}/src/net/network_config.cpp
  ${ROSETTE_ROOT}/src/support/fnv1a.cpp)
target_include_directories(rosette_net PUBLIC
  ${ROSETTE_ROOT}/include
  ${ROSETTE_ROOT}/include/rosette/net)
target_link_libraries(rosette_net PUBLIC rosette_platform)
if(ROSETTE_ROS_BACKEND_AGENT OR ROSETTE_ENABLE_MICRO_ROS)
  target_compile_definitions(rosette_net PUBLIC ROSETTE_ROS_BACKEND_AGENT)
endif()

set(ROSETTE_PORT_DIR "${_rn_port}")
include("${_rn_port}/port.cmake")
unset(ROSETTE_PORT_DIR)
