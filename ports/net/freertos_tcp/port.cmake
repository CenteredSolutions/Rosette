# SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
# SPDX-License-Identifier: Apache-2.0

# ports/net/freertos_tcp — FreeRTOS+TCP, adapted to rosette::net::SocketProvider.
#
# Included by cmake/rosette_net.cmake with ROSETTE_PORT_DIR set to this
# directory, after the `rosette_net` target exists. Needs ROSETTE_OS=freertos.
# Outputs, in rosette_net:
#   freertos_tcp_socket.cpp, freertos_tcp_igmp.cpp
#                                 the socket provider, and IGMP / MLD membership
#                                 (FreeRTOS+TCP has neither)
#   freertos_tcp_interface.hpp    NetworkInterface<Device>: a FreeRTOS+TCP interface
#                                 over any rosette::net::MacDevice (rosette/net/mac.hpp)
#   freertos_tcp_board_interface.hpp
#                                 the hook a board implements to register its
#                                 interface (boards/common/rp2_wiznet/ does, over
#                                 the WIZnet chip in MACRAW mode)
#   FreeRTOSIPConfig.h            the stack's configuration; a board may supply its
#                                 own by putting its directory first
#
# FreeRTOS+TCP comes from FREERTOS_TCP_PATH (environment or -D) when set, or is
# fetched.
#
# Nothing implements rosette/net/board_network.hpp over this stack yet, so no
# image links (PORTING.md, "Port status").

if(NOT TARGET freertos_kernel)
  message(FATAL_ERROR "ROSETTE_NET_STACK=freertos_tcp needs ROSETTE_OS=freertos")
endif()

include(FetchContent)
set(FETCHCONTENT_QUIET OFF)

# The board supplies the network interface. Left unset, FreeRTOS+TCP picks a
# port from the host system (libpcap on Linux/macOS), which is wrong for a
# firmware build.
set(FREERTOS_PLUS_TCP_NETWORK_IF "A_CUSTOM_NETWORK_IF" CACHE STRING "" FORCE)

if(DEFINED ENV{FREERTOS_TCP_PATH})
  set(FREERTOS_TCP_PATH "$ENV{FREERTOS_TCP_PATH}")
endif()

if(NOT TARGET freertos_plus_tcp)
  if(DEFINED FREERTOS_TCP_PATH AND EXISTS "${FREERTOS_TCP_PATH}")
    message(STATUS "Using local FreeRTOS+TCP: ${FREERTOS_TCP_PATH}")
    add_subdirectory(${FREERTOS_TCP_PATH} freertos_tcp EXCLUDE_FROM_ALL)
  else()
    message(STATUS "Fetching FreeRTOS-Plus-TCP V4.2.2 — first run only")
    FetchContent_Declare(freertos_tcp
      GIT_REPOSITORY https://github.com/FreeRTOS/FreeRTOS-Plus-TCP.git
      GIT_TAG        V4.2.2
      GIT_SHALLOW    TRUE
      GIT_PROGRESS   TRUE
      GIT_SUBMODULES "")
    FetchContent_MakeAvailable(freertos_tcp)
  endif()
  target_include_directories(freertos_plus_tcp PUBLIC "${ROSETTE_PORT_DIR}")
endif()

target_sources(rosette_net PRIVATE
  ${ROSETTE_PORT_DIR}/freertos_tcp_socket.cpp
  ${ROSETTE_PORT_DIR}/freertos_tcp_igmp.cpp)
target_include_directories(rosette_net PUBLIC ${ROSETTE_PORT_DIR})
target_link_libraries(rosette_net PUBLIC freertos_plus_tcp)
