# SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
# SPDX-License-Identifier: Apache-2.0

# rosette_rtps — the DDSI-RTPS core (src/rtps) for firmware builds.
#
# rmw_rosette compiles the same sources with its own capacity model
# (rmw_rosette/cmake/rmw_rosette_capacity.cmake); this target is for a board
# that runs the RTPS core directly. Every table and pool is fixed at build
# time, and the sizes are PUBLIC definitions because the participant's tables
# are laid out in headers that application code includes too (the
# one-definition rule needs every translation unit to agree).
#
# Board inputs (board.cmake, all optional):
#   ROSETTE_RTPS_MAX_DATAGRAM_SIZE  largest UDP payload the link carries:
#                                   1472 without IP fragmentation (WIZnet
#                                   hardware UDP), up to 65507 with an IP
#                                   stack that reassembles (default 1472)
#   ROSETTE_RTPS_MAX_SAMPLE_SIZE    largest serialized sample the board sends
#                                   or receives, in octets (default 4096);
#                                   a larger sample fails to publish and is
#                                   dropped as lost on receipt
#   ROSETTE_RTPS_MAX_SAMPLE_BLOCKS  samples larger than one datagram held at
#                                   once, in writer histories awaiting
#                                   acknowledgment and in reassembly
#                                   (default 2)
#   ROSETTE_RTPS_MAX_PEERS          remote participants exchanging endpoint
#                                   discovery at once (default 8)
#   ROSETTE_RTPS_MAX_KNOWN_PEERS    remote participants known, active or
#                                   parked; size it for the network (default 8)
#   ROSETTE_RTPS_MAX_REMOTE_ENDPOINTS remote writers, and separately readers,
#                                   known from SEDP (default 64; a stock
#                                   rclpy/rclcpp node announces about 9 each).
#                                   When full, an endpoint that matches a
#                                   local one displaces one that matches none.
#   ROSETTE_RTPS_LOG_LEVEL          0-5 (default 2, warnings)
#   ROSETTE_RTPS_POOL_STATS         ON to count buffer pool use per class
#                                   (rtps/buf_pool_stats.hpp) and warn when a
#                                   request gets no block (default OFF)
#   ROSETTE_POOL_<ROLE>_BLOCKS      CONTROL, DISCOVERY, MEDIUM, or DATAGRAM:
#                                   that buffer-pool class's block count
#                                   (cmake/rosette_buffer_pool.cmake); empty
#                                   keeps the measured default
#
# Output: STATIC library `rosette_rtps`, linked PUBLIC to rosette_platform and
# rosette_net, with ROSETTE_RTPS_MAX_DATAGRAM_SIZE as a compile definition
# for sizing the receive buffer and the transport limit.

if(TARGET rosette_rtps)
  return()
endif()

set(ROSETTE_RTPS_MAX_DATAGRAM_SIZE 1472 CACHE STRING "Largest UDP payload the link carries")
set(ROSETTE_RTPS_MAX_SAMPLE_SIZE 4096 CACHE STRING "Largest serialized sample sent or received")
set(ROSETTE_RTPS_MAX_SAMPLE_BLOCKS 2 CACHE STRING "Samples larger than one datagram held at once")
set(ROSETTE_RTPS_MAX_PEERS 8 CACHE STRING "Remote participants exchanging endpoint discovery at once")
set(ROSETTE_RTPS_MAX_KNOWN_PEERS 8 CACHE STRING "Remote participants known, active or parked")
set(ROSETTE_RTPS_MAX_REMOTE_ENDPOINTS 64 CACHE STRING "Remote writers (and readers) known from SEDP; when full, only those matching a local endpoint are admitted")
set(ROSETTE_RTPS_LOG_LEVEL 2 CACHE STRING "rosette::rtps log level (0-5)")
option(ROSETTE_RTPS_POOL_STATS "Count buffer pool use per class and warn on refusals" OFF)

include(${ROSETTE_ROOT}/cmake/rosette_rtps_sources.cmake)
add_library(rosette_rtps STATIC
  ${ROSETTE_RTPS_PROTOCOL_SOURCES}
  ${ROSETTE_RTPS_IO_SOURCES}
  ${ROSETTE_ROOT}/src/rtps/rtps_stdio_log.cpp
)
target_include_directories(rosette_rtps PUBLIC
  ${ROSETTE_ROOT}/include
  ${ROSETTE_ROOT}/include/rosette/rtps
)
target_link_libraries(rosette_rtps PUBLIC rosette_platform rosette_net)

# Local endpoints: a handful of topics per board.
set(_rtps_local_endpoints 8)
# Match records: one per proxy in the participant's two pools (four writers
# and four readers, eight matches each) plus the matches that wait for one.
set(_rtps_match_waiters 8)
math(EXPR _rtps_match_records "4 * 8 + 4 * 8 + ${_rtps_match_waiters}")

# Changes the participant's reliable readers hold behind a lost one, so each
# writer's changes are delivered in order (rosette/rtps/held_changes.hpp).
set(_rtps_held_changes 8)

# Buffer pool: cmake/rosette_buffer_pool.cmake picks the firmware layout,
# documents each class's role, size, and count, and prints it. The large
# class holds ROSETTE_RTPS_MAX_SAMPLE_BLOCKS blocks of
# ROSETTE_RTPS_MAX_SAMPLE_SIZE; ROSETTE_POOL_<ROLE>_BLOCKS overrides any
# other class's count. SEDP rebuilds each announcement for every send
# (sedp.hpp), so the discovery class holds only blocks in flight;
# -DRTPS_SEDP_REBUILD_ANNOUNCEMENTS=0 stores them instead, one block per
# local endpoint per builtin writer plus a disposal each.
if(NOT DEFINED RTPS_SEDP_REBUILD_ANNOUNCEMENTS)
  set(RTPS_SEDP_REBUILD_ANNOUNCEMENTS 1)
endif()

include(${ROSETTE_ROOT}/cmake/rosette_buffer_pool.cmake)
rosette_buffer_pool_layout(_rtps_pool
  LABEL rosette_rtps
  PROFILE firmware
  MAX_DATAGRAM_SIZE ${ROSETTE_RTPS_MAX_DATAGRAM_SIZE}
  MAX_SAMPLE_SIZE ${ROSETTE_RTPS_MAX_SAMPLE_SIZE}
  LARGE_BLOCKS ${ROSETTE_RTPS_MAX_SAMPLE_BLOCKS}
  LARGE_KNOB ROSETTE_RTPS_MAX_SAMPLE_BLOCKS
  LOCAL_ENDPOINTS ${_rtps_local_endpoints}
  HELD_CHANGES ${_rtps_held_changes}
  SEDP_REBUILD ${RTPS_SEDP_REBUILD_ANNOUNCEMENTS}
)

target_compile_definitions(rosette_rtps PUBLIC
  ROSETTE_RTPS_MAX_DATAGRAM_SIZE=${ROSETTE_RTPS_MAX_DATAGRAM_SIZE}
  RTPS_LOG_LEVEL=${ROSETTE_RTPS_LOG_LEVEL}
  RTPS_MAX_USER_WRITERS=4
  RTPS_MAX_USER_READERS=4
  RTPS_MAX_LOCAL_ENDPOINTS=${_rtps_local_endpoints}
  RTPS_SEDP_REBUILD_ANNOUNCEMENTS=${RTPS_SEDP_REBUILD_ANNOUNCEMENTS}
  RTPS_MAX_PEERS=${ROSETTE_RTPS_MAX_PEERS}
  RTPS_MAX_KNOWN_PEERS=${ROSETTE_RTPS_MAX_KNOWN_PEERS}
  RTPS_MAX_MATCHED_READERS=8
  RTPS_MAX_MATCHED_WRITERS=8
  RTPS_MAX_DISCOVERED_WRITERS=${ROSETTE_RTPS_MAX_REMOTE_ENDPOINTS}
  RTPS_MAX_DISCOVERED_READERS=${ROSETTE_RTPS_MAX_REMOTE_ENDPOINTS}
  RTPS_MAX_MATCH_RECORDS=${_rtps_match_records}
  RTPS_MAX_WAITING_MATCHES=${_rtps_match_waiters}
  RTPS_HELD_CHANGES=${_rtps_held_changes}u
  ${_rtps_pool_DEFINITIONS}
)
if(ROSETTE_RTPS_POOL_STATS)
  target_compile_definitions(rosette_rtps PUBLIC RTPS_BUFPOOL_STATS=1)
endif()
