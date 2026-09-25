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
#   ROSETTE_RTPS_MAX_PEERS          remote participants tracked (default 8)
#   ROSETTE_RTPS_MAX_REMOTE_ENDPOINTS remote writers, and separately readers,
#                                   known from SEDP (default 64; a stock
#                                   rclpy/rclcpp node announces about 9 each)
#   ROSETTE_RTPS_LOG_LEVEL          0-5 (default 2, warnings)
#
# Output: STATIC library `rosette_rtps`, linked PUBLIC to rosette_platform and
# rosette_net, with ROSETTE_RTPS_MAX_DATAGRAM_SIZE as a compile definition
# for sizing the receive buffer and the transport limit.

if(TARGET rosette_rtps)
  return()
endif()

set(ROSETTE_RTPS_MAX_DATAGRAM_SIZE 1472 CACHE STRING "Largest UDP payload the link carries")
set(ROSETTE_RTPS_MAX_PEERS 8 CACHE STRING "Remote participants tracked at once")
set(ROSETTE_RTPS_MAX_REMOTE_ENDPOINTS 64 CACHE STRING "Remote writers (and readers) known from SEDP")
set(ROSETTE_RTPS_LOG_LEVEL 2 CACHE STRING "rosette::rtps log level (0-5)")

add_library(rosette_rtps STATIC
  ${ROSETTE_ROOT}/src/rtps/buf_pool.cpp
  ${ROSETTE_ROOT}/src/rtps/message_parser.cpp
  ${ROSETTE_ROOT}/src/rtps/participant.cpp
  ${ROSETTE_ROOT}/src/rtps/participant_driver.cpp
  ${ROSETTE_ROOT}/src/rtps/sedp.cpp
  ${ROSETTE_ROOT}/src/rtps/spdp.cpp
  ${ROSETTE_ROOT}/src/rtps/stateful_reader.cpp
  ${ROSETTE_ROOT}/src/rtps/sn_window.cpp
  ${ROSETTE_ROOT}/src/rtps/writer_proxy.cpp
  ${ROSETTE_ROOT}/src/rtps/transport.cpp
  ${ROSETTE_ROOT}/src/rtps/rtps_stdio_log.cpp
)
target_include_directories(rosette_rtps PUBLIC
  ${ROSETTE_ROOT}/include
  ${ROSETTE_ROOT}/include/rosette/rtps
)
target_link_libraries(rosette_rtps PUBLIC rosette_platform rosette_net)

# Local endpoints: a handful of topics per board. Pools: control messages,
# discovery announcements, one datagram, and two sample classes; about 60 KiB.
math(EXPR _rtps_match_records "8 * 8")
target_compile_definitions(rosette_rtps PUBLIC
  ROSETTE_RTPS_MAX_DATAGRAM_SIZE=${ROSETTE_RTPS_MAX_DATAGRAM_SIZE}
  RTPS_LOG_LEVEL=${ROSETTE_RTPS_LOG_LEVEL}
  RTPS_MAX_USER_WRITERS=4
  RTPS_MAX_USER_READERS=4
  RTPS_MAX_LOCAL_ENDPOINTS=8
  RTPS_MAX_PEERS=${ROSETTE_RTPS_MAX_PEERS}
  RTPS_MAX_MATCHED_READERS=8
  RTPS_MAX_MATCHED_WRITERS=8
  RTPS_MAX_DISCOVERED_WRITERS=${ROSETTE_RTPS_MAX_REMOTE_ENDPOINTS}
  RTPS_MAX_DISCOVERED_READERS=${ROSETTE_RTPS_MAX_REMOTE_ENDPOINTS}
  RTPS_MAX_MATCH_RECORDS=${_rtps_match_records}
  RTPS_POOL_CLASS0_SIZE=256u
  RTPS_POOL_CLASS0_COUNT=16u
  RTPS_POOL_CLASS1_SIZE=512u
  RTPS_POOL_CLASS1_COUNT=16u
  RTPS_POOL_CLASS2_SIZE=2048u
  RTPS_POOL_CLASS2_COUNT=8u
  RTPS_POOL_CLASS3_SIZE=4096u
  RTPS_POOL_CLASS3_COUNT=4u
  RTPS_POOL_CLASS4_SIZE=8192u
  RTPS_POOL_CLASS4_COUNT=2u
)
