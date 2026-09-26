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
#   ROSETTE_RTPS_MAX_PEERS          remote participants tracked (default 8)
#   ROSETTE_RTPS_MAX_REMOTE_ENDPOINTS remote writers, and separately readers,
#                                   known from SEDP (default 64; a stock
#                                   rclpy/rclcpp node announces about 9 each).
#                                   When full, an endpoint that matches a
#                                   local one displaces one that matches none.
#   ROSETTE_RTPS_LOG_LEVEL          0-5 (default 2, warnings)
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
set(ROSETTE_RTPS_MAX_PEERS 8 CACHE STRING "Remote participants tracked at once")
set(ROSETTE_RTPS_MAX_REMOTE_ENDPOINTS 64 CACHE STRING "Remote writers (and readers) known from SEDP; when full, only those matching a local endpoint are admitted")
set(ROSETTE_RTPS_LOG_LEVEL 2 CACHE STRING "rosette::rtps log level (0-5)")

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
math(EXPR _rtps_match_records "8 * 8")

# Buffer pool. A sample is stored whole in one block (fragmented only on the
# wire), and alloc() takes the smallest class that fits, so each class is
# sized for what it holds:
#   256   control messages (HEARTBEAT, ACKNACK, GAP) and short samples
#   512   discovery: each SEDP builtin writer keeps one announcement per
#         local endpoint plus one disposal (about 150-350 octets each)
#   1024  received discovery data and mid-size samples
#   datagram  one datagram plus its RTPS headers (kRtpsDataHeadroom, 44
#         octets), rounded to 8: received DATA, built messages, and samples
#         that go out unfragmented
#   MAX_SAMPLE  whole samples larger than a datagram; never used by smaller
#         requests (buf_pool.hpp)
math(EXPR _rtps_pool_datagram "((${ROSETTE_RTPS_MAX_DATAGRAM_SIZE} + 44 + 7) / 8) * 8")
math(EXPR _rtps_pool_discovery "2 * (${_rtps_local_endpoints} + 1)")
if(NOT ROSETTE_RTPS_MAX_SAMPLE_SIZE GREATER _rtps_pool_datagram)
  message(FATAL_ERROR "ROSETTE_RTPS_MAX_SAMPLE_SIZE (${ROSETTE_RTPS_MAX_SAMPLE_SIZE}) must exceed "
                      "the datagram class (${_rtps_pool_datagram} octets)")
endif()
if(ROSETTE_RTPS_MAX_SAMPLE_SIZE GREATER 65532)
  message(FATAL_ERROR "ROSETTE_RTPS_MAX_SAMPLE_SIZE must be at most 65532 (16-bit block sizes)")
endif()

target_compile_definitions(rosette_rtps PUBLIC
  ROSETTE_RTPS_MAX_DATAGRAM_SIZE=${ROSETTE_RTPS_MAX_DATAGRAM_SIZE}
  RTPS_LOG_LEVEL=${ROSETTE_RTPS_LOG_LEVEL}
  RTPS_MAX_USER_WRITERS=4
  RTPS_MAX_USER_READERS=4
  RTPS_MAX_LOCAL_ENDPOINTS=${_rtps_local_endpoints}
  RTPS_MAX_PEERS=${ROSETTE_RTPS_MAX_PEERS}
  RTPS_MAX_MATCHED_READERS=8
  RTPS_MAX_MATCHED_WRITERS=8
  RTPS_MAX_DISCOVERED_WRITERS=${ROSETTE_RTPS_MAX_REMOTE_ENDPOINTS}
  RTPS_MAX_DISCOVERED_READERS=${ROSETTE_RTPS_MAX_REMOTE_ENDPOINTS}
  RTPS_MAX_MATCH_RECORDS=${_rtps_match_records}
  RTPS_POOL_CLASS0_SIZE=256u
  RTPS_POOL_CLASS0_COUNT=16u
  RTPS_POOL_CLASS1_SIZE=512u
  RTPS_POOL_CLASS1_COUNT=${_rtps_pool_discovery}u
  RTPS_POOL_CLASS2_SIZE=1024u
  RTPS_POOL_CLASS2_COUNT=4u
  RTPS_POOL_CLASS3_SIZE=${_rtps_pool_datagram}u
  RTPS_POOL_CLASS3_COUNT=4u
  RTPS_POOL_CLASS4_SIZE=${ROSETTE_RTPS_MAX_SAMPLE_SIZE}u
  RTPS_POOL_CLASS4_COUNT=${ROSETTE_RTPS_MAX_SAMPLE_BLOCKS}u
)
