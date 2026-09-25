# The RTPS core's source files, listed once for every build that compiles it.
#
# The core is compiled by each consumer rather than linked as one library,
# because each consumer sizes the participant's tables and buffer pool with
# its own compile definitions (cmake/rosette_rtps.cmake for firmware that runs
# RTPS directly, rmw_rosette/cmake/rmw_rosette_capacity.cmake for rmw_rosette,
# the tests' own limits) and the one-definition rule needs every translation
# unit of an image to agree on them.
#
# Input:  ROSETTE_ROOT, the repository root.
# Output:
#   ROSETTE_RTPS_PROTOCOL_SOURCES  the protocol: message parsing, discovery,
#                                  writers and readers, participant (no I/O)
#   ROSETTE_RTPS_IO_SOURCES        the UDP transport over rosette::net sockets
#                                  and the participant driver that runs it
# A consumer also compiles one log backend (src/rtps/rtps_stdio_log.cpp or
# src/platform/posix/rtps_posix_log.cpp) unless its log level is 0.

if(NOT DEFINED ROSETTE_ROOT)
  get_filename_component(ROSETTE_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." REALPATH)
endif()

set(ROSETTE_RTPS_PROTOCOL_SOURCES
  ${ROSETTE_ROOT}/src/rtps/buf_pool.cpp
  ${ROSETTE_ROOT}/src/rtps/message_parser.cpp
  ${ROSETTE_ROOT}/src/rtps/participant.cpp
  ${ROSETTE_ROOT}/src/rtps/sedp.cpp
  ${ROSETTE_ROOT}/src/rtps/endpoint_match_table.cpp
  ${ROSETTE_ROOT}/src/rtps/spdp.cpp
  ${ROSETTE_ROOT}/src/rtps/stateful_reader.cpp
  ${ROSETTE_ROOT}/src/rtps/sn_window.cpp
  ${ROSETTE_ROOT}/src/rtps/writer_proxy.cpp
)

set(ROSETTE_RTPS_IO_SOURCES
  ${ROSETTE_ROOT}/src/rtps/participant_driver.cpp
  ${ROSETTE_ROOT}/src/rtps/transport.cpp
)
