# rmw_rosette capacity model.
#
# rmw_rosette allocates nothing at runtime: every entity comes from a fixed
# pool and every RTPS endpoint from a fixed participant table. This module is
# the one place those sizes are chosen. Applications set the limits below;
# everything else is derived, so a board configuration cannot size, say, the
# SEDP tables smaller than the endpoints its nodes will create.
#
# Client libraries create entities of their own for every node:
#   - rclcpp and rclpy create six parameter services per node (get, get_types,
#     set, set_atomically, list, describe). rclc creates them only when the
#     application starts rclc_parameter_server; set the per-node count to 0 on
#     a microcontroller that does not.
#   - Every node publishes /rosout (when rcl logging is enabled) and rclcpp and
#     rclpy nodes publish /parameter_events.
# Those per-node entities are added on top of the application's own counts.
#
# Each ROS 2 service or client uses one RTPS writer and one RTPS reader. The
# participant adds one writer of its own on ros_discovery_info, the topic that
# tells stock peers which nodes it hosts.
#
# The proxies of matched remote endpoints come from two participant-wide
# pools, one for the readers matched to local writers and one for the writers
# matched to local readers (docs/design/shared_proxy_pools.md). Every local
# endpoint is guaranteed RMW_ROSETTE_MIN_MATCHED_ENDPOINTS matches and may
# borrow up to RMW_ROSETTE_MAX_MATCHED_ENDPOINTS; the ros_discovery_info
# writer, which every active peer reads, is guaranteed one match per active
# peer. Left empty, each pool holds every endpoint at its cap at once; a
# firmware sizes them to the matches it has at once instead.
#
# include/rmw_rosette/capacity.hpp states the same relationships and
# static_asserts them, so an inconsistent override fails to compile.

set(RMW_ROSETTE_MAX_NODES 4 CACHE STRING
    "Nodes per process")
set(RMW_ROSETTE_MAX_PUBLISHERS 16 CACHE STRING
    "Application publishers, excluding per-node builtin publishers")
set(RMW_ROSETTE_MAX_SUBSCRIPTIONS 16 CACHE STRING
    "Application subscriptions")
set(RMW_ROSETTE_MAX_SERVICES 4 CACHE STRING
    "Application services, excluding per-node parameter services")
set(RMW_ROSETTE_MAX_CLIENTS 4 CACHE STRING
    "Application service clients")
set(RMW_ROSETTE_PARAMETER_SERVICES_PER_NODE 6 CACHE STRING
    "Parameter services each node creates (6 for rclcpp/rclpy; 0 for rclc without a parameter server)")
set(RMW_ROSETTE_BUILTIN_PUBLISHERS_PER_NODE 2 CACHE STRING
    "Publishers each node creates itself (/rosout, /parameter_events)")
set(RMW_ROSETTE_MAX_REMOTE_PARTICIPANTS 32 CACHE STRING
    "Remote DDS participants tracked at once (every ROS 2 process is one)")
set(RMW_ROSETTE_MAX_KNOWN_PARTICIPANTS ${RMW_ROSETTE_MAX_REMOTE_PARTICIPANTS} CACHE STRING
    "Remote DDS participants known at once, active or parked; size it for the whole network. Only RMW_ROSETTE_MAX_REMOTE_PARTICIPANTS of them exchange endpoint discovery at a time")
set(RMW_ROSETTE_MAX_MATCHED_ENDPOINTS 16 CACHE STRING
    "Remote readers per local writer and remote writers per local reader")
set(RMW_ROSETTE_MIN_MATCHED_ENDPOINTS 1 CACHE STRING
    "Remote matches every local endpoint is always granted, reserved in the proxy pools for every endpoint slot")
set(RMW_ROSETTE_MATCHED_READERS "" CACHE STRING
    "Remote readers matched across all local writers at once (the ReaderProxy pool); empty: every writer at RMW_ROSETTE_MAX_MATCHED_ENDPOINTS and ros_discovery_info at RMW_ROSETTE_MAX_REMOTE_PARTICIPANTS")
set(RMW_ROSETTE_MATCH_WAITERS 64 CACHE STRING
    "Matches waiting at once for a proxy their pool or cap refused; they bind oldest first as proxies free. Each costs one match record")
set(RMW_ROSETTE_MATCHED_WRITERS "" CACHE STRING
    "Remote writers matched across all local readers at once (the WriterProxy pool); empty: every reader at RMW_ROSETTE_MAX_MATCHED_ENDPOINTS")
set(RMW_ROSETTE_MAX_REMOTE_ENDPOINTS 128 CACHE STRING
    "Remote writers (and, separately, remote readers) known from SEDP; a stock rclpy/rclcpp node has about 9 of each. When full, an endpoint that matches a local one displaces one that matches none, so on a large network this need only cover the remote endpoints this process matches")
set(RMW_ROSETTE_MAX_DATAGRAM_SIZE 65507 CACHE STRING
    "Largest UDP datagram sent or received: 65507 on hosts; 1472 (1500-byte MTU) where IP fragmentation is unavailable (WIZnet hardware UDP). Larger samples go out as DATA_FRAG; peers must cap their datagrams to this too")
set(RMW_ROSETTE_MAX_SAMPLE_SIZE 65532 CACHE STRING
    "Largest serialized message (publish, service request or reply) in octets; sizes the largest buffer-pool class (at most 65532)")
set(RMW_ROSETTE_MAX_PENDING_CHANGES 10 CACHE STRING
    "Deepest KEEP_LAST history a subscription, service, or client keeps between takes; deeper requests and KEEP_ALL run at this depth (10 holds rmw_qos_profile_default whole)")
set(RMW_ROSETTE_LARGE_BUFFERS 48 CACHE STRING
    "Blocks of RMW_ROSETTE_MAX_SAMPLE_SIZE (and, in the host pool profile, of 16 KiB): samples held in writer histories, reassembly, and take rings")
set(RMW_ROSETTE_ASSEMBLY_SLOTS "" CACHE STRING
    "Fragmented samples in reassembly at once across all subscriptions, services, and clients; empty: one per block of the two largest buffer-pool classes, the blocks a reassembled sample can occupy")

math(EXPR _rr_publishers
     "${RMW_ROSETTE_MAX_PUBLISHERS} + ${RMW_ROSETTE_MAX_NODES} * ${RMW_ROSETTE_BUILTIN_PUBLISHERS_PER_NODE}")
math(EXPR _rr_services
     "${RMW_ROSETTE_MAX_SERVICES} + ${RMW_ROSETTE_MAX_NODES} * ${RMW_ROSETTE_PARAMETER_SERVICES_PER_NODE}")
math(EXPR _rr_rtps_writers "${_rr_publishers} + ${_rr_services} + ${RMW_ROSETTE_MAX_CLIENTS} + 1")
math(EXPR _rr_rtps_readers
     "${RMW_ROSETTE_MAX_SUBSCRIPTIONS} + ${_rr_services} + ${RMW_ROSETTE_MAX_CLIENTS}")
if(_rr_rtps_writers GREATER _rr_rtps_readers)
  set(_rr_local_endpoints ${_rr_rtps_writers})
else()
  set(_rr_local_endpoints ${_rr_rtps_readers})
endif()
if(_rr_local_endpoints GREATER 253)
  message(FATAL_ERROR "rmw_rosette: ${_rr_local_endpoints} local RTPS endpoints exceed the "
                      "SEDP history limit of 253; lower the RMW_ROSETTE_MAX_* limits")
endif()

# The endpoint match table holds local and remote endpoints alike, and
# tracks only pairs that involve a local endpoint.
math(EXPR _rr_known_writers "${_rr_rtps_writers} + ${RMW_ROSETTE_MAX_REMOTE_ENDPOINTS}")
math(EXPR _rr_known_readers "${_rr_rtps_readers} + ${RMW_ROSETTE_MAX_REMOTE_ENDPOINTS}")

# Proxy pools. The ros_discovery_info writer's floor and cap are both
# RMW_ROSETTE_MAX_REMOTE_PARTICIPANTS; every other endpoint's cap is
# RMW_ROSETTE_MAX_MATCHED_ENDPOINTS.
if(RMW_ROSETTE_MATCHED_READERS STREQUAL "")
  math(EXPR _rr_reader_proxies
       "(${_rr_rtps_writers} - 1) * ${RMW_ROSETTE_MAX_MATCHED_ENDPOINTS} + ${RMW_ROSETTE_MAX_REMOTE_PARTICIPANTS}")
else()
  set(_rr_reader_proxies ${RMW_ROSETTE_MATCHED_READERS})
endif()
if(RMW_ROSETTE_MATCHED_WRITERS STREQUAL "")
  math(EXPR _rr_writer_proxies "${_rr_rtps_readers} * ${RMW_ROSETTE_MAX_MATCHED_ENDPOINTS}")
else()
  set(_rr_writer_proxies ${RMW_ROSETTE_MATCHED_WRITERS})
endif()
math(EXPR _rr_min_reader_proxies
     "(${_rr_rtps_writers} - 1) * ${RMW_ROSETTE_MIN_MATCHED_ENDPOINTS} + ${RMW_ROSETTE_MAX_REMOTE_PARTICIPANTS}")
math(EXPR _rr_min_writer_proxies "${_rr_rtps_readers} * ${RMW_ROSETTE_MIN_MATCHED_ENDPOINTS}")
if(_rr_reader_proxies LESS _rr_min_reader_proxies OR _rr_reader_proxies GREATER 65534)
  message(FATAL_ERROR "rmw_rosette: RMW_ROSETTE_MATCHED_READERS (${_rr_reader_proxies}) must be "
                      "${_rr_min_reader_proxies} to 65534: one floor per writer and one proxy per "
                      "active peer for ros_discovery_info")
endif()
if(_rr_writer_proxies LESS _rr_min_writer_proxies OR _rr_writer_proxies GREATER 65534)
  message(FATAL_ERROR "rmw_rosette: RMW_ROSETTE_MATCHED_WRITERS (${_rr_writer_proxies}) must be "
                      "${_rr_min_writer_proxies} to 65534: one floor per reader")
endif()

# Match records: every bound match holds a proxy, so one record per proxy in
# both pools, plus the matches that wait for one.
if(RMW_ROSETTE_MATCH_WAITERS LESS 0)
  message(FATAL_ERROR "rmw_rosette: RMW_ROSETTE_MATCH_WAITERS must not be negative")
endif()
math(EXPR _rr_match_records
     "${_rr_reader_proxies} + ${_rr_writer_proxies} + ${RMW_ROSETTE_MATCH_WAITERS}")

if(RMW_ROSETTE_MAX_SAMPLE_SIZE GREATER 65532)
  message(FATAL_ERROR "RMW_ROSETTE_MAX_SAMPLE_SIZE must be at most 65532 (16-bit pool block sizes)")
endif()
if(RMW_ROSETTE_MAX_DATAGRAM_SIZE LESS 512 OR RMW_ROSETTE_MAX_DATAGRAM_SIZE GREATER 65507)
  message(FATAL_ERROR "RMW_ROSETTE_MAX_DATAGRAM_SIZE must be between 512 and 65507")
endif()

# Buffer pools (rosette/rtps/buf_pool.hpp): five size classes, chosen so
# each use lands in a block near its size. A sample is stored whole in one
# block (fragmented only on the wire) and alloc() takes the smallest class
# that fits. Counts are uint8_t. Large-sample blocks are the scarce resource:
# a writer keeps up to its history depth of samples until acknowledged, and a
# reader holds one per in-flight reassembly plus each unread take-ring entry.
#
# RMW_ROSETTE_POOL_PROFILE picks the layout:
#   host      class 0  256 B       control messages (HEARTBEAT, ACKNACK, GAP)
#             class 1  512 B       SPDP/SEDP announcements
#             class 2  2 KiB       DATA/DATA_FRAG framing; small samples
#             class 3  16 KiB      medium samples
#             class 4  MAX_SAMPLE  large samples
#   firmware  class 0  256 B       control messages and short samples
#             class 1  512 B       SPDP's announcement, each SEDP
#                                  announcement while it is rebuilt and sent,
#                                  received discovery data while it is
#                                  applied, samples of 257-512 octets
#             class 2  1 KiB       received discovery data, mid-size samples
#             class 3  datagram    one datagram plus its RTPS headers (44
#                                  octets), rounded to 8: received DATA,
#                                  built messages, unfragmented samples
#             class 4  MAX_SAMPLE  samples larger than one datagram
#   The firmware counts follow what a board holds at once; the host counts
#   leave room for many peers and deep histories.
#
# SEDP announcements (RTPS_SEDP_REBUILD_ANNOUNCEMENTS, rosette/rtps/sedp.hpp)
# follow the profile too. The firmware profile rebuilds each announcement
# from the endpoint table whenever it is sent, so class 1 holds only blocks
# in flight rather than two per local endpoint for the endpoint's lifetime:
# at 33 local endpoints that is 8 blocks instead of 68 (about 30 KiB). The
# host profile keeps each announcement serialized, so replaying it to many
# peers costs no serialization. -DRTPS_SEDP_REBUILD_ANNOUNCEMENTS=0 or =1
# overrides the profile's choice, and class 1 is sized for the mode chosen.
set(RMW_ROSETTE_POOL_PROFILE host CACHE STRING
    "Buffer-pool layout: host (large, many peers) or firmware (sized by role for a microcontroller)")
set_property(CACHE RMW_ROSETTE_POOL_PROFILE PROPERTY STRINGS host firmware)

function(_rr_clamp_count out value)
  if(value GREATER 255)
    set(value 255)
  endif()
  set(${out} ${value} PARENT_SCOPE)
endfunction()

if(DEFINED RTPS_SEDP_REBUILD_ANNOUNCEMENTS)
  set(_rr_sedp_rebuild ${RTPS_SEDP_REBUILD_ANNOUNCEMENTS})
elseif(RMW_ROSETTE_POOL_PROFILE STREQUAL "firmware")
  set(_rr_sedp_rebuild 1)
else()
  set(_rr_sedp_rebuild 0)
endif()
if(NOT _rr_sedp_rebuild MATCHES "^[01]$")
  message(FATAL_ERROR "RTPS_SEDP_REBUILD_ANNOUNCEMENTS must be 0 or 1")
endif()

if(RMW_ROSETTE_POOL_PROFILE STREQUAL "host")
  set(_rr_size0 256)
  set(_rr_size1 512)
  set(_rr_size2 2048)
  set(_rr_size3 16384)
  math(EXPR _rr_pool0 "32 + ${_rr_local_endpoints}")
  math(EXPR _rr_pool1 "32 + 2 * ${_rr_local_endpoints}")
  math(EXPR _rr_pool2 "64 + ${_rr_local_endpoints}")
  set(_rr_pool3 ${RMW_ROSETTE_LARGE_BUFFERS})
elseif(RMW_ROSETTE_POOL_PROFILE STREQUAL "firmware")
  set(_rr_size0 256)
  set(_rr_size1 512)
  set(_rr_size2 1024)
  math(EXPR _rr_size3 "((${RMW_ROSETTE_MAX_DATAGRAM_SIZE} + 44 + 7) / 8) * 8")
  if(NOT RMW_ROSETTE_MAX_SAMPLE_SIZE GREATER _rr_size3)
    message(FATAL_ERROR "rmw_rosette: RMW_ROSETTE_MAX_SAMPLE_SIZE (${RMW_ROSETTE_MAX_SAMPLE_SIZE}) must "
                        "exceed the datagram class (${_rr_size3} octets) in the firmware profile")
  endif()
  math(EXPR _rr_pool0 "16 + ${_rr_local_endpoints}")
  if(_rr_sedp_rebuild)
    set(_rr_pool1 8)
  else()
    math(EXPR _rr_pool1 "2 * (${_rr_local_endpoints} + 1)")
  endif()
  set(_rr_pool2 8)
  set(_rr_pool3 4)
else()
  message(FATAL_ERROR "RMW_ROSETTE_POOL_PROFILE='${RMW_ROSETTE_POOL_PROFILE}' is not host or firmware")
endif()
_rr_clamp_count(_rr_pool0 ${_rr_pool0})
_rr_clamp_count(_rr_pool1 ${_rr_pool1})
_rr_clamp_count(_rr_pool2 ${_rr_pool2})
_rr_clamp_count(_rr_pool3 ${_rr_pool3})
_rr_clamp_count(_rr_pool4 ${RMW_ROSETTE_LARGE_BUFFERS})

# Assembly slots (rosette/rtps/fragment_assembler.hpp): a slot holds the
# block its fragmented sample assembles in, so slots beyond the blocks of the
# two largest classes would never hold a sample.
if(RMW_ROSETTE_ASSEMBLY_SLOTS STREQUAL "")
  math(EXPR _rr_assembly_slots "${_rr_pool3} + ${_rr_pool4}")
else()
  set(_rr_assembly_slots ${RMW_ROSETTE_ASSEMBLY_SLOTS})
endif()
if(_rr_assembly_slots LESS 1 OR _rr_assembly_slots GREATER 65535)
  message(FATAL_ERROR "rmw_rosette: RMW_ROSETTE_ASSEMBLY_SLOTS (${_rr_assembly_slots}) must be 1 to 65535")
endif()

message(STATUS "rmw_rosette capacity: ${_rr_publishers} publishers, "
               "${RMW_ROSETTE_MAX_SUBSCRIPTIONS} subscriptions, ${_rr_services} services, "
               "${RMW_ROSETTE_MAX_CLIENTS} clients -> ${_rr_rtps_writers} RTPS writers, "
               "${_rr_rtps_readers} RTPS readers, ${_rr_reader_proxies} reader proxies, "
               "${_rr_writer_proxies} writer proxies, ${_rr_assembly_slots} assembly slots")

#[[
Applies the capacity model to a target as compile definitions.

Every translation unit of the target, including the RTPS core sources, sees
the same values, which the one-definition rule requires: the participant's
tables are sized by these macros in headers shared across both layers.
]]
function(rmw_rosette_apply_capacity target)
  target_compile_definitions(${target} PRIVATE
    RMW_ROSETTE_MAX_NODES=${RMW_ROSETTE_MAX_NODES}
    RMW_ROSETTE_MAX_PUBLISHERS=${RMW_ROSETTE_MAX_PUBLISHERS}
    RMW_ROSETTE_MAX_SUBSCRIPTIONS=${RMW_ROSETTE_MAX_SUBSCRIPTIONS}
    RMW_ROSETTE_MAX_SERVICES=${RMW_ROSETTE_MAX_SERVICES}
    RMW_ROSETTE_MAX_CLIENTS=${RMW_ROSETTE_MAX_CLIENTS}
    RMW_ROSETTE_PARAMETER_SERVICES_PER_NODE=${RMW_ROSETTE_PARAMETER_SERVICES_PER_NODE}
    RMW_ROSETTE_BUILTIN_PUBLISHERS_PER_NODE=${RMW_ROSETTE_BUILTIN_PUBLISHERS_PER_NODE}
    RMW_ROSETTE_MAX_DATAGRAM_SIZE=${RMW_ROSETTE_MAX_DATAGRAM_SIZE}
    RMW_ROSETTE_MAX_SAMPLE_SIZE=${RMW_ROSETTE_MAX_SAMPLE_SIZE}
    RMW_ROSETTE_MAX_PENDING_CHANGES=${RMW_ROSETTE_MAX_PENDING_CHANGES}
    RTPS_SEDP_REBUILD_ANNOUNCEMENTS=${_rr_sedp_rebuild}
    RTPS_MAX_USER_WRITERS=${_rr_rtps_writers}
    RTPS_MAX_USER_READERS=${_rr_rtps_readers}
    RTPS_MAX_LOCAL_ENDPOINTS=${_rr_local_endpoints}
    RTPS_MAX_PEERS=${RMW_ROSETTE_MAX_REMOTE_PARTICIPANTS}
    RTPS_MAX_KNOWN_PEERS=${RMW_ROSETTE_MAX_KNOWN_PARTICIPANTS}
    RTPS_MAX_MATCHED_READERS=${RMW_ROSETTE_MAX_MATCHED_ENDPOINTS}
    RTPS_MAX_MATCHED_WRITERS=${RMW_ROSETTE_MAX_MATCHED_ENDPOINTS}
    RTPS_MIN_MATCHES_PER_ENDPOINT=${RMW_ROSETTE_MIN_MATCHED_ENDPOINTS}
    RTPS_READER_PROXY_POOL=${_rr_reader_proxies}
    RTPS_WRITER_PROXY_POOL=${_rr_writer_proxies}
    RTPS_MAX_DISCOVERED_WRITERS=${_rr_known_writers}
    RTPS_MAX_DISCOVERED_READERS=${_rr_known_readers}
    RTPS_MAX_MATCH_RECORDS=${_rr_match_records}
    RTPS_MAX_WAITING_MATCHES=${RMW_ROSETTE_MATCH_WAITERS}
    RTPS_POOL_CLASS0_SIZE=${_rr_size0}u
    RTPS_POOL_CLASS0_COUNT=${_rr_pool0}u
    RTPS_POOL_CLASS1_SIZE=${_rr_size1}u
    RTPS_POOL_CLASS1_COUNT=${_rr_pool1}u
    RTPS_POOL_CLASS2_SIZE=${_rr_size2}u
    RTPS_POOL_CLASS2_COUNT=${_rr_pool2}u
    RTPS_POOL_CLASS3_SIZE=${_rr_size3}u
    RTPS_POOL_CLASS3_COUNT=${_rr_pool3}u
    RTPS_POOL_CLASS4_SIZE=${RMW_ROSETTE_MAX_SAMPLE_SIZE}u
    RTPS_POOL_CLASS4_COUNT=${_rr_pool4}u
    RTPS_ASSEMBLY_SLOTS=${_rr_assembly_slots}u
  )
endfunction()
