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
# Each ROS 2 service or client uses one RTPS writer and one RTPS reader.
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
set(RMW_ROSETTE_MAX_MATCHED_ENDPOINTS 16 CACHE STRING
    "Remote readers per local writer and remote writers per local reader")
set(RMW_ROSETTE_MAX_REMOTE_ENDPOINTS 128 CACHE STRING
    "Remote writers (and, separately, remote readers) known from SEDP; a stock rclpy/rclcpp node has about 9 of each")
set(RMW_ROSETTE_MAX_DATAGRAM_SIZE 65507 CACHE STRING
    "Largest UDP datagram sent or received: 65507 on hosts; 1472 (1500-byte MTU) where IP fragmentation is unavailable (WIZnet hardware UDP). Larger samples go out as DATA_FRAG; peers must cap their datagrams to this too")
set(RMW_ROSETTE_MAX_SAMPLE_SIZE 65532 CACHE STRING
    "Largest serialized message (publish, service request or reply) in octets; sizes the largest buffer-pool class (at most 65532)")
set(RMW_ROSETTE_LARGE_BUFFERS 48 CACHE STRING
    "Blocks in each of the two largest pool classes (16 KiB and RMW_ROSETTE_MAX_SAMPLE_SIZE): samples held in writer histories, reassembly, and take rings")

math(EXPR _rr_publishers
     "${RMW_ROSETTE_MAX_PUBLISHERS} + ${RMW_ROSETTE_MAX_NODES} * ${RMW_ROSETTE_BUILTIN_PUBLISHERS_PER_NODE}")
math(EXPR _rr_services
     "${RMW_ROSETTE_MAX_SERVICES} + ${RMW_ROSETTE_MAX_NODES} * ${RMW_ROSETTE_PARAMETER_SERVICES_PER_NODE}")
math(EXPR _rr_rtps_writers "${_rr_publishers} + ${_rr_services} + ${RMW_ROSETTE_MAX_CLIENTS}")
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
math(EXPR _rr_match_records "(${_rr_rtps_writers} + ${_rr_rtps_readers}) * ${RMW_ROSETTE_MAX_MATCHED_ENDPOINTS}")

if(RMW_ROSETTE_MAX_SAMPLE_SIZE GREATER 65532)
  message(FATAL_ERROR "RMW_ROSETTE_MAX_SAMPLE_SIZE must be at most 65532 (16-bit pool block sizes)")
endif()
if(RMW_ROSETTE_MAX_DATAGRAM_SIZE LESS 512 OR RMW_ROSETTE_MAX_DATAGRAM_SIZE GREATER 65507)
  message(FATAL_ERROR "RMW_ROSETTE_MAX_DATAGRAM_SIZE must be between 512 and 65507")
endif()

# Buffer pools (rosette/rtps/buf_pool.hpp): five size classes, chosen so
# each use lands in a block near its size.
#   class 0  256 B       control messages (HEARTBEAT, ACKNACK, GAP, NACK_FRAG)
#   class 1  512 B       SPDP/SEDP announcements; one per live local endpoint
#   class 2  2 KiB       one datagram (DATA/DATA_FRAG framing); small samples
#   class 3  16 KiB      medium samples
#   class 4  MAX_SAMPLE  large samples: writer histories, reassembly, take rings
# Counts are uint8_t. Large-sample blocks are the scarce resource: a writer
# keeps up to its history depth of samples until acknowledged, and a reader
# holds one per in-flight reassembly plus each unread take-ring entry.
function(_rr_clamp_count out value)
  if(value GREATER 255)
    set(value 255)
  endif()
  set(${out} ${value} PARENT_SCOPE)
endfunction()
math(EXPR _rr_pool0 "32 + ${_rr_local_endpoints}")
math(EXPR _rr_pool1 "32 + 2 * ${_rr_local_endpoints}")
math(EXPR _rr_pool2 "64 + ${_rr_local_endpoints}")
_rr_clamp_count(_rr_pool0 ${_rr_pool0})
_rr_clamp_count(_rr_pool1 ${_rr_pool1})
_rr_clamp_count(_rr_pool2 ${_rr_pool2})
_rr_clamp_count(_rr_pool34 ${RMW_ROSETTE_LARGE_BUFFERS})

message(STATUS "rmw_rosette capacity: ${_rr_publishers} publishers, "
               "${RMW_ROSETTE_MAX_SUBSCRIPTIONS} subscriptions, ${_rr_services} services, "
               "${RMW_ROSETTE_MAX_CLIENTS} clients -> ${_rr_rtps_writers} RTPS writers, "
               "${_rr_rtps_readers} RTPS readers")

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
    RTPS_MAX_USER_WRITERS=${_rr_rtps_writers}
    RTPS_MAX_USER_READERS=${_rr_rtps_readers}
    RTPS_MAX_LOCAL_ENDPOINTS=${_rr_local_endpoints}
    RTPS_MAX_PEERS=${RMW_ROSETTE_MAX_REMOTE_PARTICIPANTS}
    RTPS_MAX_MATCHED_READERS=${RMW_ROSETTE_MAX_MATCHED_ENDPOINTS}
    RTPS_MAX_MATCHED_WRITERS=${RMW_ROSETTE_MAX_MATCHED_ENDPOINTS}
    RTPS_MAX_DISCOVERED_WRITERS=${_rr_known_writers}
    RTPS_MAX_DISCOVERED_READERS=${_rr_known_readers}
    RTPS_MAX_MATCH_RECORDS=${_rr_match_records}
    RTPS_POOL_CLASS0_SIZE=256u
    RTPS_POOL_CLASS0_COUNT=${_rr_pool0}u
    RTPS_POOL_CLASS1_SIZE=512u
    RTPS_POOL_CLASS1_COUNT=${_rr_pool1}u
    RTPS_POOL_CLASS2_SIZE=2048u
    RTPS_POOL_CLASS2_COUNT=${_rr_pool2}u
    RTPS_POOL_CLASS3_SIZE=16384u
    RTPS_POOL_CLASS3_COUNT=${_rr_pool34}u
    RTPS_POOL_CLASS4_SIZE=${RMW_ROSETTE_MAX_SAMPLE_SIZE}u
    RTPS_POOL_CLASS4_COUNT=${_rr_pool34}u
  )
endfunction()
