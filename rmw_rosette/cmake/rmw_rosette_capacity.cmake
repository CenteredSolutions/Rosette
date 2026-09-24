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
set(RMW_ROSETTE_MAX_DATAGRAM_SIZE 65536 CACHE STRING
    "Receive buffer for one UDP datagram; stock DDS peers send up to 64 KiB, and truncated datagrams are lost")
set(RMW_ROSETTE_LARGE_BUFFERS 32 CACHE STRING
    "1 KiB and 2 KiB transport buffers (received samples in flight, outgoing user data)")

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

# Buffer pools (rosette/rtps/buf_pool.hpp). Every live SEDP announcement
# holds one 512-byte block (payload plus framing headroom) for as long as the
# endpoint exists, so that class scales with the local endpoint count; the
# smaller classes carry control traffic and short samples. Counts are uint8_t.
function(_rr_clamp_count out value)
  if(value GREATER 255)
    set(value 255)
  endif()
  set(${out} ${value} PARENT_SCOPE)
endfunction()
math(EXPR _rr_pool0 "32 + ${_rr_local_endpoints}")
math(EXPR _rr_pool1 "32 + ${_rr_local_endpoints}")
math(EXPR _rr_pool2 "32 + 2 * ${_rr_local_endpoints}")
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
    RTPS_MAX_USER_WRITERS=${_rr_rtps_writers}
    RTPS_MAX_USER_READERS=${_rr_rtps_readers}
    RTPS_MAX_LOCAL_ENDPOINTS=${_rr_local_endpoints}
    RTPS_MAX_PEERS=${RMW_ROSETTE_MAX_REMOTE_PARTICIPANTS}
    RTPS_MAX_MATCHED_READERS=${RMW_ROSETTE_MAX_MATCHED_ENDPOINTS}
    RTPS_MAX_MATCHED_WRITERS=${RMW_ROSETTE_MAX_MATCHED_ENDPOINTS}
    RTPS_MAX_DISCOVERED_WRITERS=${_rr_known_writers}
    RTPS_MAX_DISCOVERED_READERS=${_rr_known_readers}
    RTPS_MAX_MATCH_RECORDS=${_rr_match_records}
    RTPS_POOL_CLASS0_COUNT=${_rr_pool0}u
    RTPS_POOL_CLASS1_COUNT=${_rr_pool1}u
    RTPS_POOL_CLASS2_COUNT=${_rr_pool2}u
    RTPS_POOL_CLASS3_COUNT=${_rr_pool34}u
    RTPS_POOL_CLASS4_COUNT=${_rr_pool34}u
  )
endfunction()
