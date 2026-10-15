/**
 * @file rmw_rosette/capacity.hpp
 * @brief Entity pool sizes, derived from the limits an application sets.
 *
 * rmw_rosette allocates nothing at runtime, so every limit is a compile-time
 * constant. cmake/rmw_rosette_capacity.cmake chooses them and passes the
 * results to every translation unit; this header restates how the pools
 * follow from those limits and static_asserts that the RTPS participant was
 * sized to hold them. An inconsistent manual override therefore fails to
 * compile instead of failing an entity creation at runtime.
 *
 * Application limits (RMW_ROSETTE_MAX_*) count only what the application
 * creates. Client libraries add per-node entities on top:
 *   - RMW_ROSETTE_PARAMETER_SERVICES_PER_NODE parameter services (6 for rclcpp
 *     and rclpy, 0 for rclc without rclc_parameter_server);
 *   - RMW_ROSETTE_BUILTIN_PUBLISHERS_PER_NODE publishers (/rosout,
 *     /parameter_events).
 * Each service and client is one RTPS writer plus one RTPS reader, and the
 * participant adds one writer on `ros_discovery_info`
 * (ros_discovery_info_writer.hpp).
 */

#pragma once

#include <cstdint>

#ifndef RMW_ROSETTE_MAX_NODES
#  define RMW_ROSETTE_MAX_NODES 4
#endif
#ifndef RMW_ROSETTE_MAX_PUBLISHERS
#  define RMW_ROSETTE_MAX_PUBLISHERS 16
#endif
#ifndef RMW_ROSETTE_MAX_SUBSCRIPTIONS
#  define RMW_ROSETTE_MAX_SUBSCRIPTIONS 16
#endif
#ifndef RMW_ROSETTE_MAX_SERVICES
#  define RMW_ROSETTE_MAX_SERVICES 4
#endif
#ifndef RMW_ROSETTE_MAX_CLIENTS
#  define RMW_ROSETTE_MAX_CLIENTS 4
#endif
#ifndef RMW_ROSETTE_PARAMETER_SERVICES_PER_NODE
#  define RMW_ROSETTE_PARAMETER_SERVICES_PER_NODE 6
#endif
#ifndef RMW_ROSETTE_BUILTIN_PUBLISHERS_PER_NODE
#  define RMW_ROSETTE_BUILTIN_PUBLISHERS_PER_NODE 2
#endif

#ifndef RMW_ROSETTE_MAX_DATAGRAM_SIZE
/// Largest UDP datagram sent or received. Writers fragment (DATA_FRAG) any
/// sample that would exceed it, and the receive buffer holds exactly one.
/// Peers must cap their datagrams to it as well (Fast DDS: maxMessageSize),
/// or their larger datagrams are truncated (counted in TransportStats).
#  define RMW_ROSETTE_MAX_DATAGRAM_SIZE 65507
#endif
#ifndef RMW_ROSETTE_MAX_SAMPLE_SIZE
/// Largest serialized message; the largest buffer-pool class.
#  define RMW_ROSETTE_MAX_SAMPLE_SIZE 65532
#endif

#ifndef RMW_ROSETTE_MAX_PENDING_CHANGES
/// Deepest KEEP_LAST history a subscription, service, or client keeps
/// between rmw takes (change_ring.hpp). Requests for a deeper history, and
/// KEEP_ALL, run at this depth and report it through their actual QoS.
/// The default holds `rmw_qos_profile_default` (depth 10) whole.
#  define RMW_ROSETTE_MAX_PENDING_CHANGES 10
#endif

#ifndef RMW_ROSETTE_MAX_WRITER_DEPTH
/// Deepest KEEP_LAST history a publisher keeps, 1 to 255. Deeper requests
/// and KEEP_ALL run at this depth and report it through their actual QoS
/// (qos.hpp). The writer slots that hold any writer are sized for it, and no
/// writer is deeper, the service reply writers included
/// (RMW_ROSETTE_REPLY_DEPTH).
#  define RMW_ROSETTE_MAX_WRITER_DEPTH 8
#endif

static_assert(RMW_ROSETTE_MAX_WRITER_DEPTH >= 1 && RMW_ROSETTE_MAX_WRITER_DEPTH <= 255,
              "RMW_ROSETTE_MAX_WRITER_DEPTH must be a writer depth, 1 to 255");

#ifndef RMW_ROSETTE_REPLY_DEPTH
/// Replies each service reply writer holds until their clients acknowledge
/// them, 1 to RMW_ROSETTE_MAX_WRITER_DEPTH. The reply history drops each reply once its client has
/// it (StatefulWriter::release_replies_on_delivery()), so this is the number of replies in flight
/// at once, not a history of past replies: a reply written while this many are unacknowledged
/// displaces the oldest of them, whose client then never receives it. Replies wait for a client
/// that is still matching (a new `ros2 service call` or `ros2 param` process), for its
/// acknowledgement, and for a repair after a lost datagram, so concurrent
/// clients need room beyond one. Every reply writer has this depth whatever
/// the service's QoS requests, and takes a shallow writer slot sized for it
/// (RTPS_SHALLOW_WRITER_DEPTH).
#  define RMW_ROSETTE_REPLY_DEPTH 4
#endif

static_assert(RMW_ROSETTE_REPLY_DEPTH >= 1
                  && RMW_ROSETTE_REPLY_DEPTH <= RMW_ROSETTE_MAX_WRITER_DEPTH,
              "RMW_ROSETTE_REPLY_DEPTH must be 1 to RMW_ROSETTE_MAX_WRITER_DEPTH");

namespace rmw_rosette {

/// Publisher pool: application publishers plus each node's own.
constexpr uint32_t kPublisherCapacity =
    RMW_ROSETTE_MAX_PUBLISHERS + RMW_ROSETTE_MAX_NODES * RMW_ROSETTE_BUILTIN_PUBLISHERS_PER_NODE;

/// Subscription pool.
constexpr uint32_t kSubscriptionCapacity = RMW_ROSETTE_MAX_SUBSCRIPTIONS;

/// Service pool: application services plus each node's parameter services.
constexpr uint32_t kServiceCapacity =
    RMW_ROSETTE_MAX_SERVICES + RMW_ROSETTE_MAX_NODES * RMW_ROSETTE_PARAMETER_SERVICES_PER_NODE;

/// Client pool.
constexpr uint32_t kClientCapacity = RMW_ROSETTE_MAX_CLIENTS;

/// Participant-level writers: the `ros_discovery_info` writer.
constexpr uint32_t kParticipantWriters = 1;

/// RTPS writers the participant must hold: publishers, service replies,
/// client requests, and the participant's own.
constexpr uint32_t kRtpsWriterCapacity =
    kPublisherCapacity + kServiceCapacity + kClientCapacity + kParticipantWriters;

/// RTPS readers the participant must hold: subscriptions, service requests,
/// client replies.
constexpr uint32_t kRtpsReaderCapacity =
    kSubscriptionCapacity + kServiceCapacity + kClientCapacity;

}  // namespace rmw_rosette

// The participant tables are sized by the build (rmw_rosette_capacity.cmake);
// these checks run wherever an RTPS header has been included first.
#ifdef RTPS_POOL_CLASS4_SIZE
static_assert(RTPS_POOL_CLASS4_SIZE >= RMW_ROSETTE_MAX_SAMPLE_SIZE,
              "the largest buffer-pool class must hold RMW_ROSETTE_MAX_SAMPLE_SIZE");
#endif
#ifdef RTPS_MAX_USER_WRITERS
static_assert(RTPS_DEEP_WRITER_DEPTH >= RMW_ROSETTE_MAX_WRITER_DEPTH,
              "the deep writer slots cannot hold a writer of RMW_ROSETTE_MAX_WRITER_DEPTH");
static_assert(RTPS_MAX_USER_WRITERS >= rmw_rosette::kRtpsWriterCapacity,
              "RTPS_MAX_USER_WRITERS cannot hold every publisher, service and client writer");
static_assert(RTPS_MAX_USER_READERS >= rmw_rosette::kRtpsReaderCapacity,
              "RTPS_MAX_USER_READERS cannot hold every subscription, service and client reader");
// Every service reply writer fits a shallow slot, and every other writer,
// which may be deeper, takes a deep slot. A reply writer asks for a shallow
// slot and never spills into a free deep one, so when the shallow class is
// enabled it needs a slot per service; with no shallow slots, replies take
// deep slots, which the deep-slot check below covers.
static_assert(RTPS_MAX_SHALLOW_WRITERS == 0 || RTPS_SHALLOW_WRITER_DEPTH >= RMW_ROSETTE_REPLY_DEPTH,
              "shallow writer slots cannot hold a reply writer of RMW_ROSETTE_REPLY_DEPTH");
static_assert(RTPS_MAX_SHALLOW_WRITERS == 0
                  || RTPS_MAX_SHALLOW_WRITERS >= rmw_rosette::kServiceCapacity,
              "RTPS_MAX_SHALLOW_WRITERS cannot hold a reply writer for every service");
static_assert(RTPS_MAX_USER_WRITERS - RTPS_MAX_SHALLOW_WRITERS
                  >= rmw_rosette::kRtpsWriterCapacity - rmw_rosette::kServiceCapacity,
              "the deep writer slots cannot hold every publisher, client and participant writer");
static_assert(RTPS_MAX_LOCAL_ENDPOINTS >= rmw_rosette::kRtpsWriterCapacity &&
              RTPS_MAX_LOCAL_ENDPOINTS >= rmw_rosette::kRtpsReaderCapacity,
              "RTPS_MAX_LOCAL_ENDPOINTS cannot announce every local endpoint through SEDP");
#endif
#ifdef RTPS_READER_PROXY_POOL
// The participant reserves every writer slot's floor when it is created, and
// the ros_discovery_info writer, created right after it, reserves the rest of
// its floor of one proxy per active peer (RosDiscoveryInfoWriter::create()).
static_assert(
    RTPS_MAX_PEERS <= 255 && RTPS_MAX_PEERS >= RTPS_MIN_MATCHES_PER_ENDPOINT,
    "the ros_discovery_info writer's floor, RTPS_MAX_PEERS, must fit an endpoint's floor");
static_assert(RTPS_READER_PROXY_POOL
                  >= (RTPS_MAX_USER_WRITERS - 1) * RTPS_MIN_MATCHES_PER_ENDPOINT + RTPS_MAX_PEERS,
              "RTPS_READER_PROXY_POOL cannot hold every writer's floor and one ros_discovery_info "
              "proxy per active peer");
#endif
