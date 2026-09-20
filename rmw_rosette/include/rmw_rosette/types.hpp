/**
 * @file rmw_rosette/types.hpp
 * @brief Internal payload structs stored behind each rmw_*_t::data pointer.
 *
 * rmw handles (rmw_node_t, rmw_publisher_t, ...) are opaque to callers; the
 * real state lives in one of these structs, allocated by the corresponding
 * rmw_create_*() and freed by rmw_destroy_*(). None of this is exposed
 * outside rmw_rosette — callers only ever see the rmw_*_t wrapper.
 *
 * CacheDepth is fixed at compile time here, not per-instance
 * -----------------------------------------------------------
 * rtps::RTPSParticipant::create_writer<CacheDepth, MaxReaders>() is a
 * function template — each depth is a distinct C++ type
 * (StatefulWriter<1,8> and StatefulWriter<8,8> do not share a base beyond
 * IStatefulWriter, which only exposes the participant-internal operations
 * acknack/tick/guid/matched_reader_add — not new_change()/write_reliable(),
 * which rmw_publish() needs).
 *
 * rmw_qos_profile_t.depth is a runtime value. Reconciling that with a
 * compile-time template parameter for every possible depth is real work
 * (a depth -> dispatch table, or a small closed set of instantiations)
 * that this scaffold defers. For now every publisher is instantiated at
 * RMW_ROSETTE_MAX_HISTORY_DEPTH and QoS requests are clamped to it; this
 * is enough to prove the shape out and is the first thing to revisit once
 * real profiles (KEEP_ALL, larger KEEP_LAST) show up.
 */

#pragma once

#include "rosette/rtps/participant.hpp"
#include "rosette/rtps/stateful_reader.hpp"
#include "rosette/rtps/stateful_writer.hpp"

#include <cstdint>

#ifndef RMW_ROSETTE_MAX_HISTORY_DEPTH
#  define RMW_ROSETTE_MAX_HISTORY_DEPTH 8
#endif

#ifndef RMW_ROSETTE_MAX_NODES
#  define RMW_ROSETTE_MAX_NODES 4
#endif

namespace rmw_rosette {

using PublisherWriter = rtps::StatefulWriter<RMW_ROSETTE_MAX_HISTORY_DEPTH,
                                              RTPS_MAX_MATCHED_READERS>;

/// Backing store for one rmw_context_t. Owns the single RTPSParticipant
/// this process/firmware image presents to the ROS graph.
struct Context {
    rtps::RTPSParticipant* participant = nullptr;
    uint8_t                domain_id   = 0;

    // TODO: transport ownership. RTPSParticipant takes an ITransport& by
    // reference (see rtps/participant.hpp); this struct needs to own or
    // reference the concrete rosette::net socket backend selected for the
    // target (POSIX / FreeRTOS+TCP / WIZnet), matching how rosette::App
    // wires transports today for the micro-ROS path.
};

/// Backing store for one rmw_node_t. RTPS has no first-class "node"
/// concept — this exists to satisfy rcl's node lifecycle and namespacing,
/// and to scope graph-cache queries (rmw_get_*_names_and_types) by node.
struct Node {
    Context*    context   = nullptr;
    const char* name      = nullptr;
    const char* namespace_ = nullptr;
};

/// Backing store for one rmw_publisher_t.
struct Publisher {
    Node*             node   = nullptr;
    PublisherWriter*  writer = nullptr;
    const void*       type_support = nullptr;  ///< rosidl_message_type_support_t*
    char              mangled_topic_name[RTPS_MAX_TOPIC_NAME_LEN] = {};
};

/// Backing store for one rmw_subscription_t.
struct Subscription {
    Node*                node   = nullptr;
    rtps::StatefulReader* reader = nullptr;
    const void*          type_support = nullptr; ///< rosidl_message_type_support_t*
    char                 mangled_topic_name[RTPS_MAX_TOPIC_NAME_LEN] = {};

    // TODO: a small ring of "change available" flags/queue populated by
    // StatefulReader::OnChangeAvailable, drained by rmw_take(). The reader
    // callback runs from RTPSParticipant::tick()/parse_received() context,
    // not from rmw_take() itself, so something has to buffer between them.
};

/// Service = a reader (requests) + a writer (replies), correlated via the
/// SampleIdentity/InlineQos machinery already present in rtps/types.hpp.
struct Service {
    Node*                node          = nullptr;
    rtps::StatefulReader* request_reader = nullptr;
    PublisherWriter*      reply_writer   = nullptr;
    const void*           type_support   = nullptr; ///< rosidl_service_type_support_t*
};

/// Client = the mirror image of Service.
struct Client {
    Node*                node           = nullptr;
    PublisherWriter*      request_writer = nullptr;
    rtps::StatefulReader* reply_reader   = nullptr;
    const void*           type_support   = nullptr;
};

/// Guard conditions have no RTPS analogue; they're a pure rmw_wait_set
/// signaling primitive (used by rcl for e.g. "wake the executor now").
struct GuardCondition {
    bool triggered = false;
};

}  // namespace rmw_rosette
