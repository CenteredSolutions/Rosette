/**
 * @file rmw_rosette/types.hpp
 * @brief Internal payload structs stored behind each rmw_*_t::data pointer.
 *
 * rmw handles (rmw_node_t, rmw_publisher_t, ...) are opaque to callers; the
 * real state lives in one of these structs, allocated by the corresponding
 * rmw_create_*() and freed by rmw_destroy_*(). None of this is exposed
 * outside rmw_rosette — callers only ever see the rmw_*_t wrapper.
 *
 * CacheDepth is chosen at runtime, held type-erased
 * ---------------------------------------------------
 * rtps::RTPSParticipant::create_writer<CacheDepth, MaxReaders>() is a
 * function template — StatefulWriter<1,8> and StatefulWriter<8,8> are
 * distinct C++ types. rmw_qos_profile_t.depth is a runtime value, so
 * rmw_create_publisher() can't pick a template argument from it directly.
 *
 * Resolved by extending rtps::IStatefulWriter (Rosette's own type-erasure
 * base, not part of the RTPS wire contract) with write_reliable()/
 * write_async() — their signatures never mention CacheDepth, only the
 * writer's internal HistoryCache storage does — so every instantiation
 * satisfies the same interface for the write path, not just the
 * participant-internal operations it originally covered. See
 * qos.hpp::select_writer_depth() for the runtime dispatch over a small
 * closed set of depths this enables, and
 * tests/loopback/rtps_loopback_smoke.cpp for a working proof (it holds
 * the writer as IStatefulWriter*, not the concrete type, and publishes
 * through it).
 */

#pragma once

#include "rosette/rtps/participant.hpp"
#include "rosette/rtps/stateful_reader.hpp"
#include "rosette/rtps/stateful_writer.hpp"

#include <cstdint>

#ifndef RMW_ROSETTE_MAX_NODES
#  define RMW_ROSETTE_MAX_NODES 4
#endif

namespace rmw_rosette {

/// Every publisher is held through this type-erased interface regardless
/// of which CacheDepth qos.hpp::select_writer_depth() chose for it.
using PublisherWriter = rtps::IStatefulWriter;

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
