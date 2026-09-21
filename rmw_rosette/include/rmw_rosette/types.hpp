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

#include "rosette/net/posix/socket.hpp"
#include "rosette/rtps/buf_pool.hpp"
#include "rosette/rtps/participant.hpp"
#include "rosette/rtps/stateful_reader.hpp"
#include "rosette/rtps/stateful_writer.hpp"
#include "rosette/rtps/transport.hpp"
#include "type_support.hpp"

#include <rmw/init.h>

#include <cstdint>
#include <new>

#ifndef RMW_ROSETTE_MAX_NODES
#  define RMW_ROSETTE_MAX_NODES 4
#endif

#ifndef RMW_ROSETTE_MAX_CONTEXTS
#  define RMW_ROSETTE_MAX_CONTEXTS 1
#endif

#ifndef RMW_ROSETTE_MAX_PENDING_CHANGES
/// Per-subscription buffer between StatefulReader::OnChangeAvailable
/// (fired from tick()/parse_received()) and rmw_take() (called from
/// application/executor context) — see PendingChanges below.
#  define RMW_ROSETTE_MAX_PENDING_CHANGES 4
#endif

namespace rmw_rosette {

/// Every publisher is held through this type-erased interface regardless
/// of which CacheDepth qos.hpp::select_writer_depth() chose for it.
using PublisherWriter = rtps::IStatefulWriter;

/// Backing store for one rmw_context_t. Owns the single RTPSParticipant
/// this process/firmware image presents to the ROS graph, plus the
/// transport it talks through.
///
/// RTPSParticipant and SocketUdpTransport both have non-default,
/// non-copyable constructors (they need domain/participant id and ports
/// known at construction time), so they can't live as plain default-
/// constructed members the way Pool<T,N> expects. They're placement-new'd
/// into aligned storage instead — the same technique
/// RTPSParticipant::create_writer<>() itself already uses for its writer
/// table (see participant.hpp) — so this stays a single static
/// allocation per context slot, constructed once at rmw_init() and torn
/// down once at rmw_context_fini(), not per-message.
struct Context {
    alignas(rtps::SocketUdpTransport) unsigned char transport_storage[sizeof(rtps::SocketUdpTransport)];
    alignas(rtps::RTPSParticipant) unsigned char participant_storage[sizeof(rtps::RTPSParticipant)];

    rosette::net::posix::PosixSocketProvider provider{};
    rtps::SocketUdpTransport* transport   = nullptr;  ///< placement-new'd into transport_storage
    rtps::RTPSParticipant*    participant = nullptr;  ///< placement-new'd into participant_storage
    uint8_t domain_id = 0;
    bool constructed = false;
};

#ifndef RMW_ROSETTE_MAX_NAME_LEN
#  define RMW_ROSETTE_MAX_NAME_LEN 64
#endif

/// Backing store for one rmw_node_t. RTPS has no first-class "node"
/// concept — this exists to satisfy rcl's node lifecycle and namespacing,
/// and to scope graph-cache queries (rmw_get_*_names_and_types) by node.
///
/// name/namespace are copied into fixed buffers rather than storing rcl's
/// pointer: rmw_create_node()'s caller makes no promise that its `name`/
/// `namespace_` arguments outlive the call, and the rmw_node_t this
/// backs must keep reporting them for its whole lifetime.
struct Node {
    Context* context = nullptr;
    char     name[RMW_ROSETTE_MAX_NAME_LEN] = {};
    char     namespace_[RMW_ROSETTE_MAX_NAME_LEN] = {};
};

/// Backing store for one rmw_publisher_t.
struct Publisher {
    Node*             node   = nullptr;
    PublisherWriter*  writer = nullptr;
    const message_type_support_callbacks_t* type_support = nullptr;
    char              mangled_topic_name[RTPS_MAX_TOPIC_NAME_LEN] = {};
};

/// One buffered CacheChange payload, copied out of the pool-backed
/// BufRef the moment StatefulReader::OnChangeAvailable fires (that
/// callback's CacheChange is only valid for the duration of the call —
/// see stateful_reader.hpp), and held here until rmw_take() drains it.
struct PendingChange {
    uint8_t  bytes[RTPS_BUF_BLOCK_SIZE] = {};
    uint32_t size = 0;
    bool     valid = false;
};

/// Backing store for one rmw_subscription_t. Bridges
/// StatefulReader::OnChangeAvailable (fired from tick()/parse_received())
/// to rmw_take() (called from application/executor context) via a small
/// fixed-capacity ring — no heap, bounded by
/// RMW_ROSETTE_MAX_PENDING_CHANGES regardless of publish rate; a
/// publisher that outruns rmw_take() drops the oldest unread change
/// rather than growing unbounded.
struct Subscription {
    Node*                node   = nullptr;
    rtps::StatefulReader* reader = nullptr;
    const message_type_support_callbacks_t* type_support = nullptr;
    char                 mangled_topic_name[RTPS_MAX_TOPIC_NAME_LEN] = {};

    PendingChange pending[RMW_ROSETTE_MAX_PENDING_CHANGES] = {};
    uint32_t      pending_head = 0;  ///< next slot to fill (oldest-overwrite on wrap)
    uint32_t      pending_count = 0;

    /**
     * @brief Copies one received change's payload into the ring, evicting
     * the oldest unread entry if the ring is full.
     *
     * Called from the `StatefulReader::OnChangeAvailable` trampoline
     * (rmw_subscription.cpp), which runs from `RTPSParticipant::tick()`/
     * `parse_received()` — a different call context than `pop_change()`,
     * which runs from whatever thread calls `rmw_take()`.
     *
     * Copies up to `sizeof(PendingChange::bytes)` bytes into the slot at
     * `pending_head`, then advances `pending_head` and grows
     * `pending_count` up to `RMW_ROSETTE_MAX_PENDING_CHANGES`. Once the
     * ring is full, advancing `pending_head` overwrites the oldest entry
     * without growing `pending_count` further, so a publisher that
     * outruns `rmw_take()` loses its oldest unread change rather than
     * growing the buffer.
     *
     * @param[in] data Payload bytes, as delivered in a `CacheChange::payload`.
     * @param[in] size Length of `data`; truncated to `sizeof(PendingChange::bytes)`.
     */
    void push_change(const uint8_t* data, uint32_t size) noexcept {
        PendingChange& slot = pending[pending_head];
        const uint32_t n = size < sizeof(slot.bytes) ? size : sizeof(slot.bytes);
        for (uint32_t i = 0; i < n; ++i) slot.bytes[i] = data[i];
        slot.size = n;
        slot.valid = true;
        pending_head = (pending_head + 1) % RMW_ROSETTE_MAX_PENDING_CHANGES;
        if (pending_count < RMW_ROSETTE_MAX_PENDING_CHANGES) ++pending_count;
    }

    /**
     * @brief Pops the oldest buffered change from the ring.
     *
     * Called from `rmw_take_with_info()` (rmw_subscription.cpp) to
     * retrieve the next change to deserialize.
     *
     * Computes the ring's tail index from `pending_head` and
     * `pending_count`, copies that slot into `out`, marks it invalid, and
     * decrements `pending_count`.
     *
     * @param[out] out Receives the popped change; left unmodified when
     *   the ring is empty.
     * @return `true` if a change was popped; `false` if the ring was empty.
     */
    bool pop_change(PendingChange& out) noexcept {
        if (pending_count == 0) return false;
        const uint32_t tail =
            (pending_head + RMW_ROSETTE_MAX_PENDING_CHANGES - pending_count) %
            RMW_ROSETTE_MAX_PENDING_CHANGES;
        out = pending[tail];
        pending[tail].valid = false;
        --pending_count;
        return true;
    }
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

/// Opaque per-context state rmw/init.h expects each vendor to define
/// (rmw/init.h only forward-declares `struct rmw_context_impl_s` as
/// `rmw_context_impl_t`). Defined here, not in rmw_init.cpp, because
/// every TU that reaches from an rmw_context_t into the participant
/// (rmw_node.cpp, rmw_wait_set.cpp, ...) needs the complete type, not
/// just the forward declaration.
struct rmw_context_impl_s {
    rmw_rosette::Context rosette_context;
};
