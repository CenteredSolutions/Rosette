/**
 * @file rmw_rosette/types.hpp
 * @brief Internal payload structs stored behind each rmw_*_t::data pointer.
 *
 * rmw handles (rmw_node_t, rmw_publisher_t, ...) are opaque to callers; the
 * real state lives in one of these structs, allocated by the corresponding
 * rmw_create_*() and freed by rmw_destroy_*(). None of this is exposed
 * outside rmw_rosette — callers only ever see the rmw_*_t wrapper.
 *
 * Writers of every history depth are one type
 * -------------------------------------------
 * rmw_qos_profile_t.depth is a runtime value, while a writer's history depth
 * is fixed when its storage is laid out (rtps::SharedPoolStatefulWriter<D>).
 * qos.hpp::create_writer_for_qos() picks D from a small closed set of depths;
 * every choice is an rtps::StatefulWriter, the one compiled writer class, so
 * publishers, service replies, and client requests all hold a plain
 * rtps::StatefulWriter* whatever depth it was created with.
 */

#pragma once

#include "rosette/net/socket.hpp"
#include "rosette/rtps/buf_pool.hpp"
#include "rosette/rtps/participant.hpp"
#include "rosette/rtps/participant_driver.hpp"
#include "rosette/rtps/stateful_reader.hpp"
#include "rosette/rtps/stateful_writer.hpp"
#include "rosette/rtps/transport.hpp"
#include "capacity.hpp"
#include "change_ring.hpp"
#include "message_codec.hpp"
#include "ros_discovery_info_writer.hpp"

#include <rmw/init.h>
#include <rmw/types.h>

#include <atomic>
#include <cstdint>
#include <new>


#ifndef RMW_ROSETTE_MAX_CONTEXTS
#  define RMW_ROSETTE_MAX_CONTEXTS 1
#endif


namespace rmw_rosette {

/**
 * @brief Backing store for one rmw_guard_condition_t.
 *
 * Guard conditions have no RTPS analogue; they are the wait-set signal rcl
 * uses to wake an executor from another thread (interrupts, SIGINT, graph
 * changes). `triggered` is atomic because rmw_trigger_guard_condition() may
 * run on any thread while rmw_wait() polls it on the executor's thread.
 * Copy assignment exists only so Pool<T,N> can reset slots.
 */
struct GuardCondition {
    std::atomic<bool> triggered{false};
    /// Context's driver wake event; signaled on trigger so a sleeping
    /// rmw_wait() returns at once instead of at its next timer.
    rosette::platform::Event* wake = nullptr;

    GuardCondition() = default;
    GuardCondition(const GuardCondition& other) noexcept
        : triggered(other.triggered.load()), wake(other.wake) {}
    GuardCondition& operator=(const GuardCondition& other) noexcept {
        triggered.store(other.triggered.load());
        wake = other.wake;
        return *this;
    }

    /// @brief Marks the condition ready and wakes the context's driver.
    /// Safe from any thread and from POSIX signal handlers.
    void trigger() noexcept {
        triggered.store(true);
        if (wake) wake->signal();
    }
};

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
    alignas(rtps::ParticipantDriver) unsigned char driver_storage[sizeof(rtps::ParticipantDriver)];

    /// Socket backend from the build's network port (network_port.hpp).
    rosette::net::SocketProvider* provider = nullptr;
    rtps::SocketUdpTransport* transport   = nullptr;  ///< placement-new'd into transport_storage
    rtps::RTPSParticipant*    participant = nullptr;  ///< placement-new'd into participant_storage
    /// Runs the participant and owns its lock; every rmw entry point that
    /// touches RTPS state or a ChangeRing holds driver->mutex().
    rtps::ParticipantDriver*  driver      = nullptr;  ///< placement-new'd into driver_storage
    /// Publishes this participant's nodes on `ros_discovery_info`; every
    /// node and endpoint creation and destruction reports to it.
    RosDiscoveryInfoWriter discovery_info;
    /// Triggered whenever the participant's view of the ROS graph changes
    /// (a peer, endpoint, or local node comes or goes). One per context,
    /// returned by rmw_node_get_graph_guard_condition() for every node, as
    /// rmw_dds_common::Context::graph_guard_condition is in stock rmw.
    GuardCondition        graph_guard;
    rmw_guard_condition_t graph_guard_handle{};
    uint8_t domain_id = 0;
    bool constructed = false;
    /// Receive buffer the driver drains sockets into; one full datagram.
    uint8_t rx_buffer[RMW_ROSETTE_MAX_DATAGRAM_SIZE] = {};
};

/// @brief Holds a context's participant lock for the enclosing scope.
class ContextLock {
public:
    explicit ContextLock(Context& ctx) noexcept : guard_(ctx.driver->mutex()) {}

private:
    rosette::platform::LockGuard guard_;
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
    rtps::StatefulWriter*  writer = nullptr;
    MessageCodec      codec;
    rmw_qos_profile_t qos{};  ///< Profile in effect, reported by rmw_publisher_get_actual_qos().
    char              mangled_topic_name[RTPS_MAX_TOPIC_NAME_LEN] = {};
};

/// Backing store for one rmw_subscription_t: an RTPS reader whose
/// changes are buffered in `received` until rmw_take() decodes them.
struct Subscription {
    Node*                 node   = nullptr;
    rtps::StatefulReader* reader = nullptr;
    MessageCodec          codec;
    rmw_qos_profile_t     qos{};  ///< Profile in effect, reported by rmw_subscription_get_actual_qos().
    char                  mangled_topic_name[RTPS_MAX_TOPIC_NAME_LEN] = {};
    ChangeRing            received;
};

/// Backing store for one rmw_service_t: a reader on the request topic and a
/// writer on the reply topic. Replies carry PID_RELATED_SAMPLE_IDENTITY =
/// {request writer GUID, request SN} so the client can correlate them.
struct Service {
    Node*                 node           = nullptr;
    rtps::StatefulReader* request_reader = nullptr;
    rtps::StatefulWriter*      reply_writer   = nullptr;
    MessageCodec          request_codec;
    MessageCodec          response_codec;
    rmw_qos_profile_t     qos{};  ///< Requested profile, applied to both endpoints.
    char                  service_name[RTPS_MAX_TOPIC_NAME_LEN] = {};
    ChangeRing            requests;
};

/// Backing store for one rmw_client_t: the mirror image of Service. Requests
/// carry PID_RELATED_SAMPLE_IDENTITY naming this client's reply reader, and
/// only replies correlated with this client's request writer are accepted.
struct Client {
    Node*                 node           = nullptr;
    rtps::StatefulWriter*      request_writer = nullptr;
    rtps::StatefulReader* reply_reader   = nullptr;
    MessageCodec          request_codec;
    MessageCodec          response_codec;
    rmw_qos_profile_t     qos{};  ///< Requested profile, applied to both endpoints.
    char                  service_name[RTPS_MAX_TOPIC_NAME_LEN] = {};
    ChangeRing            replies;
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
