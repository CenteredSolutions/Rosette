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
 * is fixed when its storage is laid out (rosette::rtps::SharedPoolStatefulWriter<D>).
 * qos.hpp::create_writer_for_qos() picks D from a small closed set of depths;
 * every choice is an rosette::rtps::StatefulWriter, the one compiled writer class, so
 * publishers, service replies, and client requests all hold a plain
 * rosette::rtps::StatefulWriter* whatever depth it was created with.
 */

#pragma once

#include "capacity.hpp"
#include "change_ring.hpp"
#include "message_codec.hpp"
#include "participant_pump.hpp"
#include "ros_discovery_info_writer.hpp"
#include "rosette/net/socket.hpp"
#include "rosette/rtps/buf_pool.hpp"
#include "rosette/rtps/participant.hpp"
#include "rosette/rtps/participant_driver.hpp"
#include "rosette/rtps/stateful_reader.hpp"
#include "rosette/rtps/stateful_writer.hpp"
#include "rosette/rtps/transport.hpp"

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
    alignas(rosette::rtps::SocketUdpTransport) unsigned char transport_storage[sizeof(
        rosette::rtps::SocketUdpTransport)];
    alignas(rosette::rtps::RTPSParticipant) unsigned char participant_storage[sizeof(
        rosette::rtps::RTPSParticipant)];
    alignas(rosette::rtps::ParticipantDriver) unsigned char driver_storage[sizeof(
        rosette::rtps::ParticipantDriver)];

    /// Socket backend from the build's network port (network_port.hpp).
    rosette::net::SocketProvider* provider = nullptr;
    rosette::rtps::SocketUdpTransport* transport =
        nullptr;  ///< placement-new'd into transport_storage
    rosette::rtps::RTPSParticipant* participant =
        nullptr;  ///< placement-new'd into participant_storage
    /// Runs the participant and owns its lock; every rmw entry point that
    /// touches RTPS state or a ChangeRing holds driver->mutex().
    rosette::rtps::ParticipantDriver* driver = nullptr;  ///< placement-new'd into driver_storage
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
    /// Set by rmw_shutdown(): no entity may be created in the context any
    /// more, and rmw_context_fini() is allowed. Atomic because the creation
    /// checks read it before taking the participant lock.
    std::atomic<bool>                 shut_down{false};
    /// Receive buffer the driver drains sockets into; one full datagram.
    uint8_t rx_buffer[RMW_ROSETTE_MAX_DATAGRAM_SIZE] = {};
#if RMW_ROSETTE_PUMP_THREAD
    /// Runs `driver` from rmw_init() to rmw_context_fini() (participant_pump.hpp).
    ParticipantPump pump;
#endif
};

/// @brief Holds a context's participant lock for the enclosing scope.
class ContextLock {
public:
    explicit ContextLock(Context& ctx) noexcept : guard_(ctx.driver->mutex()) {}

private:
    rosette::platform::LockGuard guard_;
};

/**
 * @brief Holds a context's participant lock for a scope that queues protocol
 *   work, and wakes the driver as the scope ends.
 *
 * Creating or destroying an entity queues discovery announcements, and
 * publishing, requesting or replying queues a sample; a protocol pass sends
 * them. Whoever runs the passes (the host pump thread, or another task's
 * rmw_wait() on a board) may be asleep until a later timer, so every entry
 * point that queues work takes this lock instead of ContextLock, and the pass
 * that sends the work follows at once.
 *
 * Verification:
 * - Correct means: a sample published while no thread is in rmw_wait() is
 *   sent within one pass, not at the participant's next timer.
 * - Covered by: rmw_rosette/test/test_participant_pump.cpp (TC-RMW-PUMP-001).
 */
class ContextUpdateLock {
public:
    explicit ContextUpdateLock(Context& ctx) noexcept
        : driver_(*ctx.driver), guard_(ctx.driver->mutex()) {}

    ~ContextUpdateLock() { driver_.wake(); }

    ContextUpdateLock(const ContextUpdateLock&) = delete;
    ContextUpdateLock& operator=(const ContextUpdateLock&) = delete;

private:
    rosette::rtps::ParticipantDriver& driver_;
    rosette::platform::LockGuard      guard_;
};

/**
 * @brief Blocks until `ready()` holds or `timeout_ms` passes, while the
 *   context's participant runs.
 *
 * The one wait rmw_wait() and rmw_publisher_wait_for_all_acked() share. With
 * the pump thread running (a host, unless RMW_ROSETTE_PUMP_THREAD=0) this
 * waits alongside it (ParticipantPump::wait_until()); otherwise, as on a
 * board, the caller runs the passes itself (ParticipantDriver::spin_until()).
 * Either way `ready` is evaluated under the participant lock, at once and
 * after every pass.
 *
 * @param[in] ctx        The context whose participant decides readiness.
 * @param[in] ready      Predicate; must not block, and may run on another thread.
 * @param[in] timeout_ms Longest wait; 0 checks once,
 *   rosette::platform::kWaitForever waits until `ready()` holds.
 * @return true when `ready()` held, false on timeout.
 *
 * Verification:
 * - Correct means: as ParticipantDriver::spin_until(), whichever thread runs the passes.
 * - Covered by: rmw_rosette/test/test_participant_pump.cpp,
 *   test_guard_condition.cpp, test_pub_sub_loopback.cpp.
 */
template <typename Ready>
bool wait_until_ready(Context& ctx, Ready ready, uint32_t timeout_ms) noexcept {
#if RMW_ROSETTE_PUMP_THREAD
    if (ctx.pump.running()) {
        return ctx.pump.wait_until(
            [](void* argument) noexcept { return (*static_cast<Ready*>(argument))(); }, &ready,
            timeout_ms);
    }
#endif

    return ctx.driver->spin_until(ready, timeout_ms);
}

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
    rosette::rtps::StatefulWriter* writer = nullptr;
    MessageCodec      codec;
    rmw_qos_profile_t qos{};  ///< Profile in effect, reported by rmw_publisher_get_actual_qos().
    char              mangled_topic_name[RTPS_MAX_TOPIC_NAME_LEN] = {};
};

/// Backing store for one rmw_subscription_t: an RTPS reader whose
/// changes are buffered in `received` until rmw_take() decodes them.
struct Subscription {
    Node*                 node   = nullptr;
    rosette::rtps::StatefulReader* reader = nullptr;
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
    rosette::rtps::StatefulReader* request_reader = nullptr;
    rosette::rtps::StatefulWriter* reply_writer = nullptr;
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
    rosette::rtps::StatefulWriter* request_writer = nullptr;
    rosette::rtps::StatefulReader* reply_reader = nullptr;
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
