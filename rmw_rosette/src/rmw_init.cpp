/**
 * @file rmw_init.cpp
 * @brief rmw_init/shutdown/context_fini — owns the one RTPSParticipant.
 */

#include "rmw_rosette/identifier.h"
#include "rmw_rosette/network_port.hpp"
#include "rmw_rosette/pool.hpp"
#include "rmw_rosette/ros_features.h"
#include "rmw_rosette/types.hpp"

#include <rcutils/strdup.h>
#include <rmw/error_handling.h>
#include <rmw/init.h>
#include <rmw/init_options.h>
#include <rmw/rmw.h>
#include <rmw/security_options.h>
#if RMW_ROSETTE_HAS_DISCOVERY_OPTIONS
#  include <rmw/discovery_options.h>
#endif
#include <cstdio>
#include <cstdlib>

namespace rtps = rosette::rtps;

// rmw_context_impl_s is defined in types.hpp, not here: rmw_node.cpp and
// rmw_wait_set.cpp also dereference context->impl->rosette_context, and
// rmw/init.h only forward-declares the type (`typedef struct
// rmw_context_impl_s rmw_context_impl_t`) — every TU that touches it
// needs the complete definition, not just this one.

namespace {

/**
 * @brief Computes the standard RTPS unicast port for SPDP/SEDP metatraffic.
 *
 * rmw_init() passes this to `SocketUdpTransport`'s constructor so the
 * transport binds the same port a real DDS implementation on the same
 * domain/participant slot would, letting this participant interoperate
 * with unmodified ROS 2 nodes.
 *
 * Implements the formula PB + DG * domainId + d1 + 2 * participantId
 * with PB = 7400, DG = 250, d1 = 10, duplicated here rather than exposed
 * from the equivalent helper in src/rtps/participant.cpp (which is
 * intentionally file-local — RTPSParticipant computes its own fallback
 * internally when the transport doesn't report a port).
 *
 * @param[in] domain_id ROS domain ID.
 * @param[in] participant_id Local participant slot on this domain/host.
 * @return The unicast port for SPDP/SEDP metatraffic.
 *
 * References:
 * - OMG DDSI-RTPS 2.5, §9.6.1.1.
 */
uint16_t metatraffic_unicast_port(uint8_t domain_id, uint8_t participant_id) noexcept {
    return static_cast<uint16_t>(7400u + 250u * domain_id + 10u + 2u * participant_id);
}

/**
 * @brief Computes the standard RTPS unicast port for user data.
 *
 * Same role as metatraffic_unicast_port() for the participant's default
 * (non-metatraffic) unicast locator.
 *
 * Implements PB + DG * domainId + d3 + 2 * participantId with d3 = 11.
 *
 * @param[in] domain_id ROS domain ID.
 * @param[in] participant_id Local participant slot on this domain/host.
 * @return The unicast port for user data.
 *
 * References:
 * - OMG DDSI-RTPS 2.5, §9.6.1.1.
 */
uint16_t default_unicast_port(uint8_t domain_id, uint8_t participant_id) noexcept {
    return static_cast<uint16_t>(7400u + 250u * domain_id + 11u + 2u * participant_id);
}

/// 239.255.0.1 in host order: the well-known SPDP discovery multicast
/// group (OMG DDSI-RTPS 2.5 §9.6.1.4.1).
constexpr uint32_t kSpdpMulticastAddr = 0xEFFF0001u;

/// Highest participant ID tried when looking for free unicast ports. RTPS
/// ports for one domain span PB + DG*d .. PB + DG*(d+1), and participant
/// IDs above 119 would spill into the next domain's range.
constexpr uint8_t kMaxParticipantId = 119u;

/// Highest ROS domain ID: domain 232's SPDP port (7400 + 250 * 232 = 65400)
/// is the last whose unicast ports still fit in 16 bits.
constexpr size_t kMaxDomainId = 232u;

/**
 * @brief The domain rmw_init() joins for the domain ID in its options.
 *
 * rcl_init() replaces RMW_DEFAULT_DOMAIN_ID with ROS_DOMAIN_ID before
 * rmw_init() sees it; a caller that skips rcl can still pass it, and that
 * means domain 0, the ROS 2 default.
 *
 * @param[in]  requested  The options' domain_id.
 * @param[out] domain_id  The domain to join; written only on success.
 * @return true when the domain is valid; false above kMaxDomainId.
 *
 * Verification:
 * - Correct means: RMW_DEFAULT_DOMAIN_ID gives 0, 0..232 pass through
 *   unchanged, and 233 or more is refused instead of wrapping the port
 *   arithmetic into another domain's ports.
 * - Covered by: rmw_rosette/test/test_init_options.cpp (TC-RMW-INITOPT-010); the
 *   default mapping by tests/ros2 loopback runs, which bind domain 0 without ROS_DOMAIN_ID.
 *
 * References:
 * - rmw/domain_id.h (Humble), RMW_DEFAULT_DOMAIN_ID
 * - ROS 2 documentation, "The ROS_DOMAIN_ID": domain IDs 0 to 232
 * - OMG DDSI-RTPS 2.5 §9.6.1.1 (port mapping)
 */
bool resolve_domain_id(size_t requested, uint8_t& domain_id) noexcept {
    if (requested == RMW_DEFAULT_DOMAIN_ID) {
        domain_id = 0u;
        return true;
    }

    if (requested > kMaxDomainId) {
        return false;
    }

    domain_id = static_cast<uint8_t>(requested);
    return true;
}

/**
 * @brief Computes the SPDP multicast port shared by a domain's participants.
 *
 * Implements PB + DG * domainId + d0 with PB = 7400, DG = 250, d0 = 0.
 *
 * @param[in] domain_id ROS domain ID.
 * @return The SPDP multicast port.
 *
 * References:
 * - OMG DDSI-RTPS 2.5, §9.6.1.1.
 */
uint16_t spdp_multicast_port(uint8_t domain_id) noexcept {
    return static_cast<uint16_t>(7400u + 250u * domain_id);
}

/// Process-wide pool backing every `rmw_context_impl_s` this rmw
/// implementation creates.
rmw_rosette::Pool<rmw_context_impl_s, RMW_ROSETTE_MAX_CONTEXTS>& context_pool() {
    static rmw_rosette::Pool<rmw_context_impl_s, RMW_ROSETTE_MAX_CONTEXTS> pool;
    return pool;
}

/**
 * @brief Opens a transport on the first participant ID whose ports are free.
 *
 * DDSI-RTPS derives each participant's unicast ports from a participant ID
 * that must be unique per host and domain. Stock DDS implementations pick
 * it the same way: try IDs from 0 upward until binding succeeds. The SPDP
 * multicast port is shared and never the reason an ID is rejected.
 *
 * Placement-news a SocketUdpTransport into `ctx.transport_storage` for each
 * candidate ID, keeps the first one that opens, and destroys the others.
 *
 * @param[in,out] ctx            Context whose transport storage is used.
 * @param[out]    participant_id Receives the chosen ID on success.
 * @return `true` with `ctx.transport` open; `false` when every ID up to
 *   kMaxParticipantId whose ports fit in 16 bits is taken, or the SPDP
 *   port cannot be opened.
 *
 * Verification:
 * - Correct means: several rmw_rosette and stock-RMW processes in one domain
 *   on one host each end up with distinct unicast ports.
 *
 * References:
 * - OMG DDSI-RTPS 2.5 §9.6.1.1 (participantId in the port mapping)
 */
bool open_transport_on_free_ports(rmw_rosette::Context& ctx, uint8_t& participant_id) noexcept {
    for (uint8_t id = 0; id <= kMaxParticipantId; ++id) {
        // In the highest domains the last participant IDs' ports pass 65535.
        if (7400u + 250u * ctx.domain_id + 11u + 2u * id > 65535u) {
            break;
        }

        ctx.transport = new (ctx.transport_storage)
            rtps::SocketUdpTransport(*ctx.provider, metatraffic_unicast_port(ctx.domain_id, id),
                                     default_unicast_port(ctx.domain_id, id), kSpdpMulticastAddr,
                                     spdp_multicast_port(ctx.domain_id));
        if (ctx.transport->open()) {
            participant_id = id;
            return true;
        }
        ctx.transport->~SocketUdpTransport();
        ctx.transport = nullptr;
    }
    return false;
}

/**
 * @brief The datagram limit for this context.
 *
 * RMW_ROSETTE_MAX_DATAGRAM_SIZE from the build, lowered by the environment
 * variable of the same name when set to a smaller valid value (at least 512).
 *
 * @return Largest UDP datagram to send, in octets.
 */
uint32_t effective_max_datagram_size() noexcept {
    uint32_t limit = RMW_ROSETTE_MAX_DATAGRAM_SIZE;
    if (const char* env = std::getenv("RMW_ROSETTE_MAX_DATAGRAM_SIZE")) {
        const unsigned long requested = std::strtoul(env, nullptr, 10);
        if (requested >= 512u && requested < limit) limit = static_cast<uint32_t>(requested);
    }
    return limit;
}

/**
 * @brief Prints the transport's datagram counters when requested.
 *
 * With RMW_ROSETTE_TRANSPORT_STATS set in the environment, one line goes to
 * stderr as the context is finalized. tests/ros2/run_rmw_frag_loopback.sh
 * parses it to prove that no datagram exceeded the configured limit in
 * either direction and that none was truncated.
 *
 * @param[in] transport Transport being torn down.
 */
void report_transport_stats(const rtps::SocketUdpTransport& transport) noexcept {
    if (std::getenv("RMW_ROSETTE_TRANSPORT_STATS") == nullptr) return;
    const rtps::TransportStats& st = transport.stats();
    std::fprintf(stderr,
                 "rmw_rosette transport: limit=%u tx=%u rx=%u max_tx=%u max_rx=%u "
                 "oversize_tx=%u oversize_rx=%u truncated_rx=%u dropped_rx=%u\n",
                 static_cast<unsigned>(transport.max_datagram_size()), st.tx_datagrams,
                 st.rx_datagrams, st.max_tx_datagram, st.max_rx_datagram, st.oversize_tx,
                 st.oversize_rx, st.truncated_rx, st.dropped_rx);
}

/**
 * @brief Prints the session's count of writer changes lost to a full history, when any were.
 *
 * Each counted eviction already logged a warning as it happened (rate
 * limited, StatefulWriter::evicted_unacknowledged()); this line gives the
 * session's total as the context is finalized. On a node with services it
 * is the number of replies lost to a reply history shallower than the
 * replies in flight, the evidence for raising RMW_ROSETTE_REPLY_DEPTH.
 *
 * @param[in] participant Participant being torn down.
 *
 * Verification:
 * - Correct means: one line on stderr exactly when the participant's
 *   total is above zero.
 * - Covered by: TC-PART-8.5.1-014 (the total); the line itself by inspection.
 */
void report_writer_evictions(const rtps::RTPSParticipant& participant) noexcept {
    const uint32_t evicted = participant.evicted_unacknowledged();
    if (evicted == 0u) {
        return;
    }

    std::fprintf(stderr,
                 "rmw_rosette: %u changes evicted before their readers had them "
                 "(on a service, each is a lost reply; RMW_ROSETTE_REPLY_DEPTH=%u)\n",
                 static_cast<unsigned>(evicted), static_cast<unsigned>(RMW_ROSETTE_REPLY_DEPTH));
}

/**
 * @brief Destroys a context's driver, participant, and transport, in that order.
 *
 * rmw_context_fini() and rmw_init()'s failure path share this one teardown,
 * so the participant's destructor, which is large on firmware, is emitted once.
 *
 * @param[in,out] ctx Context whose objects were constructed by rmw_init();
 *   `constructed` is cleared on return.
 */
void destroy_participant(rmw_rosette::Context& ctx) noexcept {
    ctx.driver->~ParticipantDriver();
    ctx.participant->~RTPSParticipant();
    ctx.transport->~SocketUdpTransport();
    ctx.constructed = false;
}

/**
 * @brief Triggers a context's graph guard condition; the participant's graph observer.
 *
 * Runs on the thread driving the participant, under the context lock;
 * GuardCondition::trigger() is safe there.
 *
 * @param[in] graph_guard The context's rmw_rosette::GuardCondition.
 */
void trigger_graph_guard(void* graph_guard) noexcept {
    static_cast<rmw_rosette::GuardCondition*>(graph_guard)->trigger();
}

#if RMW_ROSETTE_HAS_DISCOVERY_OPTIONS
/**
 * @brief Checks that the requested discovery scope is one this participant provides.
 *
 * rcl fills `discovery_options` from ROS_AUTOMATIC_DISCOVERY_RANGE and
 * ROS_STATIC_PEERS (default: SUBNET, no peers). The participant discovers over
 * SPDP multicast and the unicast locators of the network port, which is the
 * SUBNET range, and is also what SYSTEM_DEFAULT names for this middleware. A
 * narrower range (LOCALHOST, OFF) or a static peer list would need the
 * transport to restrict or extend where SPDP is sent; until it does, those
 * requests are refused, so a node never discovers wider than it was asked to.
 *
 * @param[in] options Discovery options from rmw_init()'s init options.
 * @return `RMW_RET_OK` for SUBNET or SYSTEM_DEFAULT without static peers;
 *   `RMW_RET_INVALID_ARGUMENT` for NOT_SET, which rmw/discovery_options.h says
 *   an implementation must reject; `RMW_RET_UNSUPPORTED` for LOCALHOST, OFF,
 *   or any static peer. The rmw error message names the refused setting.
 *
 * Verification:
 * - Correct means: rmw_init() succeeds under rcl's defaults and fails, with
 *   the context untouched, for each refused setting.
 * - Covered by: rmw_rosette/test/test_init_options.cpp (TC-RMW-INITOPT-011).
 *
 * References:
 * - rmw/discovery_options.h (Jazzy), `rmw_automatic_discovery_range_t`
 * - rcl/src/rcl/init.c (Jazzy), `rcl_init` (environment to discovery options)
 */
rmw_ret_t check_discovery_options(const rmw_discovery_options_t& options) noexcept {
    switch (options.automatic_discovery_range) {
        case RMW_AUTOMATIC_DISCOVERY_RANGE_SUBNET:
        case RMW_AUTOMATIC_DISCOVERY_RANGE_SYSTEM_DEFAULT:
            break;
        case RMW_AUTOMATIC_DISCOVERY_RANGE_NOT_SET:
            RMW_SET_ERROR_MSG("rmw_rosette: rmw_init: automatic_discovery_range is not set");
            return RMW_RET_INVALID_ARGUMENT;
        default:
            RMW_SET_ERROR_MSG(
                "rmw_rosette: rmw_init: only the SUBNET and SYSTEM_DEFAULT "
                "discovery ranges are supported");
            return RMW_RET_UNSUPPORTED;
    }

    if (options.static_peers_count != 0u) {
        RMW_SET_ERROR_MSG("rmw_rosette: rmw_init: static discovery peers are not supported");
        return RMW_RET_UNSUPPORTED;
    }

    return RMW_RET_OK;
}
#endif

}  // namespace

extern "C" {

const char * const rmw_rosette_identifier = RMW_ROSETTE_ID;

const char* rmw_get_implementation_identifier(void) {
    return rmw_rosette_identifier;
}

const char* rmw_get_serialization_format(void) {
    // XCDR1, byte-identical to every other DDS-backed rmw (message_codec.hpp).
    return "cdr";
}

/**
 * @brief Fills an rmw_init_options_t with rmw_rosette's default values.
 *
 * rcl calls this before rmw_init(), then may override fields before
 * passing the options on. `domain_id` starts as RMW_DEFAULT_DOMAIN_ID, the
 * value rcl_init() replaces with ROS_DOMAIN_ID (or 0 when unset); any other
 * value would make rcl skip that lookup, so every rcl, rclc and rclcpp
 * program would ignore ROS_DOMAIN_ID. Where the headers have discovery
 * options, they are left zero (range NOT_SET, no peers), as in the stock
 * rmws; rcl_init() sets the range from ROS_AUTOMATIC_DISCOVERY_RANGE.
 *
 * @param[out] init_options Options struct to fill.
 * @param[in] allocator Allocator to record on the options; unused beyond
 *   that, since nothing here allocates.
 * @return `RMW_RET_OK`, or `RMW_RET_INVALID_ARGUMENT` if `init_options`
 *   is null.
 *
 * Verification:
 * - Correct means: `domain_id` is RMW_DEFAULT_DOMAIN_ID, so a program
 *   started with ROS_DOMAIN_ID=65 binds domain 65's ports (7400 + 250 * 65).
 * - Covered by: rmw_rosette/test/test_init_options.cpp (TC-RMW-INITOPT-009).
 *
 * References:
 * - rmw/init_options.h, rmw/domain_id.h (Humble), RMW_DEFAULT_DOMAIN_ID
 * - rcl/src/rcl/init.c (Humble), rcl_init(): the ROS_DOMAIN_ID lookup
 */
rmw_ret_t rmw_init_options_init(rmw_init_options_t* init_options, rcutils_allocator_t allocator) {
    if (!init_options) return RMW_RET_INVALID_ARGUMENT;
    init_options->instance_id = 0;
    init_options->implementation_identifier = rmw_rosette_identifier;
    init_options->allocator = allocator;
    init_options->impl = nullptr;
    init_options->enclave = nullptr;
    init_options->domain_id = RMW_DEFAULT_DOMAIN_ID;
    init_options->security_options = rmw_get_zero_initialized_security_options();
#if RMW_ROSETTE_HAS_DISCOVERY_OPTIONS
    init_options->discovery_options = rmw_get_zero_initialized_discovery_options();
#endif
    return RMW_RET_OK;
}

/**
 * @brief Deep-copies an rmw_init_options_t, including the memory the options own.
 *
 * The options own two heap strings, `enclave` and
 * `security_options.security_root_path`, and, where the headers have
 * discovery options, the `discovery_options.static_peers` array. rcl_init()
 * fills `enclave` with rcutils_strdup() and the peers from ROS_STATIC_PEERS in
 * the context's own copy of the options, and rcl_context_fini() finalizes that
 * copy through rmw_init_options_fini(), so the copy duplicates each of them
 * with `src`'s allocator and rmw_init_options_fini() frees them; a shallow
 * copy would leave two owners of one allocation.
 *
 * @param[in] src Options to copy from; initialized by rmw_init_options_init().
 * @param[out] dst Options to copy into; must be zero-initialized
 *   (rmw_get_zero_initialized_init_options()). Untouched on failure.
 * @return `RMW_RET_OK`; `RMW_RET_INVALID_ARGUMENT` if either argument is
 *   null, `dst` is already initialized or `src`'s allocator is invalid;
 *   `RMW_RET_INCORRECT_RMW_IMPLEMENTATION` if `src` is another rmw's;
 *   `RMW_RET_BAD_ALLOC` if a string or the peer list cannot be duplicated.
 *
 * Verification:
 * - Correct means: `dst` equals `src` field for field except that its
 *   strings are its own copies, and finalizing both returns every byte
 *   they allocated.
 * - Covered by: rmw_rosette/test/test_init_options.cpp
 *   (TC-RMW-INITOPT-002, -004, -007, -008, -012).
 *
 * References:
 * - rmw/init_options.h (Humble, Jazzy), `rmw_init_options_copy`, `rmw_init_options_fini`
 * - rmw/security_options.h (Humble), `rmw_security_options_copy`
 * - rmw/discovery_options.h (Jazzy), `rmw_discovery_options_copy`
 * - rcl/src/rcl/init.c (Humble), `rcl_init` (the enclave rcutils_strdup)
 */
rmw_ret_t rmw_init_options_copy(const rmw_init_options_t* src, rmw_init_options_t* dst) {
    if (!src || !dst) {
        RMW_SET_ERROR_MSG("rmw_rosette: rmw_init_options_copy: null argument");
        return RMW_RET_INVALID_ARGUMENT;
    }

    if (src->implementation_identifier != rmw_rosette_identifier) {
        RMW_SET_ERROR_MSG("rmw_rosette: rmw_init_options_copy: options of another rmw");
        return RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
    }

    if (dst->implementation_identifier != nullptr) {
        RMW_SET_ERROR_MSG("rmw_rosette: rmw_init_options_copy: destination already initialized");
        return RMW_RET_INVALID_ARGUMENT;
    }

    const rcutils_allocator_t* allocator = &src->allocator;
    if (!rcutils_allocator_is_valid(allocator)) {
        RMW_SET_ERROR_MSG("rmw_rosette: rmw_init_options_copy: invalid allocator");
        return RMW_RET_INVALID_ARGUMENT;
    }

    rmw_init_options_t copy = *src;
    copy.enclave = nullptr;
    copy.security_options = rmw_get_zero_initialized_security_options();
    if (src->enclave != nullptr) {
        copy.enclave = rcutils_strdup(src->enclave, *allocator);
        if (copy.enclave == nullptr) {
            RMW_SET_ERROR_MSG("rmw_rosette: rmw_init_options_copy: cannot copy the enclave");
            return RMW_RET_BAD_ALLOC;
        }
    }

    if (rmw_security_options_copy(&src->security_options, allocator, &copy.security_options)
        != RMW_RET_OK) {
        allocator->deallocate(copy.enclave, allocator->state);
        return RMW_RET_BAD_ALLOC;
    }

#if RMW_ROSETTE_HAS_DISCOVERY_OPTIONS
    // The peer list records its allocator by value; the copy takes a mutable one.
    rcutils_allocator_t peers_allocator = *allocator;
    copy.discovery_options = rmw_get_zero_initialized_discovery_options();
    if (rmw_discovery_options_copy(&src->discovery_options, &peers_allocator,
                                   &copy.discovery_options)
        != RMW_RET_OK) {
        allocator->deallocate(copy.enclave, allocator->state);
        (void)rmw_security_options_fini(&copy.security_options, allocator);
        return RMW_RET_BAD_ALLOC;
    }
#endif

    *dst = copy;
    return RMW_RET_OK;
}

/**
 * @brief Releases an rmw_init_options_t and the memory it owns.
 *
 * Frees `enclave` and `security_options.security_root_path` with the
 * options' allocator, and the discovery peer list with the allocator recorded
 * beside it (see rmw_init_options_copy() for who fills them),
 * then resets the options. Options that were never initialized, or were
 * already finalized, carry no implementation identifier, own nothing and
 * are only reset. Options stamped by another rmw are rejected before any
 * of their fields is read, because their layout and ownership rules are
 * not this rmw's.
 *
 * @param[in,out] init_options Options to finalize; reset to the
 *   zero-initialized state on return, and untouched on failure.
 * @return `RMW_RET_OK`; `RMW_RET_INVALID_ARGUMENT` if `init_options` is
 *   null, or owns a string but has no valid allocator to free it with;
 *   `RMW_RET_INCORRECT_RMW_IMPLEMENTATION` if the options are another
 *   rmw's.
 *
 * Verification:
 * - Correct means: after rcl_init() and rcl_shutdown() / rcl_context_fini()
 *   the heap is back where it was; on the RP2040 that is the session
 *   restart check, `rcl_heap` equal from one session to the next. Foreign
 *   options come back with every field and allocation unchanged.
 * - Covered by: rmw_rosette/test/test_init_options.cpp
 *   (TC-RMW-INITOPT-001, -003, -005, -006).
 *
 * References:
 * - rmw/init_options.h (Humble, Jazzy), `rmw_init_options_fini`
 * - rmw/security_options.h (Humble), `rmw_security_options_fini`
 * - rmw/discovery_options.h (Jazzy), `rmw_discovery_options_fini`
 */
rmw_ret_t rmw_init_options_fini(rmw_init_options_t* init_options) {
    if (!init_options) {
        RMW_SET_ERROR_MSG("rmw_rosette: rmw_init_options_fini: null argument");
        return RMW_RET_INVALID_ARGUMENT;
    }

    if (init_options->implementation_identifier != nullptr
        && init_options->implementation_identifier != rmw_rosette_identifier) {
        RMW_SET_ERROR_MSG("rmw_rosette: rmw_init_options_fini: options of another rmw");
        return RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
    }

    const bool owns_strings = init_options->enclave != nullptr
                              || init_options->security_options.security_root_path != nullptr;
    const rcutils_allocator_t* allocator = &init_options->allocator;
    if (owns_strings) {
        if (!rcutils_allocator_is_valid(allocator)) {
            RMW_SET_ERROR_MSG("rmw_rosette: rmw_init_options_fini: invalid allocator");
            return RMW_RET_INVALID_ARGUMENT;
        }

        allocator->deallocate(init_options->enclave, allocator->state);
        (void)rmw_security_options_fini(&init_options->security_options, allocator);
    }

#if RMW_ROSETTE_HAS_DISCOVERY_OPTIONS
    // The peer list records its own allocator, so it is released whether or
    // not the options own a string.
    if (init_options->discovery_options.static_peers != nullptr
        && rmw_discovery_options_fini(&init_options->discovery_options) != RMW_RET_OK) {
        return RMW_RET_INVALID_ARGUMENT;
    }
#endif

    *init_options = rmw_get_zero_initialized_init_options();
    return RMW_RET_OK;
}

/**
 * @brief Creates the RTPS participant and UDP transport backing one rmw context.
 *
 * rcl calls this once per process (or once per context it manages) to
 * bring rmw_rosette's RTPS participant onto the ROS graph before any
 * node/publisher/subscription can be created against it.
 *
 * Draws a context slot from context_pool(), opens a `SocketUdpTransport`
 * over the build's socket backend (network_socket_provider()) on the first
 * free participant ID (open_transport_on_free_ports()), then placement-news
 * the `RTPSParticipant` that borrows it. The participant advertises the
 * network port's IPv4 address (network_ipv4_address()) rather than loopback:
 * stock DDS implementations ignore loopback locators from participants they
 * cannot prove are local. The participant's first user writer is the
 * `ros_discovery_info` writer (ros_discovery_info_writer.hpp), which every
 * node created on this context reports to. The context's graph guard
 * condition is triggered by every graph change the participant reports.
 *
 * @param[in] options Init options; `domain_id` and `instance_id` are used,
 *   and `discovery_options` where the headers have them
 *   (check_discovery_options()). `domain_id` is 0 to 232, or
 *   RMW_DEFAULT_DOMAIN_ID for domain 0.
 * @param[out] context Context to initialize.
 * @return `RMW_RET_OK` on success; `RMW_RET_INVALID_ARGUMENT` if either
 *   argument is null, no discovery range is set, or `domain_id` is above 232;
 *   `RMW_RET_UNSUPPORTED` for a discovery range or static peers this
 *   participant cannot provide; `RMW_RET_ERROR` if the context pool is exhausted,
 *   no participant ID with free ports was found, or the participant cannot
 *   hold the `ros_discovery_info` writer.
 *
 * Verification:
 * - Correct means: a stock RMW in the same domain discovers this
 *   participant and matches its endpoints.
 * - Covered by: tests/ros2/run_rmw_interop_loopback.sh.
 *
 * References:
 * - rmw/init.h (Humble, Jazzy), `rmw_init`
 * - OMG DDSI-RTPS 2.5 §8.5.3 (SPDP), §9.6.1.1 (port mapping)
 */
rmw_ret_t rmw_init(const rmw_init_options_t* options, rmw_context_t* context) {
    if (!options || !context) {
        return RMW_RET_INVALID_ARGUMENT;
    }

#if RMW_ROSETTE_HAS_DISCOVERY_OPTIONS
    const rmw_ret_t discovery_ret = check_discovery_options(options->discovery_options);
    if (discovery_ret != RMW_RET_OK) {
        return discovery_ret;
    }
#endif

    uint8_t domain_id = 0u;
    if (!resolve_domain_id(options->domain_id, domain_id)) {
        RMW_SET_ERROR_MSG("rmw_rosette: rmw_init: domain_id above 232");
        return RMW_RET_INVALID_ARGUMENT;
    }

    rmw_context_impl_s* impl = context_pool().acquire();
    if (!impl) {
        RMW_SET_ERROR_MSG("rmw_rosette: rmw_init: context pool exhausted "
                          "(RMW_ROSETTE_MAX_CONTEXTS)");
        return RMW_RET_ERROR;
    }

    rmw_rosette::Context& ctx = impl->rosette_context;
    ctx.domain_id = domain_id;
    ctx.provider = &rmw_rosette::network_socket_provider();
    uint8_t participant_id = 0;
    if (!open_transport_on_free_ports(ctx, participant_id)) {
        RMW_SET_ERROR_MSG("rmw_rosette: rmw_init: no free RTPS participant ports "
                          "in this domain");
        context_pool().release(impl);
        return RMW_RET_ERROR;
    }

    ctx.participant = new (ctx.participant_storage)
        rtps::RTPSParticipant(ctx.domain_id, participant_id, *ctx.transport);

    // Writers fragment against this limit; peers must respect it too. The
    // environment may lower it (never raise it) for a link with a smaller
    // MTU, e.g. to rehearse a WIZnet link's 1472 on a host.
    ctx.transport->set_max_datagram_size(effective_max_datagram_size());
    // RMW_ROSETTE_STRICT_LINK rehearses such a link's losses on a host: every
    // received datagram above the limit is discarded, as the wire would.
    ctx.transport->set_strict_link(std::getenv("RMW_ROSETTE_STRICT_LINK") != nullptr);
    ctx.participant->add_ipv4_address(rmw_rosette::network_ipv4_address(),
                                      rmw_rosette::network_ipv4_prefix_len());
    ctx.driver = new (ctx.driver_storage) rtps::ParticipantDriver(
        *ctx.participant, *ctx.transport, *ctx.provider, ctx.rx_buffer, sizeof(ctx.rx_buffer));
    ctx.constructed = true;
    ctx.graph_guard.wake = &ctx.driver->wake_event();
    ctx.graph_guard_handle.implementation_identifier = rmw_rosette_identifier;
    ctx.graph_guard_handle.data = &ctx.graph_guard;
    ctx.graph_guard_handle.context = context;
    ctx.participant->set_on_graph_change(trigger_graph_guard, &ctx.graph_guard);
    if (!ctx.discovery_info.create(*ctx.participant)) {
        RMW_SET_ERROR_MSG("rmw_rosette: rmw_init: cannot create the ros_discovery_info writer "
                          "(participant writer table full)");
        destroy_participant(ctx);
        context_pool().release(impl);
        return RMW_RET_ERROR;
    }

    context->instance_id = options->instance_id;
    context->implementation_identifier = rmw_rosette_identifier;
    context->impl = impl;
    context->actual_domain_id = ctx.domain_id;
    return RMW_RET_OK;
}

/**
 * @brief Marks a context as no longer accepting new work.
 *
 * `RTPSParticipant` has no explicit departure announcement — matched
 * peers learn of this participant going away only through SPDP lease
 * expiry, at the configured `leaseDuration` (see
 * `SPDPdiscoveredParticipantData` in rtps/types.hpp), regardless of when
 * `rmw_shutdown()` is called relative to `rmw_context_fini()`.
 *
 * @param[in] context Context to shut down.
 * @return `RMW_RET_OK`, or `RMW_RET_INVALID_ARGUMENT` if `context` or its
 *   `impl` is null.
 */
rmw_ret_t rmw_shutdown(rmw_context_t* context) {
    if (!context || !context->impl) return RMW_RET_INVALID_ARGUMENT;
    return RMW_RET_OK;
}

/**
 * @brief Tears down the RTPS participant and transport behind a context
 * and returns its slot to the pool.
 *
 * @param[in,out] context Context to finalize; `impl` is set to `nullptr`
 *   on return.
 * @return `RMW_RET_OK`, or `RMW_RET_INVALID_ARGUMENT` if `context` or its
 *   `impl` is null.
 */
rmw_ret_t rmw_context_fini(rmw_context_t* context) {
    if (!context || !context->impl) return RMW_RET_INVALID_ARGUMENT;
    rmw_context_impl_s* impl = context->impl;
    rmw_rosette::Context& ctx = impl->rosette_context;
    if (ctx.constructed) {
        report_transport_stats(*ctx.transport);
        report_writer_evictions(*ctx.participant);
        destroy_participant(ctx);
    }
    context_pool().release(impl);
    context->impl = nullptr;
    return RMW_RET_OK;
}

}  // extern "C"
