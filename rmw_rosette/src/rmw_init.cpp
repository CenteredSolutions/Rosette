/**
 * @file rmw_init.cpp
 * @brief rmw_init/shutdown/context_fini — owns the one RTPSParticipant.
 */

#include "rmw_rosette/identifier.h"
#include "rmw_rosette/pool.hpp"
#include "rmw_rosette/types.hpp"

#include "rosette/platform/host_runtime.hpp"  // rosette_platform_get_ipv4_addr

#include <rmw/init.h>
#include <rmw/init_options.h>
#include <rmw/rmw.h>
#include <rmw/error_handling.h>

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
 *   kMaxParticipantId is taken or the SPDP port cannot be opened.
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
        ctx.transport = new (ctx.transport_storage) rtps::SocketUdpTransport(
            ctx.provider,
            metatraffic_unicast_port(ctx.domain_id, id),
            default_unicast_port(ctx.domain_id, id),
            kSpdpMulticastAddr,
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
 * rcl calls this before rmw_init(), then may override fields (most
 * notably `domain_id`) before passing the options on.
 *
 * @param[out] init_options Options struct to fill.
 * @param[in] allocator Allocator to record on the options; unused beyond
 *   that, since nothing here allocates.
 * @return `RMW_RET_OK`, or `RMW_RET_INVALID_ARGUMENT` if `init_options`
 *   is null.
 */
rmw_ret_t rmw_init_options_init(rmw_init_options_t* init_options, rcutils_allocator_t allocator) {
    if (!init_options) return RMW_RET_INVALID_ARGUMENT;
    init_options->instance_id = 0;
    init_options->implementation_identifier = rmw_rosette_identifier;
    init_options->allocator = allocator;
    init_options->impl = nullptr;
    init_options->enclave = nullptr;
    init_options->domain_id = 0;  // RMW_DEFAULT_DOMAIN_ID
    init_options->security_options = rmw_get_zero_initialized_security_options();
    return RMW_RET_OK;
}

/**
 * @brief Deep-copies an rmw_init_options_t.
 *
 * @param[in] src Options to copy from.
 * @param[out] dst Options to copy into.
 * @return `RMW_RET_OK`, or `RMW_RET_INVALID_ARGUMENT` if either argument
 *   is null.
 */
rmw_ret_t rmw_init_options_copy(const rmw_init_options_t* src, rmw_init_options_t* dst) {
    if (!src || !dst) return RMW_RET_INVALID_ARGUMENT;
    *dst = *src;  // Every field here is a plain value or non-owned pointer.
    return RMW_RET_OK;
}

/**
 * @brief Releases an rmw_init_options_t.
 *
 * @param[in,out] init_options Options to finalize; reset to the
 *   zero-initialized state on return.
 * @return `RMW_RET_OK`, or `RMW_RET_INVALID_ARGUMENT` if `init_options`
 *   is null.
 */
rmw_ret_t rmw_init_options_fini(rmw_init_options_t* init_options) {
    if (!init_options) return RMW_RET_INVALID_ARGUMENT;
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
 * over `rosette::net::posix::PosixSocketProvider` on the first free
 * participant ID (open_transport_on_free_ports()), then placement-news the
 * `RTPSParticipant` that borrows it. The participant advertises the host's
 * primary IPv4 address (`rosette_platform_get_ipv4_addr()`, overridable with
 * `ROSETTE_LOCAL_IP`) rather than loopback: stock DDS implementations
 * ignore loopback locators from participants they cannot prove are local.
 *
 * @param[in] options Init options; `domain_id` and `instance_id` are used.
 * @param[out] context Context to initialize.
 * @return `RMW_RET_OK` on success; `RMW_RET_INVALID_ARGUMENT` if either
 *   argument is null; `RMW_RET_ERROR` if the context pool is exhausted or
 *   no participant ID with free ports was found.
 *
 * Verification:
 * - Correct means: a stock RMW in the same domain discovers this
 *   participant and matches its endpoints.
 * - Covered by: tests/ros2/run_rmw_interop_loopback.sh.
 *
 * References:
 * - rmw/init.h (Humble), `rmw_init`
 * - OMG DDSI-RTPS 2.5 §8.5.3 (SPDP), §9.6.1.1 (port mapping)
 */
rmw_ret_t rmw_init(const rmw_init_options_t* options, rmw_context_t* context) {
    if (!options || !context) return RMW_RET_INVALID_ARGUMENT;

    rmw_context_impl_s* impl = context_pool().acquire();
    if (!impl) {
        RMW_SET_ERROR_MSG("rmw_rosette: rmw_init: context pool exhausted "
                          "(RMW_ROSETTE_MAX_CONTEXTS)");
        return RMW_RET_ERROR;
    }

    rmw_rosette::Context& ctx = impl->rosette_context;
    ctx.domain_id = static_cast<uint8_t>(options->domain_id);
    uint8_t participant_id = 0;
    if (!open_transport_on_free_ports(ctx, participant_id)) {
        RMW_SET_ERROR_MSG("rmw_rosette: rmw_init: no free RTPS participant ports "
                          "in this domain");
        context_pool().release(impl);
        return RMW_RET_ERROR;
    }

    ctx.participant = new (ctx.participant_storage) rtps::RTPSParticipant(
        ctx.domain_id, participant_id, *ctx.transport);

    ctx.participant->add_ipv4_address(rosette_platform_get_ipv4_addr());
    ctx.driver = new (ctx.driver_storage) rtps::ParticipantDriver(
        *ctx.participant, *ctx.transport, ctx.provider, ctx.rx_buffer, sizeof(ctx.rx_buffer));
    ctx.constructed = true;

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
        ctx.driver->~ParticipantDriver();
        ctx.participant->~RTPSParticipant();
        ctx.transport->~SocketUdpTransport();
        ctx.constructed = false;
    }
    context_pool().release(impl);
    context->impl = nullptr;
    return RMW_RET_OK;
}

}  // extern "C"
