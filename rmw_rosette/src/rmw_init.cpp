/**
 * @file rmw_init.cpp
 * @brief rmw_init/shutdown/context_fini — owns the one RTPSParticipant.
 */

#include "rmw_rosette/identifier.h"
#include "rmw_rosette/pool.hpp"
#include "rmw_rosette/types.hpp"

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

// Standard RTPS/DDSI discovery ports — OMG DDSI-RTPS 2.5 §9.6.1.1:
//   PB = 7400, DG = 250, d0 = 0 (multicast metatraffic),
//   d1 = 10 (unicast metatraffic), d3 = 11 (unicast user data).
// Duplicated from the equivalent (TU-private) helper in
// src/rtps/participant.cpp rather than exposed from there, since it's a
// two-line formula and participant.cpp's version is intentionally
// unexported (RTPSParticipant computes its own fallback internally when
// the transport doesn't report a port — see participant.cpp's
// has_addresses()-gated tick()/announce() callers).
uint16_t metatraffic_unicast_port(uint8_t domain_id, uint8_t participant_id) noexcept {
    return static_cast<uint16_t>(7400u + 250u * domain_id + 10u + 2u * participant_id);
}
uint16_t default_unicast_port(uint8_t domain_id, uint8_t participant_id) noexcept {
    return static_cast<uint16_t>(7400u + 250u * domain_id + 11u + 2u * participant_id);
}

// 239.255.0.1, big-endian — OMG DDSI-RTPS 2.5 §9.6.1, the well-known SPDP
// discovery multicast group (see spdp.hpp).
constexpr uint32_t kSpdpMulticastAddrBe = 0xEFFF0001u;

rmw_rosette::Pool<rmw_context_impl_s, RMW_ROSETTE_MAX_CONTEXTS>& context_pool() {
    static rmw_rosette::Pool<rmw_context_impl_s, RMW_ROSETTE_MAX_CONTEXTS> pool;
    return pool;
}

// Distinguishes multiple local participants sharing one domain/host.
// RMW_ROSETTE_MAX_CONTEXTS is typically 1, so this rarely matters; kept
// simple rather than plumbed through options for that reason.
uint8_t next_participant_id() noexcept {
    static uint8_t next = 0;
    return next++;
}

}  // namespace

extern "C" {

const char * const rmw_rosette_identifier = RMW_ROSETTE_ID;

const char* rmw_get_implementation_identifier(void) {
    return rmw_rosette_identifier;
}

const char* rmw_get_serialization_format(void) {
    // Standard CDR, same as every other DDS-backed rmw — we reuse the
    // rosidl_typesupport_microxrcedds_c serializers as-is (see
    // type_support.hpp), which already produce this.
    return "cdr";
}

rmw_ret_t rmw_init_options_init(rmw_init_options_t* init_options, rcutils_allocator_t allocator) {
    if (!init_options) return RMW_RET_INVALID_ARGUMENT;
    init_options->instance_id = 0;
    init_options->implementation_identifier = rmw_rosette_identifier;
    init_options->allocator = allocator;
    init_options->impl = nullptr;
    init_options->enclave = nullptr;
    init_options->domain_id = 0;  // RMW_DEFAULT_DOMAIN_ID; overridden by the caller before rmw_init().
    init_options->security_options = rmw_get_zero_initialized_security_options();
    return RMW_RET_OK;
}

rmw_ret_t rmw_init_options_copy(const rmw_init_options_t* src, rmw_init_options_t* dst) {
    if (!src || !dst) return RMW_RET_INVALID_ARGUMENT;
    *dst = *src;  // Every field here is a plain value or non-owned pointer.
    return RMW_RET_OK;
}

rmw_ret_t rmw_init_options_fini(rmw_init_options_t* init_options) {
    if (!init_options) return RMW_RET_INVALID_ARGUMENT;
    *init_options = rmw_get_zero_initialized_init_options();
    return RMW_RET_OK;
}

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
    const uint8_t participant_id = next_participant_id();

    // Placement-new the transport, then the participant that borrows it —
    // see types.hpp's Context comment for why these aren't plain members.
    ctx.transport = new (ctx.transport_storage) rtps::SocketUdpTransport(
        ctx.provider,
        metatraffic_unicast_port(ctx.domain_id, participant_id),
        default_unicast_port(ctx.domain_id, participant_id),
        kSpdpMulticastAddrBe);

    if (!ctx.transport->open()) {
        RMW_SET_ERROR_MSG("rmw_rosette: rmw_init: SocketUdpTransport::open() failed "
                          "(ports already in use?)");
        ctx.transport->~SocketUdpTransport();
        context_pool().release(impl);
        return RMW_RET_ERROR;
    }

    ctx.participant = new (ctx.participant_storage) rtps::RTPSParticipant(
        ctx.domain_id, participant_id, *ctx.transport);

    // Loopback is enough for a single-host graph (the common case during
    // bring-up); a board target adds its real interface address the same
    // way once rosette::net's address providers are wired in here — see
    // the transport-selection gap this used to be blocked on, now
    // resolved for the POSIX case specifically.
    ctx.participant->add_ipv4_address(0x7F000001u);
    ctx.constructed = true;

    context->instance_id = options->instance_id;
    context->implementation_identifier = rmw_rosette_identifier;
    context->impl = impl;
    context->actual_domain_id = ctx.domain_id;
    return RMW_RET_OK;
}

rmw_ret_t rmw_shutdown(rmw_context_t* context) {
    if (!context || !context->impl) return RMW_RET_INVALID_ARGUMENT;
    // TODO: announce departure (SPDP lease expiry) before tearing down,
    // so matched peers don't have to wait out the full lease duration to
    // notice. RTPSParticipant has no explicit "goodbye" API yet.
    return RMW_RET_OK;
}

rmw_ret_t rmw_context_fini(rmw_context_t* context) {
    if (!context || !context->impl) return RMW_RET_INVALID_ARGUMENT;
    rmw_context_impl_s* impl = context->impl;
    rmw_rosette::Context& ctx = impl->rosette_context;
    if (ctx.constructed) {
        ctx.participant->~RTPSParticipant();
        ctx.transport->~SocketUdpTransport();
        ctx.constructed = false;
    }
    context_pool().release(impl);
    context->impl = nullptr;
    return RMW_RET_OK;
}

}  // extern "C"
