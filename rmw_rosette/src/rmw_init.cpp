/**
 * @file rmw_init.cpp
 * @brief rmw_init/shutdown/context_fini — owns the one RTPSParticipant.
 */

#include "rmw_rosette/identifier.h"
#include "rmw_rosette/types.hpp"

#include <rmw/init.h>
#include <rmw/init_options.h>
#include <rmw/rmw.h>
#include <rmw/error_handling.h>

extern "C" {

const char * const rmw_rosette_identifier = RMW_ROSETTE_ID;

/// Opaque per-context state rmw/init.h expects each vendor to define.
struct rmw_context_impl_s {
    rmw_rosette::Context rosette_context;
};

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
    // TODO: domain_id, security_options, discovery_options — plumb through
    // to Context::domain_id and the SPDP multicast locator setup that
    // RTPSParticipant::add_ipv4_address()/announce() already support.
    return RMW_RET_OK;
}

rmw_ret_t rmw_init_options_copy(const rmw_init_options_t* src, rmw_init_options_t* dst) {
    (void)src; (void)dst;
    RMW_SET_ERROR_MSG("rmw_rosette: rmw_init_options_copy not implemented");
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_init_options_fini(rmw_init_options_t* init_options) {
    if (!init_options) return RMW_RET_INVALID_ARGUMENT;
    *init_options = rmw_get_zero_initialized_init_options();
    return RMW_RET_OK;
}

rmw_ret_t rmw_init(const rmw_init_options_t* options, rmw_context_t* context) {
    if (!options || !context) return RMW_RET_INVALID_ARGUMENT;

    // TODO: real allocation should go through options->allocator, not new —
    // matching the no-heap posture of the rest of rosette::rtps means this
    // probably wants a static/pool-allocated rmw_context_impl_s rather than
    // one per rmw_init() call. Left as `new` here only so the shape reads
    // clearly; revisit before this runs on-target.
    auto* impl = new rmw_context_impl_s{};

    // TODO: construct impl->rosette_context.participant here. Needs:
    //   - a domain_id (from options, once rmw_init_options_init plumbs it)
    //   - an ITransport& — the concrete rosette::net socket backend for
    //     this board (POSIX / FreeRTOS+TCP / WIZnet), which today is
    //     selected in rosette::App / the board's CMake config, not
    //     something rmw_init() has visibility into yet. This is the main
    //     open wiring question for a real rmw_init().
    //   - rtps_platform_make_guid_prefix() is already implemented per
    //     board (see participant.hpp's platform hook) and needs no changes.

    context->instance_id = options->instance_id;
    context->implementation_identifier = rmw_rosette_identifier;
    context->impl = impl;
    context->actual_domain_id = 0;  // TODO: from options once available.
    return RMW_RET_OK;
}

rmw_ret_t rmw_shutdown(rmw_context_t* context) {
    if (!context || !context->impl) return RMW_RET_INVALID_ARGUMENT;
    // TODO: tear down the participant cleanly (SPDP goodbye / lease
    // expiry announcement) rather than just dropping it.
    return RMW_RET_OK;
}

rmw_ret_t rmw_context_fini(rmw_context_t* context) {
    if (!context || !context->impl) return RMW_RET_INVALID_ARGUMENT;
    delete context->impl;
    context->impl = nullptr;
    return RMW_RET_OK;
}

}  // extern "C"
