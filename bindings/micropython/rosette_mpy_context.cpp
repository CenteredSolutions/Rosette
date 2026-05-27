/**
 * @file bindings/micropython/rosette_mpy_context.cpp
 * @brief `rosette.Context` implementation unit.
 *
 * The context owns backend lifecycle and holds a reference to the Python
 * Transport object. All socket I/O is delegated to that Transport. The context
 * stores agent_ip_obj and agent_port only after _set_transport() is called,
 * so the backend can read them at start() time.
 */

#include "rosette_mpy_backend.hpp"

extern "C" {
#include "py/objlist.h"
#include "py/nlr.h"
#include "py/runtime.h"
}

#include <cstring>
#include <cstdio>
#include <cstdlib>

namespace rosette::bindings::micropython {

namespace {

constexpr uint32_t kDefaultClientKey      = 0x01020304u;
constexpr uint16_t kDefaultExecutorHandles = 8u;
constexpr uint32_t kDefaultSpinTimeoutMs  = 1u;

[[noreturn]] void RaiseRuntimeError(const char* message)
{
    mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT(message));
}

void ContextPrint(const mp_print_t* print,
                  mp_obj_t self_in,
                  mp_print_kind_t kind)
{
    (void)kind;
    const auto* self = static_cast<mp_rosette_context_obj_t*>(MP_OBJ_TO_PTR(self_in));
    mp_printf(print,
              "Context(started=%d)",
              self->started ? 1 : 0);
}

mp_obj_t ContextMakeNew(const mp_obj_type_t* type,
                        size_t n_args,
                        size_t n_kw,
                        const mp_obj_t* all_args)
{
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_client_key,       MP_ARG_INT, {.u_int = static_cast<mp_int_t>(kDefaultClientKey)} },
        { MP_QSTR_domain_id,        MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_executor_handles, MP_ARG_INT, {.u_int = static_cast<mp_int_t>(kDefaultExecutorHandles)} },
        { MP_QSTR_spin_timeout_ms,  MP_ARG_INT, {.u_int = static_cast<mp_int_t>(kDefaultSpinTimeoutMs)} },
    };

    mp_arg_val_t parsed[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all_kw_array(n_args, n_kw, all_args,
                              MP_ARRAY_SIZE(allowed_args), allowed_args, parsed);

    auto* self = mp_obj_malloc(mp_rosette_context_obj_t, type);

    self->transport_obj     = mp_const_none;
    self->agent_ip_obj      = mp_const_none;
    self->subscriptions     = mp_obj_new_list(0, nullptr);

    self->client_key        = static_cast<uint32_t>(parsed[0].u_int);
    self->domain_id         = static_cast<uint8_t>(parsed[1].u_int);
    self->executor_handles  = static_cast<uint16_t>(parsed[2].u_int);
    self->spin_timeout_us   = static_cast<uint32_t>(parsed[3].u_int) * 1000u;
    self->local_port        = 0u;
    self->agent_port        = 8888u;
    self->backend_start_error = 0u;

    self->started = false;
    self->closed  = false;
    self->backend = nullptr;

    return MP_OBJ_FROM_PTR(self);
}

/**
 * @brief Wires a Python Transport object into this context.
 *
 * Stores the transport reference and reads agent_ip / agent_port from the
 * transport so the backend can use them at start() time. Initialises the C++
 * CustomTransport adapter on the transport. Idempotent.
 *
 * Python signature: ``ctx._set_transport(transport) -> None``
 */
mp_obj_t ContextSetTransportMethod(size_t n_args, const mp_obj_t* args)
{
    auto* self = static_cast<mp_rosette_context_obj_t*>(MP_OBJ_TO_PTR(args[0]));
    if (self->transport_obj != mp_const_none) {
        return mp_const_none;
    }

    mp_obj_t transport_native = mp_obj_cast_to_native_base(
        args[1], MP_OBJ_FROM_PTR(&rosette_transport_type));
    if (transport_native == MP_OBJ_NULL) {
        mp_raise_TypeError(MP_ERROR_TEXT("_set_transport expects a Transport"));
    }
    auto* transport = static_cast<mp_rosette_transport_obj_t*>(
        MP_OBJ_TO_PTR(transport_native));

    // Set read_budget from spin_timeout.
    transport->read_budget_us = self->spin_timeout_us;
    transport->python_obj = args[1];

    self->transport_obj = args[1];
    self->agent_ip_obj = n_args >= 3 ? args[2] : mp_const_none;
    self->agent_port = n_args >= 4
        ? static_cast<uint16_t>(mp_obj_get_int(args[3]))
        : self->agent_port;

    if (!TransportInit(transport)) {
        RaiseRuntimeError("rosette: failed to initialize transport adapter");
    }

    return mp_const_none;
}

/**
 * @brief Returns the Transport object attached to this context.
 *
 * Python signature: ``ctx.transport() -> Transport | None``
 */
mp_obj_t ContextTransportMethod(mp_obj_t self_in)
{
    const auto* self = static_cast<mp_rosette_context_obj_t*>(MP_OBJ_TO_PTR(self_in));
    return self->transport_obj;
}

mp_obj_t ContextStartMethod(mp_obj_t self_in)
{
    auto* self = static_cast<mp_rosette_context_obj_t*>(MP_OBJ_TO_PTR(self_in));
    if (!self->started) {
        if (self->transport_obj == mp_const_none) {
            RaiseRuntimeError("rosette: call open() before start()");
        }
        if (!ContextStart(self)) {
            mp_raise_msg_varg(&mp_type_RuntimeError,
                              MP_ERROR_TEXT("rosette backend start failed: %s"),
                              ContextStartErrorMessage(self));
        }
        self->started = true;
        self->closed  = false;
    }
    return mp_const_true;
}

mp_obj_t ContextTickMethod(mp_obj_t self_in)
{
    auto* self = static_cast<mp_rosette_context_obj_t*>(MP_OBJ_TO_PTR(self_in));
    if (!self->started) {
        return mp_const_false;
    }
    return mp_obj_new_bool(ContextTick(self));
}

mp_obj_t ContextCloseMethod(mp_obj_t self_in)
{
    auto* self = static_cast<mp_rosette_context_obj_t*>(MP_OBJ_TO_PTR(self_in));
    if (self->started) {
        ContextFini(self);
        self->started = false;
    }
    if (self->transport_obj != mp_const_none) {
        mp_obj_t transport_native = mp_obj_cast_to_native_base(
            self->transport_obj, MP_OBJ_FROM_PTR(&rosette_transport_type));
        if (transport_native == MP_OBJ_NULL) {
            self->transport_obj = mp_const_none;
            self->closed = true;
            return mp_const_none;
        }
        auto* transport = static_cast<mp_rosette_transport_obj_t*>(
            MP_OBJ_TO_PTR(transport_native));
        TransportFini(transport);
        // Call Python close() on the transport.
        nlr_buf_t nlr;
        if (nlr_push(&nlr) == 0) {
            mp_call_function_0(mp_load_attr(self->transport_obj, MP_QSTR_close));
            nlr_pop();
        }
        self->transport_obj = mp_const_none;
    }
    self->closed = true;
    return mp_const_none;
}

mp_obj_t ContextStartedMethod(mp_obj_t self_in)
{
    const auto* self = static_cast<mp_rosette_context_obj_t*>(MP_OBJ_TO_PTR(self_in));
    return mp_obj_new_bool(self->started);
}

}  // namespace

}  // namespace rosette::bindings::micropython

extern "C" {

extern const mp_obj_dict_t rosette_context_locals_dict;

mp_obj_t rosette_context_start(mp_obj_t s)
    { return rosette::bindings::micropython::ContextStartMethod(s); }
mp_obj_t rosette_context_tick(mp_obj_t s)
    { return rosette::bindings::micropython::ContextTickMethod(s); }
mp_obj_t rosette_context_close(mp_obj_t s)
    { return rosette::bindings::micropython::ContextCloseMethod(s); }
mp_obj_t rosette_context_started(mp_obj_t s)
    { return rosette::bindings::micropython::ContextStartedMethod(s); }
mp_obj_t rosette_context_set_transport(mp_obj_t s, mp_obj_t t)
    { const mp_obj_t args[2] = {s, t}; return rosette::bindings::micropython::ContextSetTransportMethod(2, args); }
mp_obj_t rosette_context_set_transport_var(size_t n, const mp_obj_t* a)
    { return rosette::bindings::micropython::ContextSetTransportMethod(n, a); }
mp_obj_t rosette_context_transport(mp_obj_t s)
    { return rosette::bindings::micropython::ContextTransportMethod(s); }
mp_obj_t rosette_context_create_node(size_t n, const mp_obj_t* a);  /* defined in rosette_mpy_node.cpp */

const mp_obj_full_type_t rosette_context_type = {
    { &mp_type_type },
    MP_TYPE_FLAG_NONE,
    MP_QSTR_Context,
    1, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3,
    {
        (const void*)rosette::bindings::micropython::ContextMakeNew,
        (const void*)rosette::bindings::micropython::ContextPrint,
        &rosette_context_locals_dict,
        NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
    },
};

}  // extern "C"
