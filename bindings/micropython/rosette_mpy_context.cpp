/**
 * @file bindings/micropython/rosette_mpy_context.cpp
 * @brief `rosette.Context` implementation unit.
 *
 * The context type owns the MicroPython socket object, the custom transport
 * wiring, and the Rosette micro-ROS backend instance. It is the binding's
 * top-level lifecycle owner and the root that child node, publisher, and
 * subscription objects reference.
 *
 * The implementation in this unit is intentionally isolated from schema and
 * poll code so context lifecycle changes do not require touching message codec
 * logic or stream hooks.
 */

#include "rosette_mpy_backend.hpp"
#include "rosette/ros/micro_ros/multicast_agent_locator.hpp"

extern "C" {
#include "py/objlist.h"
#include "py/nlr.h"
#include "py/objtuple.h"
#include "py/runtime.h"
#include "py/mperrno.h"
}

#include <cstring>
#include <cstdio>

namespace rosette::bindings::micropython {

extern const mp_obj_fun_builtin_var_t rosette_context_create_node_obj;
extern const mp_obj_fun_builtin_fixed_t rosette_context_connected_obj;
extern const mp_obj_fun_builtin_fixed_t rosette_context_send_packet_obj;
extern const mp_obj_fun_builtin_var_t rosette_context_recv_packet_obj;

namespace {

constexpr uint16_t kDefaultAgentPort = 8888u;
constexpr uint32_t kDefaultClientKey = 0x01020304u;
constexpr uint16_t kDefaultExecutorHandles = 8u;
constexpr uint32_t kDefaultSpinTimeoutMs = 1u;
constexpr const char* kDefaultAgentIp = "127.0.0.1";

/**
 * @brief Raises a runtime error when a socket setup step fails.
 *
 * The context constructor performs a sequence of MicroPython object calls to
 * import the `socket` module, create a UDP socket, and configure the chosen
 * local and remote endpoints. Centralizing the error text keeps those failure
 * paths consistent and makes the socket bring-up sequence easier to review.
 *
 * @param[in] message Human-readable detail about the failed step.
 */
[[noreturn]] void RaiseSocketSetupError(const char* message)
{
    mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT(message));
}

/**
 * @brief Looks up and caches one bound method from a Python object.
 *
 * Rosette's transport callback path should not repeat dynamic attribute lookup
 * for every packet. The context constructor resolves the small set of socket
 * methods it needs once and stores the bound method objects in the context.
 *
 * @param[in] object Python object that owns the method.
 * @param[in] method_name QSTR name of the method to resolve.
 * @return Bound method object ready to pass to `mp_call_function_*`.
 */
mp_obj_t LoadBoundMethod(mp_obj_t object, const qstr method_name)
{
    return mp_load_attr(object, method_name);
}

/**
 * @brief Imports the MicroPython `socket` module.
 *
 * The binding uses the object runtime import path so the same constructor code
 * works for both built-in user modules and dynamic native modules.
 *
 * @return Imported `socket` module object.
 */
mp_obj_t ImportSocketModule()
{
    return mp_import_name(MP_QSTR_socket, mp_const_none, MP_OBJ_NEW_SMALL_INT(0));
}

/**
 * @brief Constructs one UDP socket object from the imported socket module.
 *
 * The constructor resolves `socket.AF_INET` and `socket.SOCK_DGRAM` from the
 * imported module and uses the public `socket.socket(...)` callable to build
 * the transport socket. That keeps the binding on the supported object layer
 * instead of depending on MicroPython NIC internals.
 *
 * @param[in] socket_module Imported `socket` module object.
 * @return Newly created UDP socket object.
 */
mp_obj_t CreateUdpSocket(mp_obj_t socket_module)
{
    mp_obj_t ctor = mp_load_attr(socket_module, MP_QSTR_socket);
    mp_obj_t args[2] = {
        mp_load_attr(socket_module, MP_QSTR_AF_INET),
        mp_load_attr(socket_module, MP_QSTR_SOCK_DGRAM),
    };
    return mp_call_function_n_kw(ctor, 2, 0, args);
}

/**
 * @brief Resolves one socket endpoint through `socket.getaddrinfo(...)`.
 *
 * MicroPython ports do not all use the same concrete address-object shape.
 * The unix port expects a packed sockaddr buffer, while other ports commonly
 * accept a host/port tuple. Resolving through `getaddrinfo` lets the socket
 * module produce the address object that matches the active port.
 *
 * @param[in] socket_module Imported `socket` module object.
 * @param[in] host_obj Python string naming the endpoint host.
 * @param[in] port UDP port number.
 * @return Address object suitable for the active port's `bind()` or
 * `connect()` implementation.
 */
mp_obj_t ResolveAddress(mp_obj_t socket_module,
                        mp_obj_t host_obj,
                        const uint16_t port)
{
    mp_obj_t getaddrinfo = mp_load_attr(socket_module, MP_QSTR_getaddrinfo);
    mp_obj_t args[4] = {
        host_obj,
        mp_obj_new_int_from_uint(port),
        mp_load_attr(socket_module, MP_QSTR_AF_INET),
        mp_load_attr(socket_module, MP_QSTR_SOCK_DGRAM),
    };
    mp_obj_t info_list = mp_call_function_n_kw(getaddrinfo, 4, 0, args);
    mp_obj_t first = mp_obj_subscr(info_list, MP_OBJ_NEW_SMALL_INT(0), MP_OBJ_SENTINEL);
    return mp_obj_subscr(first, mp_obj_new_int(-1), MP_OBJ_SENTINEL);
}

/**
 * @brief Returns whether one caught exception is `OSError(EPERM)`.
 *
 * Constructor-time UDP `connect()` is useful because it simplifies the later
 * transport callbacks to `send()` and `recv()`, but some local smoke
 * environments prohibit network syscalls and surface that as `OSError(EPERM)`.
 * The binding optionally treats just that error as a non-fatal fallback case.
 *
 * @param[in] exception Raised MicroPython exception object.
 * @return `true` when the exception is an `OSError` whose value is `EPERM`.
 */
bool IsOperationNotPermittedException(mp_obj_t exception)
{
    const auto* exc_base = static_cast<mp_obj_base_t*>(MP_OBJ_TO_PTR(exception));
    if (!mp_obj_is_subclass_fast(MP_OBJ_FROM_PTR(exc_base->type), MP_OBJ_FROM_PTR(&mp_type_OSError))) {
        return false;
    }

    mp_obj_t value = mp_obj_exception_get_value(exception);
    if (!mp_obj_is_small_int(value)) {
        return false;
    }
    return mp_obj_get_int(value) == MP_EPERM;
}

/**
 * @brief Attempts to connect a UDP socket and optionally tolerates `EPERM`.
 *
 * The binding defaults to connected UDP sockets because that matches the
 * eventual transport callback shape. During local unix-port smoke testing, a
 * sandboxed host may reject the `connect()` syscall with `EPERM`. When the
 * context enables `connect_fallback_eperm`, that specific failure leaves the
 * socket usable but marks the context as unconnected.
 *
 * @param[in,out] self Context that owns the socket and connect policy.
 * @return `true` when the socket is connected or when an allowed local
 * fallback was applied.
 */
bool TryConnectSocket(mp_rosette_context_obj_t* self)
{
    nlr_buf_t nlr;
    if (nlr_push(&nlr) == 0) {
        mp_call_function_1(self->meth_connect, self->remote_addr_obj);
        nlr_pop();
        self->connected = true;
        return true;
    }

    mp_obj_t exception = MP_OBJ_FROM_PTR(nlr.ret_val);
    if (self->connect_fallback_eperm && IsOperationNotPermittedException(exception)) {
        self->connected = false;
        return true;
    }

    nlr_raise(exception);
}

/**
 * @brief Closes the MicroPython socket owned by a context when present.
 *
 * The context owns the transport socket for its entire lifetime, and close is
 * intentionally idempotent so explicit `close()` calls and repeated shutdown
 * paths behave the same way.
 *
 * @param[in,out] self Context that owns the socket object.
 */
void CloseSocketIfNeeded(mp_rosette_context_obj_t* self)
{
    if (self == nullptr || self->closed || self->socket_obj == mp_const_none ||
        self->meth_close == mp_const_none) {
        return;
    }

    mp_call_function_0(self->meth_close);
    self->closed = true;
    self->connected = false;
    self->transport.socket_ready = false;
}

/**
 * @brief Prints a compact diagnostic representation of a context object.
 *
 * The representation is intentionally geared toward bring-up and smoke testing
 * so users can quickly confirm whether socket creation and backend startup ran.
 *
 * @param[in] print MicroPython print destination.
 * @param[in] self Context object to display.
 * @param[in] kind Print behavior requested by MicroPython.
 */
void ContextPrint(const mp_print_t* print,
                  mp_obj_t self_in,
                  mp_print_kind_t kind)
{
    (void)kind;
    const auto* self = static_cast<mp_rosette_context_obj_t*>(MP_OBJ_TO_PTR(self_in));
    mp_printf(print,
              "Context(agent_port=%u, local_port=%u, started=%d, connected=%d)",
              static_cast<unsigned>(self->agent_port),
              static_cast<unsigned>(self->local_port),
              self->started ? 1 : 0,
              self->connected ? 1 : 0);
}

/**
 * @brief Creates and configures one `rosette.Context` object.
 *
 * The constructor owns the entire Python-side transport setup sequence:
 * importing `socket`, constructing a UDP socket, optionally binding the local
 * endpoint, connecting the remote endpoint, switching the socket to
 * non-blocking mode, and caching the small set of bound socket methods needed
 * by the transport callback path.
 *
 * @param[in] type MicroPython type object being instantiated.
 * @param[in] n_args Number of positional arguments.
 * @param[in] n_kw Number of keyword arguments.
 * @param[in] all_args Positional and keyword argument values.
 * @return Newly initialized `rosette.Context` instance.
 */
mp_obj_t ContextMakeNew(const mp_obj_type_t* type,
                        size_t n_args,
                        size_t n_kw,
                        const mp_obj_t* all_args)
{
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_agent_ip, MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_agent_port, MP_ARG_INT, {.u_int = static_cast<mp_int_t>(kDefaultAgentPort)} },
        { MP_QSTR_local_port, MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_client_key, MP_ARG_INT, {.u_int = static_cast<mp_int_t>(kDefaultClientKey)} },
        { MP_QSTR_domain_id, MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_executor_handles, MP_ARG_INT, {.u_int = static_cast<mp_int_t>(kDefaultExecutorHandles)} },
        { MP_QSTR_spin_timeout_ms, MP_ARG_INT, {.u_int = static_cast<mp_int_t>(kDefaultSpinTimeoutMs)} },
        { MP_QSTR_connected_udp, MP_ARG_BOOL, {.u_bool = true} },
        { MP_QSTR_connect_fallback_eperm, MP_ARG_BOOL, {.u_bool = true} },
    };

    mp_arg_val_t parsed[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all_kw_array(n_args,
                              n_kw,
                              all_args,
                              MP_ARRAY_SIZE(allowed_args),
                              allowed_args,
                              parsed);

    auto* self = mp_obj_malloc(mp_rosette_context_obj_t, type);
    std::memset(&self->transport, 0, sizeof(self->transport));

    self->socket_module_obj = mp_const_none;
    self->socket_obj = mp_const_none;
    self->agent_ip_obj = mp_const_none;
    self->remote_addr_obj = mp_const_none;
    self->local_addr_obj = mp_const_none;
    self->meth_bind = mp_const_none;
    self->meth_connect = mp_const_none;
    self->meth_send = mp_const_none;
    self->meth_sendto = mp_const_none;
    self->meth_recv = mp_const_none;
    self->meth_recvfrom = mp_const_none;
    self->meth_setblocking = mp_const_none;
    self->meth_close = mp_const_none;
    self->subscriptions = mp_obj_new_list(0, nullptr);

    self->client_key = static_cast<uint32_t>(parsed[3].u_int);
    self->spin_timeout_ns = static_cast<uint32_t>(parsed[6].u_int) * 1000000u;
    self->agent_port = static_cast<uint16_t>(parsed[1].u_int);
    self->local_port = static_cast<uint16_t>(parsed[2].u_int);
    self->executor_handles = static_cast<uint16_t>(parsed[5].u_int);
    self->backend_start_error = 0u;
    self->domain_id = static_cast<uint8_t>(parsed[4].u_int);
    self->started = false;
    self->closed = false;
    self->connected = false;
    self->use_connected_udp = parsed[7].u_bool;
    self->connect_fallback_eperm = parsed[8].u_bool;
    self->backend = nullptr;

    self->transport.owner = self;
    self->transport.socket_ready = false;
    self->transport.read_budget_us = static_cast<uint32_t>(parsed[6].u_int) * 1000u;
    self->transport.implementation = nullptr;

    mp_obj_t agent_ip_obj = parsed[0].u_obj;
    if (agent_ip_obj == MP_OBJ_NULL) {
        agent_ip_obj = mp_obj_new_str(kDefaultAgentIp, std::strlen(kDefaultAgentIp));
    }
    self->agent_ip_obj = agent_ip_obj;

    self->socket_module_obj = ImportSocketModule();
    self->socket_obj = CreateUdpSocket(self->socket_module_obj);

    self->meth_bind = LoadBoundMethod(self->socket_obj, MP_QSTR_bind);
    self->meth_connect = LoadBoundMethod(self->socket_obj, MP_QSTR_connect);
    self->meth_send = LoadBoundMethod(self->socket_obj, MP_QSTR_send);
    self->meth_sendto = LoadBoundMethod(self->socket_obj, MP_QSTR_sendto);
    self->meth_recv = LoadBoundMethod(self->socket_obj, MP_QSTR_recv);
    self->meth_recvfrom = LoadBoundMethod(self->socket_obj, MP_QSTR_recvfrom);
    self->meth_setblocking = LoadBoundMethod(self->socket_obj, MP_QSTR_setblocking);
    self->meth_close = LoadBoundMethod(self->socket_obj, MP_QSTR_close);

    self->remote_addr_obj = ResolveAddress(
        self->socket_module_obj, agent_ip_obj, self->agent_port);
    if (self->local_port != 0u) {
        self->local_addr_obj = ResolveAddress(
            self->socket_module_obj,
            mp_obj_new_str("0.0.0.0", 7),
            self->local_port);
        mp_call_function_1(self->meth_bind, self->local_addr_obj);
    }

    if (self->use_connected_udp) {
        (void)TryConnectSocket(self);
    }

    mp_call_function_1(self->meth_setblocking, mp_const_false);
    self->transport.socket_ready = true;
    if (!TransportInit(self)) {
        RaiseSocketSetupError("failed to initialize rosette transport adapter");
    }
    return MP_OBJ_FROM_PTR(self);
}

/**
 * @brief Starts the Rosette backend for one context.
 *
 * This method is deliberately separate from the constructor so object creation
 * can validate MicroPython socket bring-up first and only then attempt backend
 * startup. That separation mirrors Rosette's existing context lifecycle.
 *
 * @param[in] self_in MicroPython context object.
 * @return `True` when the context is running after the call.
 */
mp_obj_t ContextStartMethod(mp_obj_t self_in)
{
    auto* self = static_cast<mp_rosette_context_obj_t*>(MP_OBJ_TO_PTR(self_in));
    if (!self->started) {
        if (!ContextStart(self)) {
            mp_raise_msg_varg(&mp_type_RuntimeError,
                              MP_ERROR_TEXT("rosette backend start failed: %s"),
                              ContextStartErrorMessage(self));
        }
        self->started = true;
        self->closed = false;
    }
    return mp_const_true;
}

/**
 * @brief Advances the backend executor for one context.
 *
 * @param[in] self_in MicroPython context object.
 * @return `True` when the tick completed successfully.
 */
mp_obj_t ContextTickMethod(mp_obj_t self_in)
{
    auto* self = static_cast<mp_rosette_context_obj_t*>(MP_OBJ_TO_PTR(self_in));
    if (!self->started) {
        return mp_const_false;
    }
    return mp_obj_new_bool(ContextTick(self));
}

/**
 * @brief Shuts down the backend and closes the owned socket.
 *
 * @param[in] self_in MicroPython context object.
 * @return `None`.
 */
mp_obj_t ContextCloseMethod(mp_obj_t self_in)
{
    auto* self = static_cast<mp_rosette_context_obj_t*>(MP_OBJ_TO_PTR(self_in));
    if (self->started) {
        ContextFini(self);
        self->started = false;
    }
    TransportFini(self);
    CloseSocketIfNeeded(self);
    return mp_const_none;
}

/**
 * @brief Returns whether the context has been started.
 *
 * @param[in] self_in MicroPython context object.
 * @return `True` when the backend start sequence has completed.
 */
mp_obj_t ContextStartedMethod(mp_obj_t self_in)
{
    const auto* self = static_cast<mp_rosette_context_obj_t*>(MP_OBJ_TO_PTR(self_in));
    return mp_obj_new_bool(self->started);
}

/**
 * @brief Returns the MicroPython socket object owned by this context.
 *
 * Exposes the underlying UDP socket so Python-side async code can register it
 * with the event loop's I/O queue (via ``queue_read``) for non-blocking
 * receives — specifically during agent discovery, where the Python layer
 * manages the send/recv cycle without holding any blocking syscall.
 *
 * @param[in] self_in MicroPython context object.
 * @return The socket object, or ``None`` when the context is closed.
 */
mp_obj_t ContextSocketMethod(mp_obj_t self_in)
{
    const auto* self = static_cast<mp_rosette_context_obj_t*>(MP_OBJ_TO_PTR(self_in));
    return self->socket_obj;
}

/**
 * @brief Builds a Micro XRCE-DDS GET_INFO discovery probe packet.
 *
 * Returns the 16-byte probe as a `bytes` object ready to pass to
 * `ctx.send_packet()` or directly to `socket.sendto()`.  The multicast
 * destination address is also returned as a pre-resolved socket address so
 * the Python layer can forward it straight to `sendto()`.
 *
 * Python signature: ``ctx._discovery_build_probe() -> (bytes, addr)``
 *
 * @param[in] self_in Context whose socket module is used to resolve the
 * multicast address.
 * @return 2-tuple ``(probe_bytes, multicast_addr)`` ready for ``sendto()``.
 */
mp_obj_t ContextDiscoveryBuildProbeMethod(mp_obj_t self_in)
{
    using namespace milkros::micro_ros::detail;

    auto* self = static_cast<mp_rosette_context_obj_t*>(MP_OBJ_TO_PTR(self_in));
    if (self->socket_obj == mp_const_none || self->closed) {
        mp_raise_msg(&mp_type_RuntimeError,
                     MP_ERROR_TEXT("Context socket is not available"));
    }

    uint8_t probe[kGetInfoPacketSize] = {};
    if (!build_get_info_packet(probe)) {
        mp_raise_msg(&mp_type_RuntimeError,
                     MP_ERROR_TEXT("Failed to build XRCE GET_INFO packet"));
    }

    mp_obj_t probe_bytes = mp_obj_new_bytes(probe, sizeof(probe));
    mp_obj_t multicast_addr = ResolveAddress(
        self->socket_module_obj,
        mp_obj_new_str("239.255.0.2", 10),
        7400u);

    mp_obj_t items[2] = { probe_bytes, multicast_addr };
    return mp_obj_new_tuple(2, items);
}

/**
 * @brief Attempts to parse one XRCE INFO reply and update the context endpoint.
 *
 * Called by the Python async loop after it receives a UDP packet from the
 * non-blocking socket.  Parses the packet for an IPv4 transport locator.  On
 * success, updates ``agent_ip_obj``, ``agent_port``, and ``remote_addr_obj``
 * in place and re-connects the socket when ``use_connected_udp`` is set, then
 * returns a ``(ip_str, port_int)`` tuple.  Returns ``None`` when the packet is
 * not a valid INFO reply or carries no usable locator — the Python layer
 * should discard and continue waiting.
 *
 * Python signature:
 *   ``ctx._discovery_parse_reply(packet: bytes) -> (str, int) | None``
 *
 * @param[in] self_in Context to update on a successful parse.
 * @param[in] packet_in Python ``bytes`` object containing the received packet.
 * @return ``(ip_str, port_int)`` tuple, or ``None`` when not a valid reply.
 */
mp_obj_t ContextDiscoveryParseReplyMethod(mp_obj_t self_in, mp_obj_t packet_in)
{
    using namespace milkros::micro_ros::detail;

    auto* self = static_cast<mp_rosette_context_obj_t*>(MP_OBJ_TO_PTR(self_in));

    mp_buffer_info_t info;
    mp_get_buffer_raise(packet_in, &info, MP_BUFFER_READ);

    char ip_str[16] = {};
    char port_str[8] = {};
    if (!parse_info_packet(static_cast<const uint8_t*>(info.buf),
                           info.len,
                           ip_str, sizeof(ip_str),
                           port_str, sizeof(port_str))) {
        return mp_const_none;
    }

    char* endptr = nullptr;
    const unsigned long port_val = std::strtoul(port_str, &endptr, 10);
    if (endptr == port_str || port_val == 0u || port_val > 65535u) {
        return mp_const_none;
    }
    const auto agent_port = static_cast<uint16_t>(port_val);

    // Update the context endpoint so start() reaches the discovered agent.
    self->agent_ip_obj = mp_obj_new_str(ip_str, std::strlen(ip_str));
    self->agent_port = agent_port;
    self->remote_addr_obj = ResolveAddress(
        self->socket_module_obj, self->agent_ip_obj, agent_port);

    if (self->use_connected_udp) {
        (void)TryConnectSocket(self);
    }

    mp_obj_t tuple_items[2] = {
        self->agent_ip_obj,
        mp_obj_new_int_from_uint(agent_port),
    };
    return mp_obj_new_tuple(2, tuple_items);
}

}  // namespace

MP_DEFINE_CONST_FUN_OBJ_1(rosette_context_start_obj, ContextStartMethod);
MP_DEFINE_CONST_FUN_OBJ_1(rosette_context_tick_obj, ContextTickMethod);
MP_DEFINE_CONST_FUN_OBJ_1(rosette_context_close_obj, ContextCloseMethod);
MP_DEFINE_CONST_FUN_OBJ_1(rosette_context_started_obj, ContextStartedMethod);
MP_DEFINE_CONST_FUN_OBJ_1(rosette_context_socket_obj, ContextSocketMethod);
MP_DEFINE_CONST_FUN_OBJ_1(rosette_context_discovery_build_probe_obj,
                           ContextDiscoveryBuildProbeMethod);
MP_DEFINE_CONST_FUN_OBJ_2(rosette_context_discovery_parse_reply_obj,
                           ContextDiscoveryParseReplyMethod);

static const mp_rom_map_elem_t rosette_context_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_start),                    MP_ROM_PTR(&rosette_context_start_obj) },
    { MP_ROM_QSTR(MP_QSTR_tick),                     MP_ROM_PTR(&rosette_context_tick_obj) },
    { MP_ROM_QSTR(MP_QSTR_close),                    MP_ROM_PTR(&rosette_context_close_obj) },
    { MP_ROM_QSTR(MP_QSTR_started),                  MP_ROM_PTR(&rosette_context_started_obj) },
    { MP_ROM_QSTR(MP_QSTR_connected),                MP_ROM_PTR(&rosette_context_connected_obj) },
    { MP_ROM_QSTR(MP_QSTR_socket),                   MP_ROM_PTR(&rosette_context_socket_obj) },
    { MP_ROM_QSTR(MP_QSTR_create_node),              MP_ROM_PTR(&rosette_context_create_node_obj) },
    { MP_ROM_QSTR(MP_QSTR_send_packet),              MP_ROM_PTR(&rosette_context_send_packet_obj) },
    { MP_ROM_QSTR(MP_QSTR_recv_packet),              MP_ROM_PTR(&rosette_context_recv_packet_obj) },
    { MP_ROM_QSTR(MP_QSTR__discovery_build_probe),   MP_ROM_PTR(&rosette_context_discovery_build_probe_obj) },
    { MP_ROM_QSTR(MP_QSTR__discovery_parse_reply),   MP_ROM_PTR(&rosette_context_discovery_parse_reply_obj) },
};

static MP_DEFINE_CONST_DICT(rosette_context_locals_dict, rosette_context_locals_dict_table);

}  // namespace rosette::bindings::micropython

extern "C" {

const mp_obj_full_type_t rosette_context_type = {
    { &mp_type_type },
    MP_TYPE_FLAG_NONE,
    MP_QSTR_Context,
    1,  // make_new
    2,  // print
    0,  // call
    0,  // unary_op
    0,  // binary_op
    0,  // attr
    0,  // subscr
    0,  // iter
    0,  // buffer
    0,  // protocol
    0,  // parent
    3,  // locals_dict
    {
        reinterpret_cast<const void*>(rosette::bindings::micropython::ContextMakeNew),
        reinterpret_cast<const void*>(rosette::bindings::micropython::ContextPrint),
        &rosette::bindings::micropython::rosette_context_locals_dict,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
    },
};

}  // extern "C"
