/**
 * @file bindings/micropython/rosette_mpy_transport.cpp
 * @brief MicroPython-socket-backed custom transport implementation unit.
 *
 * The transport layer bridges Rosette's micro-ROS custom transport callbacks
 * to a normal MicroPython UDP socket object. Dynamic native modules can use
 * this path without depending on internal NIC structs because the socket
 * object itself already delegates network I/O through the active MicroPython
 * port's networking implementation.
 */

#include "rosette_mpy_backend.hpp"
#include "rosette/ros/micro_ros/custom_transport.hpp"

extern "C" {
#include "py/nlr.h"
#include "py/runtime.h"
#include "py/mperrno.h"
}

namespace rosette::bindings::micropython {

namespace {

using ::milkros::micro_ros::CustomTransport;

/**
 * @brief Resolves one UDP endpoint through the active MicroPython socket module.
 *
 * The transport adapter accepts endpoint updates from Rosette's micro-ROS
 * context when static agent configuration or a locator provides a new address.
 * Resolving through `getaddrinfo()` preserves the active port's preferred
 * sockaddr representation instead of guessing a tuple layout in native code.
 *
 * @param[in] socket_module Imported MicroPython `socket` module object.
 * @param[in] address UTF-8 host string for the remote endpoint.
 * @param[in] port UTF-8 decimal port string for the remote endpoint.
 * @return Socket-address object suitable for `sendto()` and `connect()`.
 *
 * References:
 * - MicroPython socket module documentation: `socket.getaddrinfo()`.
 */
mp_obj_t ResolveAddressObject(mp_obj_t socket_module,
                              const char* address,
                              const char* port)
{
    mp_obj_t getaddrinfo = mp_load_attr(socket_module, MP_QSTR_getaddrinfo);
    mp_obj_t args[4] = {
        mp_obj_new_str(address, strlen(address)),
        mp_obj_new_str(port, strlen(port)),
        mp_load_attr(socket_module, MP_QSTR_AF_INET),
        mp_load_attr(socket_module, MP_QSTR_SOCK_DGRAM),
    };
    mp_obj_t info_list = mp_call_function_n_kw(getaddrinfo, 4, 0, args);
    mp_obj_t first = mp_obj_subscr(info_list, MP_OBJ_NEW_SMALL_INT(0), MP_OBJ_SENTINEL);
    return mp_obj_subscr(first, mp_obj_new_int(-1), MP_OBJ_SENTINEL);
}

int GetOSErrorCode(mp_obj_t exception)
{
    const auto* exc_base = static_cast<mp_obj_base_t*>(MP_OBJ_TO_PTR(exception));
    if (!mp_obj_is_subclass_fast(MP_OBJ_FROM_PTR(exc_base->type), MP_OBJ_FROM_PTR(&mp_type_OSError))) {
        return 0;
    }

    mp_obj_t value = mp_obj_exception_get_value(exception);
    if (!mp_obj_is_small_int(value)) {
        return 0;
    }

    return mp_obj_get_int(value);
}
/**
 * @brief Sends one UDP packet through the context-owned socket.
 *
 * When the constructor succeeded in connecting the socket, the transport can
 * use `send()` directly. If constructor-time `connect()` fell back because of
 * an `EPERM` smoke environment, the transport sends through `sendto()` using
 * the pre-resolved remote address object.
 *
 * @param[in,out] self Context whose socket should transmit the payload.
 * @param[in] payload Python bytes-like object containing the packet payload.
 * @return Python integer number of bytes reported as sent.
 */
mp_obj_t ContextSendPacketMethod(mp_obj_t self_in, mp_obj_t payload)
{
    auto* self = static_cast<mp_rosette_context_obj_t*>(MP_OBJ_TO_PTR(self_in));
    return TransportSendPacket(self, payload);
}

/**
 * @brief Receives one UDP packet from the context-owned socket.
 *
 * The read path mirrors the send path: connected sockets use `recv()`, while
 * unconnected fallback sockets use `recvfrom()` and discard the returned peer
 * address because the binding already tracks the intended remote endpoint.
 *
 * If the socket would block, the method returns `None`.
 *
 * @param[in] self_in Context whose socket should receive from.
 * @param[in] max_bytes_in Maximum packet size to request from the socket.
 * @return Received packet bytes, or `None` when no packet is ready.
 */
mp_obj_t ContextRecvPacketMethod(mp_obj_t self_in, mp_obj_t max_bytes_in)
{
    auto* self = static_cast<mp_rosette_context_obj_t*>(MP_OBJ_TO_PTR(self_in));
    return TransportRecvPacket(self, max_bytes_in);
}

/**
 * @brief Reports whether the context currently treats its UDP socket as connected.
 *
 * Constructor-time `connect()` may have succeeded, or it may have fallen back
 * because of a sandboxed smoke environment. This accessor makes that state
 * visible to Python tests without requiring string parsing of `repr(context)`.
 *
 * @param[in] self_in Context to inspect.
 * @return `True` when the transport socket is connected for `send()` / `recv()`.
 */
mp_obj_t ContextConnectedMethod(mp_obj_t self_in)
{
    const auto* self = static_cast<mp_rosette_context_obj_t*>(MP_OBJ_TO_PTR(self_in));
    return mp_obj_new_bool(TransportConnected(self));
}

/**
 * @brief MicroPython-socket-backed Rosette custom transport adapter.
 *
 * This adapter is the bridge point between the Python-owned UDP socket and the
 * future micro-ROS XRCE session. It implements Rosette's small transport
 * contract now, and the Python-facing packet helpers reuse the same adapter so
 * transport behavior stays consistent between manual smoke tests and backend
 * integration.
 */
class MicroPythonSocketTransport final : public CustomTransport {
public:
    explicit MicroPythonSocketTransport(mp_rosette_context_obj_t* owner) noexcept
        : owner_(owner)
    {}

    /**
     * @brief Updates the transport's remote agent endpoint.
     *
     * Rosette's micro-ROS context calls this before opening a custom
     * transport. The adapter reuses the imported `socket` module to resolve
     * the endpoint into the active port's preferred sockaddr representation
     * and stores that object on the owning context for later `sendto()`
     * operations.
     *
     * @param[in] address UTF-8 host string for the micro-ROS agent.
     * @param[in] port UTF-8 decimal port string for the micro-ROS agent.
     * @return `true` when the endpoint was accepted and resolved.
     *
     * References:
     * - Micro XRCE-DDS client transport configuration model.
     * - MicroPython socket module documentation for `getaddrinfo()`.
     */
    bool configure_agent_endpoint(const char* address,
                                  const char* port) noexcept override
    {
        if (owner_ == nullptr || owner_->socket_module_obj == mp_const_none ||
            address == nullptr || port == nullptr) {
            return false;
        }

        nlr_buf_t nlr;
        if (nlr_push(&nlr) == 0) {
            owner_->remote_addr_obj =
                ResolveAddressObject(owner_->socket_module_obj, address, port);
            nlr_pop();
            return true;
        }
        return false;
    }

    bool open() noexcept override
    {
        return owner_ != nullptr && owner_->transport.socket_ready &&
               owner_->socket_obj != mp_const_none && !owner_->closed;
    }

    bool close() noexcept override
    {
        if (owner_ == nullptr || owner_->closed || owner_->socket_obj == mp_const_none ||
            owner_->meth_close == mp_const_none) {
            return true;
        }
        mp_call_function_0(owner_->meth_close);
        owner_->closed = true;
        owner_->connected = false;
        owner_->transport.socket_ready = false;
        return true;
    }

    size_t write(const uint8_t* data,
                 size_t size,
                 uint8_t* error) noexcept override
    {
        if (owner_ == nullptr || !open() || (data == nullptr && size != 0u)) {
            if (error != nullptr) {
                *error = 1u;
            }
            return 0u;
        }

        mp_obj_t payload = mp_obj_new_bytes(data, size);
        mp_obj_t result = TransportSendPacket(owner_, payload);
        if (error != nullptr) {
            *error = 0u;
        }
        return static_cast<size_t>(mp_obj_get_int(result));
    }

    size_t read(uint8_t* data,
                size_t size,
                int timeout_ms,
                uint8_t* error) noexcept override
    {
        if (owner_ == nullptr || !open() || data == nullptr) {
            if (error != nullptr) {
                *error = 1u;
            }
            return 0u;
        }

        (void)timeout_ms;
        mp_obj_t result = TransportRecvPacket(owner_, mp_obj_new_int_from_uint(size));
        if (result == mp_const_none) {
            if (error != nullptr) {
                *error = 1u;
            }
            return 0u;
        }

        mp_buffer_info_t info;
        mp_get_buffer_raise(result, &info, MP_BUFFER_READ);
        const size_t copy_size = info.len < size ? info.len : size;
        memcpy(data, info.buf, copy_size);
        if (error != nullptr) {
            *error = 0u;
        }
        return copy_size;
    }

private:
    mp_rosette_context_obj_t* owner_ = nullptr;
};

}  // namespace

bool TransportInit(mp_rosette_context_obj_t* self) noexcept
{
    if (self == nullptr) {
        return false;
    }
    if (self->transport.implementation != nullptr) {
        return true;
    }
    auto* transport = new MicroPythonSocketTransport(self);
    self->transport.implementation = transport;
    return true;
}

void TransportFini(mp_rosette_context_obj_t* self) noexcept
{
    if (self == nullptr || self->transport.implementation == nullptr) {
        return;
    }
    auto* transport =
        static_cast<MicroPythonSocketTransport*>(self->transport.implementation);
    delete transport;
    self->transport.implementation = nullptr;
}

bool TransportConnected(const mp_rosette_context_obj_t* self) noexcept
{
    return self != nullptr && self->connected;
}

mp_obj_t TransportSendPacket(mp_rosette_context_obj_t* self,
                             mp_obj_t payload)
{
    if (self == nullptr || self->socket_obj == mp_const_none || self->closed) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("Context socket is closed"));
    }

    if (self->connected) {
        return mp_call_function_1(self->meth_send, payload);
    }

    return mp_call_function_2(self->meth_sendto, payload, self->remote_addr_obj);
}

mp_obj_t TransportRecvPacket(mp_rosette_context_obj_t* self,
                             mp_obj_t max_bytes_in)
{
    if (self == nullptr || self->socket_obj == mp_const_none || self->closed) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("Context socket is closed"));
    }

    nlr_buf_t nlr;
    if (nlr_push(&nlr) == 0) {
        mp_obj_t result = mp_const_none;
        if (self->connected) {
            result = mp_call_function_1(self->meth_recv, max_bytes_in);
        } else {
            mp_obj_t tuple = mp_call_function_1(self->meth_recvfrom, max_bytes_in);
            result = mp_obj_subscr(tuple, MP_OBJ_NEW_SMALL_INT(0), MP_OBJ_SENTINEL);
        }
        nlr_pop();
        return result;
    }

    mp_obj_t exception = MP_OBJ_FROM_PTR(nlr.ret_val);
    const int err = GetOSErrorCode(exception);
    if (err == MP_EAGAIN || err == MP_EWOULDBLOCK || err == MP_ETIMEDOUT) {
        return mp_const_none;
    }
    if (err == MP_EPERM && self->connect_fallback_eperm && !self->connected) {
        return mp_const_none;
    }
    nlr_raise(exception);
}

extern const mp_obj_fun_builtin_fixed_t rosette_context_connected_obj = {
    .base = { &mp_type_fun_builtin_1 },
    .fun = { ._1 = ContextConnectedMethod },
};

extern const mp_obj_fun_builtin_fixed_t rosette_context_send_packet_obj = {
    .base = { &mp_type_fun_builtin_2 },
    .fun = { ._2 = ContextSendPacketMethod },
};

extern const mp_obj_fun_builtin_var_t rosette_context_recv_packet_obj = {
    .base = { &mp_type_fun_builtin_var },
    .sig = MP_OBJ_FUN_MAKE_SIG(1, 2, false),
    .fun = { .var = [](size_t n_args, const mp_obj_t* args) -> mp_obj_t {
        mp_obj_t max_bytes = n_args >= 2 ? args[1] : MP_OBJ_NEW_SMALL_INT(2048);
        return ContextRecvPacketMethod(args[0], max_bytes);
    } },
};

}  // namespace rosette::bindings::micropython
