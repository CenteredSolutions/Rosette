/**
 * @file bindings/micropython/rosette_mpy_transport.cpp
 * @brief `_rosette.Transport` C base type and CustomTransport adapter.
 *
 * The C base type is minimal: it holds flags (socket_ready, connected) and
 * the implementation pointer to the C++ CustomTransport adapter. All I/O is
 * implemented in the Python subclass. The C++ adapter dispatches write/read
 * calls to the Python methods (send, recv) via mp_load_attr at call time.
 *
 * Discovery helpers (build_probe, parse_reply) live here because they are
 * transport-layer concerns: they produce/consume XRCE packets on the wire.
 */

#include "rosette_mpy_backend.hpp"
#include "rosette/ros/micro_ros/custom_transport.hpp"
#include "rosette/ros/micro_ros/multicast_agent_locator.hpp"

#include <cstring>
#include <cstdlib>
#include <new>

extern "C" {
#include "py/nlr.h"
#include "py/runtime.h"
#include "py/stream.h"
#include "py/mperrno.h"
#include "stdio.h"
}

namespace rosette::bindings::micropython {

namespace {

using ::milkros::micro_ros::CustomTransport;

// ---------------------------------------------------------------------------
// Helper: call a named method on a Python object
// ---------------------------------------------------------------------------

static mp_obj_t CallMethod1(mp_obj_t obj, qstr name, mp_obj_t arg)
{
    return mp_call_function_1(mp_load_attr(obj, name), arg);
}

static mp_obj_t CallMethod2(mp_obj_t obj, qstr name, mp_obj_t a, mp_obj_t b)
{
    return mp_call_function_2(mp_load_attr(obj, name), a, b);
}

static mp_obj_t CallMethod0(mp_obj_t obj, qstr name)
{
    return mp_call_function_0(mp_load_attr(obj, name));
}

// ---------------------------------------------------------------------------
// Helper: extract OSError errno from an NLR-caught exception
// ---------------------------------------------------------------------------

static int GetOSErrorCode(mp_obj_t exception)
{
    const auto* exc_base = static_cast<mp_obj_base_t*>(MP_OBJ_TO_PTR(exception));
    if (!mp_obj_is_subclass_fast(MP_OBJ_FROM_PTR(exc_base->type),
                                  MP_OBJ_FROM_PTR(&mp_type_OSError))) {
        return 0;
    }
    mp_obj_t value = mp_obj_exception_get_value(exception);
    if (!mp_obj_is_small_int(value)) {
        return 0;
    }
    return mp_obj_get_int(value);
}

// ---------------------------------------------------------------------------
// MicroPythonSocketTransport — CustomTransport adapter
//
// Dispatches write/read/configure_agent_endpoint/close to the Python
// Transport object (owner_) via dynamic method lookup. open() just checks
// the socket_ready flag on the C struct.
// ---------------------------------------------------------------------------

class MicroPythonSocketTransport final : public CustomTransport {
public:
    explicit MicroPythonSocketTransport(mp_rosette_transport_obj_t* owner) noexcept
        : owner_(owner)
    {}

    bool configure_agent_endpoint(const char* address,
                                  const char* port) noexcept override
    {
        if (owner_ == nullptr || address == nullptr || port == nullptr) {
            return false;
        }
        char* endptr = nullptr;
        const unsigned long port_val = strtoul(port, &endptr, 10);
        if (endptr == port || port_val == 0u || port_val > 65535u) {
            return false;
        }

        nlr_buf_t nlr;
        if (nlr_push(&nlr) == 0) {
            mp_obj_t addr_obj = mp_obj_new_str(address, strlen(address));
            mp_obj_t port_obj = mp_obj_new_int_from_uint(static_cast<uint16_t>(port_val));
            CallMethod2(owner_->python_obj, MP_QSTR_connect, addr_obj, port_obj);
            nlr_pop();
            // connect() succeeded without exception → connected UDP.
            owner_->connected = true;
            return true;
        }
        // connect() raised (EPERM fallback or real error) → sendto mode.
        owner_->connected = false;
        return true;
    }

    bool open() noexcept override
    {
        return owner_ != nullptr && owner_->socket_ready;
    }

    bool close() noexcept override
    {
        if (owner_ == nullptr) {
            return true;
        }
        nlr_buf_t nlr;
        if (nlr_push(&nlr) == 0) {
            CallMethod0(owner_->python_obj, MP_QSTR_close);
            nlr_pop();
        }
        owner_->socket_ready = false;
        owner_->connected    = false;
        return true;
    }

    size_t write(const uint8_t* data,
                 size_t size,
                 uint8_t* error) noexcept override
    {
        if (owner_ == nullptr || !owner_->socket_ready || (data == nullptr && size != 0u)) {
            if (error) *error = 1u;
            return 0u;
        }

        nlr_buf_t nlr;
        if (nlr_push(&nlr) == 0) {
            mp_obj_t payload = mp_obj_new_bytes(data, size);
            mp_obj_t result = CallMethod1(owner_->python_obj, MP_QSTR_send, payload);
            nlr_pop();
            if (error) *error = 0u;
            return static_cast<size_t>(mp_obj_get_int(result));
        }
        if (error) *error = 1u;
        return 0u;
    }

    size_t read(uint8_t* data,
                size_t size,
                int timeout_ms,
                uint8_t* error) noexcept override
    {
        if (owner_ == nullptr || !owner_->socket_ready || data == nullptr) {
            if (error) *error = 1u;
            return 0u;
        }
        (void)timeout_ms;

        nlr_buf_t nlr;
        if (nlr_push(&nlr) == 0) {
            mp_obj_t result = CallMethod1(owner_->python_obj, MP_QSTR_recv,
                                          mp_obj_new_int_from_uint(size));
            nlr_pop();
            if (result == mp_const_none) {
                if (error) *error = 1u;
                return 0u;
            }
            mp_buffer_info_t info;
            mp_get_buffer_raise(result, &info, MP_BUFFER_READ);
            const size_t n = info.len < size ? info.len : size;
            memcpy(data, info.buf, n);
            if (error) *error = 0u;
            return n;
        }

        mp_obj_t exception = MP_OBJ_FROM_PTR(nlr.ret_val);
        const int err = GetOSErrorCode(exception);
        if (err == MP_EAGAIN || err == MP_EWOULDBLOCK || err == MP_ETIMEDOUT) {
            if (error) *error = 1u;
            return 0u;
        }
        // Re-raise unexpected errors by returning error — C++ noexcept prevents throw.
        if (error) *error = 1u;
        return 0u;
    }

    mp_rosette_transport_obj_t* owner_ = nullptr;
};

// ---------------------------------------------------------------------------
// Transport MakeNew
// ---------------------------------------------------------------------------

mp_obj_t TransportMakeNew(const mp_obj_type_t* type,
                          size_t n_args,
                          size_t n_kw,
                          const mp_obj_t* args)
{
    (void)n_args; (void)n_kw; (void)args;
    auto* self = mp_obj_malloc(mp_rosette_transport_obj_t, type);
    self->python_obj = MP_OBJ_FROM_PTR(self);
    self->socket_ready = false;
    self->connected    = false;
    self->read_budget_us = 0u;
    self->implementation = nullptr;
    return MP_OBJ_FROM_PTR(self);
}

// ---------------------------------------------------------------------------
// Discovery helpers — build the XRCE GET_INFO probe and parse the reply.
// These live on Transport because they are transport-layer operations.
// ---------------------------------------------------------------------------

mp_obj_t TransportBuildProbeMethod(void)
{
    using namespace milkros::micro_ros::detail;

    uint8_t probe[kGetInfoPacketSize] = {};
    if (!build_get_info_packet(probe)) {
        mp_raise_msg(&mp_type_RuntimeError,
                     MP_ERROR_TEXT("Failed to build XRCE GET_INFO packet"));
    }

    // Return just the probe bytes. The Python layer resolves the multicast
    // address using socket.getaddrinfo() so no socket module import is needed here.
    return mp_obj_new_bytes(probe, sizeof(probe));
}

mp_obj_t TransportParseReplyMethod(mp_obj_t packet_in)
{
    using namespace milkros::micro_ros::detail;

    mp_buffer_info_t info;
    mp_get_buffer_raise(packet_in, &info, MP_BUFFER_READ);

    char ip_str[16]  = {};
    char port_str[8] = {};
    if (!parse_info_packet(static_cast<const uint8_t*>(info.buf),
                           info.len,
                           ip_str, sizeof(ip_str),
                           port_str, sizeof(port_str))) {
        return mp_const_none;
    }

    char* endptr = nullptr;
    const unsigned long port_val = strtoul(port_str, &endptr, 10);
    if (endptr == port_str || port_val == 0u || port_val > 65535u) {
        return mp_const_none;
    }

    mp_obj_t items[2] = {
        mp_obj_new_str(ip_str, strlen(ip_str)),
        mp_obj_new_int_from_uint(static_cast<uint16_t>(port_val)),
    };
    return mp_obj_new_tuple(2, items);
}

}  // namespace

// ---------------------------------------------------------------------------
// TransportInit / TransportFini — allocate/free the C++ adapter
// ---------------------------------------------------------------------------

bool TransportInit(mp_rosette_transport_obj_t* self) noexcept
{
    if (self == nullptr) {
        return false;
    }
    if (self->implementation != nullptr) {
        return true;
    }

    auto* allocd = m_new_maybe(uint8_t, sizeof(MicroPythonSocketTransport));
    if (allocd == nullptr) {
        mp_raise_type(&mp_type_MemoryError);
        return false;
    }
    auto* transport = new (allocd) MicroPythonSocketTransport(self);
    self->implementation = transport;
    self->socket_ready = true;   // transport is wired — Python open() already created the socket
    return true;
}

void TransportFini(mp_rosette_transport_obj_t* self) noexcept
{
    if (self == nullptr || self->implementation == nullptr) {
        return;
    }
    auto* transport = static_cast<MicroPythonSocketTransport*>(self->implementation);
    transport->~MicroPythonSocketTransport();
    m_del(uint8_t, transport, sizeof(MicroPythonSocketTransport));
    self->implementation = nullptr;
}

bool TransportConnected(const mp_rosette_transport_obj_t* self) noexcept
{
    return self != nullptr && self->connected;
}

}  // namespace rosette::bindings::micropython

extern "C" {

mp_obj_t rosette_transport_make_new(const mp_obj_type_t* type,
                                    size_t n_args, size_t n_kw,
                                    const mp_obj_t* args)
    { return rosette::bindings::micropython::TransportMakeNew(type, n_args, n_kw, args); }

mp_obj_t rosette_transport_build_probe()
    { return rosette::bindings::micropython::TransportBuildProbeMethod(); }
mp_obj_t rosette_transport_parse_reply(mp_obj_t p)
    { return rosette::bindings::micropython::TransportParseReplyMethod(p); }

}  // extern "C"
