/**
 * @file bindings/micropython/rosette_mpy_stream.cpp
 * @brief Poll/stream protocol for Subscription and Publisher.
 *
 * Implements `mp_stream_p_t` so both types participate in `select.poll()` and
 * MicroPython's asyncio event loop via `stream.queue_read` / `stream.queue_write`.
 *
 * Subscription reports MP_STREAM_POLL_RD when its ring buffer holds at least one
 * frame. Publisher reports MP_STREAM_POLL_WR unconditionally for best-effort
 * endpoints; for reliable endpoints it reports writable when the underlying
 * backend's send queue has room (currently modelled as always-ready — the
 * reliable backpressure path can be wired in later via PublisherWritable()).
 *
 * Neither type implements read() / write() — those operations are handled by
 * the typed recv() / publish() methods. ioctl is the only entry point used here.
 */

#include "rosette_mpy_backend.hpp"

extern "C" {
#include "py/stream.h"
}

namespace rosette::bindings::micropython {

namespace {

// ---------------------------------------------------------------------------
// Subscription stream protocol
// ---------------------------------------------------------------------------

mp_uint_t SubscriptionRead(mp_obj_t, void*, mp_uint_t, int* errcode)
{
    *errcode = MP_EPERM;
    return MP_STREAM_ERROR;
}

mp_uint_t SubscriptionWrite(mp_obj_t, const void*, mp_uint_t, int* errcode)
{
    *errcode = MP_EPERM;
    return MP_STREAM_ERROR;
}

mp_uint_t SubscriptionIoctl(mp_obj_t self_in, mp_uint_t request,
                             uintptr_t arg, int* errcode)
{
    (void)errcode;
    if (request != MP_STREAM_POLL) {
        *errcode = MP_EINVAL;
        return MP_STREAM_ERROR;
    }
    const auto* self =
        static_cast<const mp_rosette_subscription_obj_t*>(MP_OBJ_TO_PTR(self_in));
    mp_uint_t ret = 0u;
    if ((arg & MP_STREAM_POLL_RD) && SubscriptionReadable(self)) {
        ret |= MP_STREAM_POLL_RD;
    }
    return ret;
}

// ---------------------------------------------------------------------------
// Publisher stream protocol
// ---------------------------------------------------------------------------

mp_uint_t PublisherRead(mp_obj_t, void*, mp_uint_t, int* errcode)
{
    *errcode = MP_EPERM;
    return MP_STREAM_ERROR;
}

mp_uint_t PublisherWrite(mp_obj_t, const void*, mp_uint_t, int* errcode)
{
    *errcode = MP_EPERM;
    return MP_STREAM_ERROR;
}

mp_uint_t PublisherIoctl(mp_obj_t self_in, mp_uint_t request,
                          uintptr_t arg, int* errcode)
{
    (void)errcode;
    if (request != MP_STREAM_POLL) {
        *errcode = MP_EINVAL;
        return MP_STREAM_ERROR;
    }
    const auto* self =
        static_cast<const mp_rosette_publisher_obj_t*>(MP_OBJ_TO_PTR(self_in));
    (void)self;
    mp_uint_t ret = 0u;
    // Always writable — backpressure for reliable publishers can be added when
    // PublisherWritable() exists in the backend.
    if (arg & MP_STREAM_POLL_WR) {
        ret |= MP_STREAM_POLL_WR;
    }
    return ret;
}

}  // namespace

// Exported so the type table entries in rosette_mpy_pubsub.cpp can reference
// them as extern C symbols.
const mp_stream_p_t rosette_subscription_stream_p = {
    .read  = SubscriptionRead,
    .write = SubscriptionWrite,
    .ioctl = SubscriptionIoctl,
};

const mp_stream_p_t rosette_publisher_stream_p = {
    .read  = PublisherRead,
    .write = PublisherWrite,
    .ioctl = PublisherIoctl,
};

}  // namespace rosette::bindings::micropython
