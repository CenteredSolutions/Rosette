/**
 * @file bindings/micropython/rosette_mpy_stream.cpp
 * @brief Poll/stream protocol for Subscription and Publisher.
 *
 * Subscriptions use a callback model: the cooperative pump delivers messages
 * immediately via Python callbacks. There is no C-level ring buffer or readiness
 * flag to poll, so subscription poll always returns not-readable.
 */

#include "rosette_mpy_backend.hpp"

extern "C" {
#include "py/stream.h"

mp_uint_t rosette_subscription_read(mp_obj_t o, void* buf, mp_uint_t n, int* e)
    { (void)o; (void)buf; (void)n; *e = MP_EPERM; return MP_STREAM_ERROR; }
mp_uint_t rosette_subscription_write(mp_obj_t o, const void* buf, mp_uint_t n, int* e)
    { (void)o; (void)buf; (void)n; *e = MP_EPERM; return MP_STREAM_ERROR; }
mp_uint_t rosette_subscription_ioctl(mp_obj_t self_in, mp_uint_t request,
                                      uintptr_t arg, int* errcode)
{
    (void)self_in; (void)arg; (void)errcode;
    if (request != MP_STREAM_POLL) { *errcode = MP_EINVAL; return MP_STREAM_ERROR; }
    /* Callback-only model: no buffered data to poll. */
    return 0u;
}

mp_uint_t rosette_publisher_read(mp_obj_t o, void* buf, mp_uint_t n, int* e)
    { (void)o; (void)buf; (void)n; *e = MP_EPERM; return MP_STREAM_ERROR; }
mp_uint_t rosette_publisher_write(mp_obj_t o, const void* buf, mp_uint_t n, int* e)
    { (void)o; (void)buf; (void)n; *e = MP_EPERM; return MP_STREAM_ERROR; }
mp_uint_t rosette_publisher_ioctl(mp_obj_t self_in, mp_uint_t request,
                                   uintptr_t arg, int* errcode)
{
    (void)errcode; (void)self_in;
    if (request != MP_STREAM_POLL) { *errcode = MP_EINVAL; return MP_STREAM_ERROR; }
    mp_uint_t ret = 0u;
    if (arg & MP_STREAM_POLL_WR) ret |= MP_STREAM_POLL_WR;
    return ret;
}

}  // extern "C"
