/**
 * @file bindings/micropython/rosette_mpy_stream.cpp
 * @brief Poll/stream protocol for Subscription and Publisher.
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
    (void)errcode;
    if (request != MP_STREAM_POLL) { *errcode = MP_EINVAL; return MP_STREAM_ERROR; }
    const mp_rosette_subscription_obj_t* self =
        (const mp_rosette_subscription_obj_t*)MP_OBJ_TO_PTR(self_in);
    mp_uint_t ret = 0u;
    if ((arg & MP_STREAM_POLL_RD) &&
        rosette::bindings::micropython::SubscriptionReadable(self))
        ret |= MP_STREAM_POLL_RD;
    return ret;
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
