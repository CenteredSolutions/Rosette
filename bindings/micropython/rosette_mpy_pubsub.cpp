// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file bindings/micropython/rosette_mpy_pubsub.cpp
 * @brief `rosette.Publisher` and `rosette.Subscription` implementation unit.
 *
 * Subscriptions use a pure callback model: received CDR payloads are decoded
 * immediately by SubscriptionDispatchPending() and delivered to the Python
 * callback. Python code may build a queue or asyncio task on top of that
 * callback. There is no C-level ring buffer.
 *
 * Backend helpers (SubscriptionInit/Fini) remain in the
 * rosette::bindings::micropython namespace because backend_runtime.cpp calls
 * them by that name. All Python-facing functions are extern "C" directly.
 */

#include "rosette_mpy_backend.hpp"

extern "C" {
#include "py/runtime.h"
#include "py/stream.h"
}

namespace rosette::bindings::micropython {

bool SubscriptionInit(mp_rosette_subscription_obj_t* self,
                      const EndpointOptions& options) noexcept
{
    if (self == nullptr) return false;
    self->depth    = options.depth;
    self->reliable = options.reliable;
    self->backend  = nullptr;
    return true;
}

void SubscriptionFini(mp_rosette_subscription_obj_t* self) noexcept
{
    if (self == nullptr) return;
    SubscriptionBackendFini(self);
    self->backend = nullptr;
}

}  // namespace rosette::bindings::micropython

// ---------------------------------------------------------------------------
// Python-facing methods — extern "C" implementations (no shim layer).
// ---------------------------------------------------------------------------

extern "C" {

#include "py/runtime.h"

extern const mp_obj_dict_t rosette_publisher_locals_dict;
extern const mp_obj_dict_t rosette_subscription_locals_dict;
extern const mp_stream_p_t rosette_publisher_stream_p;
extern const mp_stream_p_t rosette_subscription_stream_p;

mp_obj_t rosette_publisher_make_new(const mp_obj_type_t* type,
                                    size_t n_args, size_t n_kw,
                                    const mp_obj_t* all_args)
{
    (void)type; (void)n_args; (void)n_kw; (void)all_args;
    mp_raise_TypeError(MP_ERROR_TEXT("Publisher objects are created by Node factory methods"));
}

void rosette_publisher_print(const mp_print_t* print, mp_obj_t self_in, mp_print_kind_t kind)
{
    (void)kind;
    const mp_rosette_publisher_obj_t* self =
        (const mp_rosette_publisher_obj_t*)MP_OBJ_TO_PTR(self_in);
    mp_printf(print, "Publisher(topic=");
    mp_obj_print_helper(print, self->topic_obj, PRINT_STR);
    mp_printf(print, ")");
}

mp_obj_t rosette_publisher_publish(mp_obj_t self_in, mp_obj_t payload)
{
    mp_rosette_publisher_obj_t* self =
        (mp_rosette_publisher_obj_t*)MP_OBJ_TO_PTR(self_in);
    if (!rosette::bindings::micropython::PublisherPublish(self, payload)) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("Rosette publish failed"));
    }
    return mp_const_none;
}

mp_obj_t rosette_subscription_make_new(const mp_obj_type_t* type,
                                       size_t n_args, size_t n_kw,
                                       const mp_obj_t* all_args)
{
    (void)type; (void)n_args; (void)n_kw; (void)all_args;
    mp_raise_TypeError(MP_ERROR_TEXT("Subscription objects are created by Node factory methods"));
}

void rosette_subscription_print(const mp_print_t* print, mp_obj_t self_in, mp_print_kind_t kind)
{
    (void)kind;
    const mp_rosette_subscription_obj_t* self =
        (const mp_rosette_subscription_obj_t*)MP_OBJ_TO_PTR(self_in);
    mp_printf(print, "Subscription(topic=");
    mp_obj_print_helper(print, self->topic_obj, PRINT_STR);
    mp_printf(print, ")");
}

const mp_obj_full_type_t rosette_publisher_type = {
    { &mp_type_type },
    MP_TYPE_FLAG_ITER_IS_STREAM,
    MP_QSTR_Publisher,
    1, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3,
    {
        (const void*)rosette_publisher_make_new,
        (const void*)rosette_publisher_print,
        &rosette_publisher_locals_dict,
        NULL, NULL, NULL, NULL, NULL,
        (const void*)&rosette_publisher_stream_p,
        NULL, NULL,
    },
};

const mp_obj_full_type_t rosette_subscription_type = {
    { &mp_type_type },
    MP_TYPE_FLAG_ITER_IS_STREAM,
    MP_QSTR_Subscription,
    1, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3,
    {
        (const void*)rosette_subscription_make_new,
        (const void*)rosette_subscription_print,
        &rosette_subscription_locals_dict,
        NULL, NULL, NULL, NULL, NULL,
        (const void*)&rosette_subscription_stream_p,
        NULL, NULL,
    },
};

}  // extern "C"
