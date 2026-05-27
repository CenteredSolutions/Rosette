/**
 * @file bindings/micropython/rosette_mpy_pubsub.cpp
 * @brief `rosette.Publisher` and `rosette.Subscription` implementation unit.
 */

#include "rosette_mpy_backend.hpp"

extern "C" {
#include "py/runtime.h"
#include "py/stream.h"
}

#include <cstring>

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
    if (self->ring != nullptr) {
        for (size_t i = 0u; i < self->ring_capacity; ++i) {
            if (self->ring[i].data != nullptr) {
                m_del(uint8_t, self->ring[i].data, self->max_payload_size);
                self->ring[i].data = nullptr;
            }
        }
        m_del(rosette_mpy_frame_t, self->ring, self->ring_capacity);
        self->ring = nullptr;
    }
    self->ring_capacity = 0u;
    self->ring_head     = 0u;
    self->ring_tail     = 0u;
    self->queued_count  = 0u;
    self->backend       = nullptr;
}

static bool EnsureRingAllocated(mp_rosette_subscription_obj_t* self) noexcept
{
    if (self->ring != nullptr) return true;
    const size_t capacity = (self->depth > 0u) ? (size_t)self->depth : 4u;
    self->ring = m_new(rosette_mpy_frame_t, capacity);
    if (self->ring == nullptr) return false;
    for (size_t i = 0u; i < capacity; ++i) {
        self->ring[i].data     = m_new(uint8_t, self->max_payload_size);
        self->ring[i].length   = 0u;
        self->ring[i].occupied = false;
        if (self->ring[i].data == nullptr) {
            for (size_t j = 0u; j < i; ++j)
                m_del(uint8_t, self->ring[j].data, self->max_payload_size);
            m_del(rosette_mpy_frame_t, self->ring, capacity);
            self->ring = nullptr;
            return false;
        }
    }
    self->ring_capacity = capacity;
    self->ring_head     = 0u;
    self->ring_tail     = 0u;
    self->queued_count  = 0u;
    return true;
}

bool SubscriptionEnqueueFrame(mp_rosette_subscription_obj_t* self,
                              const uint8_t* payload,
                              size_t payload_size) noexcept
{
    if (self == nullptr || payload == nullptr) return false;
    if (payload_size > self->max_payload_size || self->max_payload_size == 0u) return false;
    if (!EnsureRingAllocated(self)) return false;
    if (self->queued_count >= self->ring_capacity) {
        self->ring_head = (self->ring_head + 1u) % self->ring_capacity;
        --self->queued_count;
    }
    rosette_mpy_frame_t* slot = &self->ring[self->ring_tail];
    std::memcpy(slot->data, payload, payload_size);
    slot->length   = payload_size;
    slot->occupied = true;
    self->ring_tail = (self->ring_tail + 1u) % self->ring_capacity;
    ++self->queued_count;
    return true;
}

bool SubscriptionReadable(const mp_rosette_subscription_obj_t* self) noexcept
{
    return self != nullptr && self->queued_count != 0u;
}

}  // namespace rosette::bindings::micropython

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
    mp_printf(print, ", queued=%u)", (unsigned)self->queued_count);
}

mp_obj_t rosette_subscription_recv(mp_obj_t self_in)
{
    mp_rosette_subscription_obj_t* self =
        (mp_rosette_subscription_obj_t*)MP_OBJ_TO_PTR(self_in);
    if (self->ring == NULL || self->queued_count == 0u) return mp_const_none;
    rosette_mpy_frame_t* slot = &self->ring[self->ring_head];
    if (!slot->occupied || slot->length == 0u) return mp_const_none;

    const mp_rosette_schema_obj_t* schema =
        (const mp_rosette_schema_obj_t*)MP_OBJ_TO_PTR(self->schema_obj);
    mp_obj_t result = rosette::bindings::micropython::SchemaCdrDecode(
        schema, slot->data, slot->length);

    slot->occupied = false;
    slot->length   = 0u;
    self->ring_head = (self->ring_head + 1u) % self->ring_capacity;
    --self->queued_count;
    return result;
}

mp_obj_t rosette_subscription_queued_count(mp_obj_t self_in)
{
    const mp_rosette_subscription_obj_t* self =
        (const mp_rosette_subscription_obj_t*)MP_OBJ_TO_PTR(self_in);
    return MP_OBJ_NEW_SMALL_INT((mp_int_t)self->queued_count);
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
