/**
 * @file bindings/micropython/rosette_mpy_pubsub.cpp
 * @brief `rosette.Publisher` and `rosette.Subscription` implementation unit.
 *
 * Publishers and subscriptions translate between the Python-facing runtime
 * schema objects and Rosette's backend handles. The publish path encodes
 * Python dictionaries into CDR payloads, while the subscription path stores
 * received payload bytes in a bounded ring buffer and decodes them on demand.
 */

#include "rosette_mpy_backend.hpp"

extern "C" {
#include "py/runtime.h"
}

#include <cstring>

namespace rosette::bindings::micropython {

// Defined in rosette_mpy_stream.cpp.
extern const mp_stream_p_t rosette_subscription_stream_p;
extern const mp_stream_p_t rosette_publisher_stream_p;

namespace {

[[noreturn]] void RaiseFactoryOnly()
{
    mp_raise_TypeError(MP_ERROR_TEXT("Publisher and Subscription objects are created by Node factory methods"));
}

void PublisherPrint(const mp_print_t* print,
                    mp_obj_t self_in,
                    mp_print_kind_t kind)
{
    (void)kind;
    const auto* self = static_cast<mp_rosette_publisher_obj_t*>(MP_OBJ_TO_PTR(self_in));
    mp_printf(print, "Publisher(topic=");
    mp_obj_print_helper(print, self->topic_obj, PRINT_STR);
    mp_printf(print, ")");
}

void SubscriptionPrint(const mp_print_t* print,
                       mp_obj_t self_in,
                       mp_print_kind_t kind)
{
    (void)kind;
    const auto* self = static_cast<mp_rosette_subscription_obj_t*>(MP_OBJ_TO_PTR(self_in));
    mp_printf(print, "Subscription(topic=");
    mp_obj_print_helper(print, self->topic_obj, PRINT_STR);
    mp_printf(print, ", queued=%u)", static_cast<unsigned>(self->queued_count));
}

mp_obj_t PublisherMakeNew(const mp_obj_type_t* type,
                          size_t n_args,
                          size_t n_kw,
                          const mp_obj_t* all_args)
{
    (void)type;
    (void)n_args;
    (void)n_kw;
    (void)all_args;
    RaiseFactoryOnly();
}

mp_obj_t SubscriptionMakeNew(const mp_obj_type_t* type,
                             size_t n_args,
                             size_t n_kw,
                             const mp_obj_t* all_args)
{
    (void)type;
    (void)n_args;
    (void)n_kw;
    (void)all_args;
    RaiseFactoryOnly();
}

mp_obj_t PublisherPublishMethod(mp_obj_t self_in, mp_obj_t payload)
{
    auto* self = static_cast<mp_rosette_publisher_obj_t*>(MP_OBJ_TO_PTR(self_in));
    if (!PublisherPublish(self, payload)) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("Rosette publish failed"));
    }
    return mp_const_none;
}

/**
 * @brief Pops the oldest received frame and decodes it into a Message object.
 *
 * Returns None when the ring is empty.  The CDR decode is schema-driven so
 * no generated C type structs are needed — any schema parsed at runtime from
 * `.msg` text works equally well.
 *
 * @param[in] self_in Subscription object.
 * @return Decoded Message, or None when no frame is available.
 */
mp_obj_t SubscriptionRecvMethod(mp_obj_t self_in)
{
    auto* self = static_cast<mp_rosette_subscription_obj_t*>(MP_OBJ_TO_PTR(self_in));

    if (self->ring == nullptr || self->queued_count == 0u) {
        return mp_const_none;
    }

    rosette_mpy_frame_t* slot = &self->ring[self->ring_head];
    if (!slot->occupied || slot->length == 0u) {
        return mp_const_none;
    }

    const auto* schema =
        static_cast<const mp_rosette_schema_obj_t*>(MP_OBJ_TO_PTR(self->schema_obj));

    mp_obj_t result = SchemaCdrDecode(schema, slot->data, slot->length);

    // Advance the ring regardless of decode success so a malformed frame
    // does not permanently stall the subscription.
    slot->occupied = false;
    slot->length   = 0u;
    self->ring_head = (self->ring_head + 1u) % self->ring_capacity;
    --self->queued_count;

    return result;
}

}  // namespace

bool SubscriptionInit(mp_rosette_subscription_obj_t* self,
                      const EndpointOptions& options) noexcept
{
    if (self == nullptr) {
        return false;
    }
    self->depth    = options.depth;
    self->reliable = options.reliable;
    self->backend  = self;
    return true;
}

void SubscriptionFini(mp_rosette_subscription_obj_t* self) noexcept
{
    if (self == nullptr) {
        return;
    }
    // Release ring frame buffers if any were allocated.
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

/**
 * @brief Lazily allocates the ring buffer the first time a frame arrives.
 *
 * The depth and max_payload_size are known after SubscriptionInit, but the
 * frame buffers are deferred to first use so that subscriptions that never
 * receive data incur no heap cost.
 *
 * @param[in,out] self Subscription whose ring should be allocated.
 * @return true when the ring is ready.
 */
bool EnsureRingAllocated(mp_rosette_subscription_obj_t* self) noexcept
{
    if (self->ring != nullptr) {
        return true;
    }
    const size_t capacity = (self->depth > 0u) ? static_cast<size_t>(self->depth) : 4u;
    self->ring = m_new(rosette_mpy_frame_t, capacity);
    if (self->ring == nullptr) {
        return false;
    }
    for (size_t i = 0u; i < capacity; ++i) {
        self->ring[i].data     = m_new(uint8_t, self->max_payload_size);
        self->ring[i].length   = 0u;
        self->ring[i].occupied = false;
        if (self->ring[i].data == nullptr) {
            // Partial allocation — release what we got.
            for (size_t j = 0u; j < i; ++j) {
                m_del(uint8_t, self->ring[j].data, self->max_payload_size);
            }
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
    if (self == nullptr || payload == nullptr) {
        return false;
    }
    if (payload_size > self->max_payload_size || self->max_payload_size == 0u) {
        return false;
    }
    if (!EnsureRingAllocated(self)) {
        return false;
    }
    if (self->queued_count >= self->ring_capacity) {
        // Ring is full: drop the oldest frame (overwrite head).
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

mp_obj_t SubscriptionQueuedCountMethod(mp_obj_t self_in)
{
    const auto* self =
        static_cast<const mp_rosette_subscription_obj_t*>(MP_OBJ_TO_PTR(self_in));
    return MP_OBJ_NEW_SMALL_INT(static_cast<mp_int_t>(self->queued_count));
}

MP_DEFINE_CONST_FUN_OBJ_2(rosette_publisher_publish_obj, PublisherPublishMethod);
MP_DEFINE_CONST_FUN_OBJ_1(rosette_subscription_recv_obj, SubscriptionRecvMethod);
MP_DEFINE_CONST_FUN_OBJ_1(rosette_subscription_queued_count_obj, SubscriptionQueuedCountMethod);

static const mp_rom_map_elem_t rosette_publisher_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_publish), MP_ROM_PTR(&rosette_publisher_publish_obj) },
};

static const mp_rom_map_elem_t rosette_subscription_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_recv),         MP_ROM_PTR(&rosette_subscription_recv_obj) },
    { MP_ROM_QSTR(MP_QSTR_queued_count), MP_ROM_PTR(&rosette_subscription_queued_count_obj) },
};

static MP_DEFINE_CONST_DICT(
    rosette_publisher_locals_dict, rosette_publisher_locals_dict_table);
static MP_DEFINE_CONST_DICT(
    rosette_subscription_locals_dict, rosette_subscription_locals_dict_table);

}  // namespace rosette::bindings::micropython

extern "C" {

const mp_obj_full_type_t rosette_publisher_type = {
    { &mp_type_type },
    MP_TYPE_FLAG_ITER_IS_STREAM,
    MP_QSTR_Publisher,
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
        reinterpret_cast<const void*>(rosette::bindings::micropython::PublisherMakeNew),
        reinterpret_cast<const void*>(rosette::bindings::micropython::PublisherPrint),
        &rosette::bindings::micropython::rosette_publisher_locals_dict,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        reinterpret_cast<const void*>(&rosette::bindings::micropython::rosette_publisher_stream_p),
        nullptr,
        nullptr,
    },
};

const mp_obj_full_type_t rosette_subscription_type = {
    { &mp_type_type },
    MP_TYPE_FLAG_ITER_IS_STREAM,
    MP_QSTR_Subscription,
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
        reinterpret_cast<const void*>(rosette::bindings::micropython::SubscriptionMakeNew),
        reinterpret_cast<const void*>(rosette::bindings::micropython::SubscriptionPrint),
        &rosette::bindings::micropython::rosette_subscription_locals_dict,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        reinterpret_cast<const void*>(&rosette::bindings::micropython::rosette_subscription_stream_p),
        nullptr,
        nullptr,
    },
};

}  // extern "C"
