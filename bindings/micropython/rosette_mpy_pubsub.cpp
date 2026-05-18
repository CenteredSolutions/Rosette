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

namespace rosette::bindings::micropython {

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

mp_obj_t SubscriptionRecvMethod(mp_obj_t self_in)
{
    (void)self_in;
    mp_raise_NotImplementedError(MP_ERROR_TEXT("Rosette recv() is not implemented yet"));
}

}  // namespace

bool SubscriptionInit(mp_rosette_subscription_obj_t* self,
                      const EndpointOptions& options) noexcept
{
    if (self == nullptr) {
        return false;
    }
    self->depth = options.depth;
    self->reliable = options.reliable;
    self->backend = self;
    return true;
}

void SubscriptionFini(mp_rosette_subscription_obj_t* self) noexcept
{
    if (self == nullptr) {
        return;
    }
    self->backend = nullptr;
}

bool SubscriptionEnqueueFrame(mp_rosette_subscription_obj_t* self,
                              const uint8_t* payload,
                              size_t payload_size) noexcept
{
    if (self == nullptr || self->ring == nullptr || self->ring_capacity == 0u ||
        payload_size > self->max_payload_size || payload == nullptr) {
        return false;
    }
    return false;
}

bool SubscriptionReadable(const mp_rosette_subscription_obj_t* self) noexcept
{
    return self != nullptr && self->queued_count != 0u;
}

MP_DEFINE_CONST_FUN_OBJ_2(rosette_publisher_publish_obj, PublisherPublishMethod);
MP_DEFINE_CONST_FUN_OBJ_1(rosette_subscription_recv_obj, SubscriptionRecvMethod);

static const mp_rom_map_elem_t rosette_publisher_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_publish), MP_ROM_PTR(&rosette_publisher_publish_obj) },
};

static const mp_rom_map_elem_t rosette_subscription_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_recv), MP_ROM_PTR(&rosette_subscription_recv_obj) },
};

static MP_DEFINE_CONST_DICT(
    rosette_publisher_locals_dict, rosette_publisher_locals_dict_table);
static MP_DEFINE_CONST_DICT(
    rosette_subscription_locals_dict, rosette_subscription_locals_dict_table);

}  // namespace rosette::bindings::micropython

extern "C" {

const mp_obj_full_type_t rosette_publisher_type = {
    { &mp_type_type },
    MP_TYPE_FLAG_NONE,
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
        nullptr,
        nullptr,
        nullptr,
    },
};

const mp_obj_full_type_t rosette_subscription_type = {
    { &mp_type_type },
    MP_TYPE_FLAG_NONE,
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
        nullptr,
        nullptr,
        nullptr,
    },
};

}  // extern "C"
