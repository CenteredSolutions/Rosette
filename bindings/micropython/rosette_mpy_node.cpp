/**
 * @file bindings/micropython/rosette_mpy_node.cpp
 * @brief `rosette.Node` implementation unit.
 *
 * All Python-facing functions are defined as extern "C" directly — there is no
 * separate shim layer.  Backend calls use the rosette::bindings::micropython
 * namespace via a using-directive scoped to each function body.
 */

#include "rosette_mpy_backend.hpp"

extern "C" {
#include "py/runtime.h"

extern const mp_obj_dict_t rosette_node_locals_dict;

void rosette_node_print(const mp_print_t* print, mp_obj_t self_in, mp_print_kind_t kind)
{
    (void)kind;
    const mp_rosette_node_obj_t* self = (const mp_rosette_node_obj_t*)MP_OBJ_TO_PTR(self_in);
    mp_printf(print, "Node(name=");
    mp_obj_print_helper(print, self->name_obj, PRINT_STR);
    mp_printf(print, ", namespace=");
    mp_obj_print_helper(print, self->namespace_obj, PRINT_STR);
    mp_printf(print, ")");
}

mp_obj_t rosette_node_make_new(const mp_obj_type_t* type,
                               size_t n_args, size_t n_kw,
                               const mp_obj_t* all_args)
{
    (void)type; (void)n_args; (void)n_kw; (void)all_args;
    mp_raise_TypeError(MP_ERROR_TEXT("Node objects are created by Context.create_node()"));
}

mp_obj_t rosette_context_create_node(size_t n_args, const mp_obj_t* args)
{
    using namespace rosette::bindings::micropython;
    mp_rosette_context_obj_t* context = (mp_rosette_context_obj_t*)MP_OBJ_TO_PTR(args[0]);
    if (!context->started) {
        mp_raise_msg(&mp_type_RuntimeError,
                     MP_ERROR_TEXT("Context must be started before creating nodes"));
    }

    mp_rosette_node_obj_t* node = mp_obj_malloc(
        mp_rosette_node_obj_t,
        (const mp_obj_type_t*)&rosette_node_type);
    node->context_obj   = args[0];
    node->name_obj      = args[1];
    node->namespace_obj = n_args >= 3 ? args[2] : mp_obj_new_str("", 0);
    node->backend       = NULL;

    if (!NodeInit(node)) {
        mp_raise_msg(&mp_type_RuntimeError,
                     MP_ERROR_TEXT("Rosette node backend init failed"));
    }
    return MP_OBJ_FROM_PTR(node);
}

mp_obj_t rosette_node_create_publisher(size_t n_args, const mp_obj_t* args)
{
    using namespace rosette::bindings::micropython;
    mp_rosette_node_obj_t* node = (mp_rosette_node_obj_t*)MP_OBJ_TO_PTR(args[0]);
    if (node->backend == NULL) {
        mp_raise_msg(&mp_type_RuntimeError,
                     MP_ERROR_TEXT("Node backend is not initialized"));
    }
    if (!mp_obj_is_type(args[1], (const mp_obj_type_t*)&rosette_schema_type)) {
        mp_raise_TypeError(MP_ERROR_TEXT("create_publisher expects a Schema"));
    }

    EndpointOptions options{};
    if (n_args >= 4) options.depth    = (uint16_t)mp_obj_get_int(args[3]);
    if (n_args >= 5) options.reliable = mp_obj_is_true(args[4]);

    mp_rosette_publisher_obj_t* publisher = mp_obj_malloc(
        mp_rosette_publisher_obj_t,
        (const mp_obj_type_t*)&rosette_publisher_type);
    publisher->node_obj   = args[0];
    publisher->schema_obj = args[1];
    publisher->topic_obj  = args[2];
    publisher->depth      = options.depth;
    publisher->reliable   = options.reliable;
    publisher->backend    = NULL;

    if (!PublisherInit(publisher, options)) {
        mp_raise_msg(&mp_type_RuntimeError,
                     MP_ERROR_TEXT("Rosette publisher backend init failed"));
    }
    return MP_OBJ_FROM_PTR(publisher);
}

mp_obj_t rosette_node_create_subscription(size_t n_args, const mp_obj_t* args)
{
    using namespace rosette::bindings::micropython;
    mp_rosette_node_obj_t* node = (mp_rosette_node_obj_t*)MP_OBJ_TO_PTR(args[0]);
    if (node->backend == NULL) {
        mp_raise_msg(&mp_type_RuntimeError,
                     MP_ERROR_TEXT("Node backend is not initialized"));
    }
    if (!mp_obj_is_type(args[1], (const mp_obj_type_t*)&rosette_schema_type)) {
        mp_raise_TypeError(MP_ERROR_TEXT("create_subscription expects a Schema"));
    }

    mp_obj_t callback = mp_const_none;
    EndpointOptions options{};
    if (n_args >= 4) {
        if (args[3] == mp_const_none || mp_obj_is_callable(args[3])) {
            callback = args[3];
            if (n_args >= 5) options.depth    = (uint16_t)mp_obj_get_int(args[4]);
            if (n_args >= 6) options.reliable = mp_obj_is_true(args[5]);
        } else {
            options.depth    = (uint16_t)mp_obj_get_int(args[3]);
            if (n_args >= 5) options.reliable = mp_obj_is_true(args[4]);
        }
    }

    mp_rosette_subscription_obj_t* sub = mp_obj_malloc(
        mp_rosette_subscription_obj_t,
        (const mp_obj_type_t*)&rosette_subscription_type);
    sub->node_obj     = args[0];
    sub->schema_obj   = args[1];
    sub->topic_obj    = args[2];
    sub->callback_obj = callback;
    sub->depth        = options.depth;
    sub->reliable     = options.reliable;
    sub->backend      = NULL;

    if (!SubscriptionInit(sub, options)) {
        mp_raise_msg(&mp_type_RuntimeError,
                     MP_ERROR_TEXT("Rosette subscription init failed"));
    }
    if (!SubscriptionBackendInit(sub, options)) {
        SubscriptionFini(sub);
        mp_raise_msg(&mp_type_RuntimeError,
                     MP_ERROR_TEXT("Rosette subscription backend init failed"));
    }

    mp_rosette_context_obj_t* context_obj =
        (mp_rosette_context_obj_t*)MP_OBJ_TO_PTR(
            ((mp_rosette_node_obj_t*)MP_OBJ_TO_PTR(args[0]))->context_obj);
    mp_obj_list_append(context_obj->subscriptions, MP_OBJ_FROM_PTR(sub));

    return MP_OBJ_FROM_PTR(sub);
}

mp_obj_t rosette_node_name(mp_obj_t self_in)
{
    return ((mp_rosette_node_obj_t*)MP_OBJ_TO_PTR(self_in))->name_obj;
}

mp_obj_t rosette_node_namespace(mp_obj_t self_in)
{
    return ((mp_rosette_node_obj_t*)MP_OBJ_TO_PTR(self_in))->namespace_obj;
}

const mp_obj_full_type_t rosette_node_type = {
    { &mp_type_type },
    MP_TYPE_FLAG_NONE,
    MP_QSTR_Node,
    1,  // make_new
    2,  // print
    0, 0, 0, 0, 0, 0, 0, 0, 0,
    3,  // locals_dict
    {
        (const void*)rosette_node_make_new,
        (const void*)rosette_node_print,
        &rosette_node_locals_dict,
        NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
    },
};

}  // extern "C"
