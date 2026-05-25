/**
 * @file bindings/micropython/rosette_mpy_node.cpp
 * @brief `rosette.Node` implementation unit.
 *
 * Nodes are lightweight naming wrappers over a started context. This unit owns
 * the node constructor, node finalization, and the factory surface that
 * creates publishers and subscriptions with topic names relative to the node's
 * namespace.
 */

#include "rosette_mpy_backend.hpp"

extern "C" {
#include "py/runtime.h"
}

namespace rosette::bindings::micropython {

namespace {

/**
 * @brief Displays a compact representation of a node.
 *
 * @param[in] print MicroPython print destination.
 * @param[in] self_in Node object to display.
 * @param[in] kind Print behavior requested by MicroPython.
 */
void NodePrint(const mp_print_t* print,
               mp_obj_t self_in,
               mp_print_kind_t kind)
{
    (void)kind;
    const auto* self = static_cast<mp_rosette_node_obj_t*>(MP_OBJ_TO_PTR(self_in));
    mp_printf(print, "Node(name=");
    mp_obj_print_helper(print, self->name_obj, PRINT_STR);
    mp_printf(print, ", namespace=");
    mp_obj_print_helper(print, self->namespace_obj, PRINT_STR);
    mp_printf(print, ")");
}

/**
 * @brief Rejects direct `Node(...)` construction from Python.
 *
 * Nodes are created from a started context so they inherit the context-owned
 * backend and transport state.
 */
mp_obj_t NodeMakeNew(const mp_obj_type_t* type,
                     size_t n_args,
                     size_t n_kw,
                     const mp_obj_t* all_args)
{
    (void)type;
    (void)n_args;
    (void)n_kw;
    (void)all_args;
    mp_raise_TypeError(MP_ERROR_TEXT("Node objects are created by Context.create_node()"));
}

/**
 * @brief Creates a node owned by one started context.
 *
 * @param[in] self_in Parent `Context`.
 * @param[in] name_obj Node name.
 * @param[in] namespace_obj Optional ROS namespace.
 * @return Newly allocated `Node` object.
 */
mp_obj_t ContextCreateNode(size_t n_args, const mp_obj_t* args)
{
    auto* context = static_cast<mp_rosette_context_obj_t*>(MP_OBJ_TO_PTR(args[0]));
    if (!context->started) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("Context must be started before creating nodes"));
    }

    auto* node = mp_obj_malloc(
        mp_rosette_node_obj_t,
        reinterpret_cast<const mp_obj_type_t*>(&rosette_node_type));
    node->context_obj = args[0];
    node->name_obj = args[1];
    node->namespace_obj = n_args >= 3 ? args[2] : mp_obj_new_str("", 0);
    node->backend = nullptr;

    if (!NodeInit(node)) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("Rosette node backend init failed"));
    }
    return MP_OBJ_FROM_PTR(node);
}

mp_obj_t NodeCreatePublisher(size_t n_args, const mp_obj_t* args)
{
    auto* node = static_cast<mp_rosette_node_obj_t*>(MP_OBJ_TO_PTR(args[0]));
    if (node->backend == nullptr) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("Node backend is not initialized"));
    }
    if (!mp_obj_is_type(args[1], reinterpret_cast<const mp_obj_type_t*>(&rosette_schema_type))) {
        mp_raise_TypeError(MP_ERROR_TEXT("create_publisher expects a Schema"));
    }

    EndpointOptions options{};
    if (n_args >= 4) {
        options.depth = static_cast<uint16_t>(mp_obj_get_int(args[3]));
    }
    if (n_args >= 5) {
        options.reliable = mp_obj_is_true(args[4]);
    }

    auto* publisher = mp_obj_malloc(
        mp_rosette_publisher_obj_t,
        reinterpret_cast<const mp_obj_type_t*>(&rosette_publisher_type));
    publisher->node_obj = args[0];
    publisher->schema_obj = args[1];
    publisher->topic_obj = args[2];
    publisher->depth = options.depth;
    publisher->reliable = options.reliable;
    publisher->backend = nullptr;

    if (!PublisherInit(publisher, options)) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("Rosette publisher backend init failed"));
    }
    return MP_OBJ_FROM_PTR(publisher);
}

mp_obj_t NodeCreateSubscription(size_t n_args, const mp_obj_t* args)
{
    auto* node = static_cast<mp_rosette_node_obj_t*>(MP_OBJ_TO_PTR(args[0]));
    if (node->backend == nullptr) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("Node backend is not initialized"));
    }
    if (!mp_obj_is_type(args[1], reinterpret_cast<const mp_obj_type_t*>(&rosette_schema_type))) {
        mp_raise_TypeError(MP_ERROR_TEXT("create_subscription expects a Schema"));
    }

    EndpointOptions options{};
    if (n_args >= 4) {
        options.depth = static_cast<uint16_t>(mp_obj_get_int(args[3]));
    }
    if (n_args >= 5) {
        options.reliable = mp_obj_is_true(args[4]);
    }

    auto* sub = mp_obj_malloc(
        mp_rosette_subscription_obj_t,
        reinterpret_cast<const mp_obj_type_t*>(&rosette_subscription_type));
    sub->node_obj   = args[0];
    sub->schema_obj = args[1];
    sub->topic_obj  = args[2];
    sub->depth      = options.depth;
    sub->reliable   = options.reliable;
    sub->ring       = nullptr;
    sub->ring_capacity = 0u;
    sub->ring_head     = 0u;
    sub->ring_tail     = 0u;
    sub->queued_count  = 0u;
    sub->max_payload_size = 512u; // default; schema can refine this
    sub->backend    = nullptr;

    if (!SubscriptionInit(sub, options)) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("Rosette subscription backend init failed"));
    }

    // Register with the context subscription list so tick() delivers frames.
    auto* context_obj = static_cast<mp_rosette_context_obj_t*>(
        MP_OBJ_TO_PTR(static_cast<mp_rosette_node_obj_t*>(MP_OBJ_TO_PTR(args[0]))->context_obj));
    mp_obj_list_append(context_obj->subscriptions, MP_OBJ_FROM_PTR(sub));

    return MP_OBJ_FROM_PTR(sub);
}

mp_obj_t NodeName(mp_obj_t self_in)
{
    auto* self = static_cast<mp_rosette_node_obj_t*>(MP_OBJ_TO_PTR(self_in));
    return self->name_obj;
}

mp_obj_t NodeNamespace(mp_obj_t self_in)
{
    auto* self = static_cast<mp_rosette_node_obj_t*>(MP_OBJ_TO_PTR(self_in));
    return self->namespace_obj;
}

}  // namespace

extern const mp_obj_fun_builtin_var_t rosette_context_create_node_obj = {
    .base = { &mp_type_fun_builtin_var },
    .sig = MP_OBJ_FUN_MAKE_SIG(2, 3, false),
    .fun = { .var = ContextCreateNode },
};
MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(
    rosette_node_create_publisher_obj, 3, 5, NodeCreatePublisher);
MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(
    rosette_node_create_subscription_obj, 3, 6, NodeCreateSubscription);
MP_DEFINE_CONST_FUN_OBJ_1(rosette_node_name_obj, NodeName);
MP_DEFINE_CONST_FUN_OBJ_1(rosette_node_namespace_obj, NodeNamespace);

static const mp_rom_map_elem_t rosette_node_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_create_publisher), MP_ROM_PTR(&rosette_node_create_publisher_obj) },
    { MP_ROM_QSTR(MP_QSTR_create_subscription), MP_ROM_PTR(&rosette_node_create_subscription_obj) },
    { MP_ROM_QSTR(MP_QSTR_name), MP_ROM_PTR(&rosette_node_name_obj) },
    { MP_ROM_QSTR(MP_QSTR_node_namespace), MP_ROM_PTR(&rosette_node_namespace_obj) },
};

static MP_DEFINE_CONST_DICT(rosette_node_locals_dict, rosette_node_locals_dict_table);

}  // namespace rosette::bindings::micropython

extern "C" {

const mp_obj_full_type_t rosette_node_type = {
    { &mp_type_type },
    MP_TYPE_FLAG_NONE,
    MP_QSTR_Node,
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
        reinterpret_cast<const void*>(rosette::bindings::micropython::NodeMakeNew),
        reinterpret_cast<const void*>(rosette::bindings::micropython::NodePrint),
        &rosette::bindings::micropython::rosette_node_locals_dict,
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
