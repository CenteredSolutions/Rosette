/**
 * @file bindings/micropython/rosette_mpy_obj_defs.c
 * @brief Pure-C definitions for MicroPython function objects and locals dicts.
 *
 * MP_DEFINE_CONST_FUN_OBJ_* macros expand to C99 designated initialisers
 * which are not legal in C++17.  All such definitions live here, compiled as C,
 * and the C++ translation units expose plain C-linkage functions that the
 * macros reference directly.
 */

#include "py/obj.h"
#include "py/runtime.h"
#include "py/stream.h"

/* -------------------------------------------------------------------------
 * Transport method declarations (defined in rosette_mpy_transport.cpp).
 * ---------------------------------------------------------------------- */

extern mp_obj_t rosette_transport_build_probe();
extern mp_obj_t rosette_transport_parse_reply(mp_obj_t);
extern mp_uint_t rosette_transport_ioctl(mp_obj_t, mp_uint_t, uintptr_t, int*);

static MP_DEFINE_CONST_FUN_OBJ_0(rosette_transport_build_probe_obj, rosette_transport_build_probe);
static MP_DEFINE_CONST_FUN_OBJ_1(rosette_transport_parse_reply_obj, rosette_transport_parse_reply);

static const mp_rom_map_elem_t rosette_transport_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR__build_probe), MP_ROM_PTR(&rosette_transport_build_probe_obj) },
    { MP_ROM_QSTR(MP_QSTR__parse_reply), MP_ROM_PTR(&rosette_transport_parse_reply_obj) },
};

MP_DEFINE_CONST_DICT(rosette_transport_locals_dict, rosette_transport_locals_dict_table);


/* -------------------------------------------------------------------------
 * Context method declarations (defined in rosette_mpy_context.cpp).
 * ---------------------------------------------------------------------- */

extern mp_obj_t rosette_context_start(mp_obj_t);
extern mp_obj_t rosette_context_tick(mp_obj_t);
extern mp_obj_t rosette_context_close(mp_obj_t);
extern mp_obj_t rosette_context_started(mp_obj_t);
extern mp_obj_t rosette_context_set_transport(mp_obj_t, mp_obj_t);
extern mp_obj_t rosette_context_set_transport_var(size_t, const mp_obj_t*);
extern mp_obj_t rosette_context_transport(mp_obj_t);
extern mp_obj_t rosette_context_create_node(size_t, const mp_obj_t*);

static MP_DEFINE_CONST_FUN_OBJ_1(rosette_context_start_obj,         rosette_context_start);
static MP_DEFINE_CONST_FUN_OBJ_1(rosette_context_tick_obj,          rosette_context_tick);
static MP_DEFINE_CONST_FUN_OBJ_1(rosette_context_close_obj,         rosette_context_close);
static MP_DEFINE_CONST_FUN_OBJ_1(rosette_context_started_obj,       rosette_context_started);
MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(rosette_context_set_transport_obj, 2, 4,
    rosette_context_set_transport_var);
static MP_DEFINE_CONST_FUN_OBJ_1(rosette_context_transport_obj,     rosette_context_transport);
MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(rosette_context_create_node_obj, 2, 3,
    rosette_context_create_node);

static const mp_rom_map_elem_t rosette_context_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_start),          MP_ROM_PTR(&rosette_context_start_obj) },
    { MP_ROM_QSTR(MP_QSTR_tick),           MP_ROM_PTR(&rosette_context_tick_obj) },
    { MP_ROM_QSTR(MP_QSTR_close),          MP_ROM_PTR(&rosette_context_close_obj) },
    { MP_ROM_QSTR(MP_QSTR_started),        MP_ROM_PTR(&rosette_context_started_obj) },
    { MP_ROM_QSTR(MP_QSTR__set_transport), MP_ROM_PTR(&rosette_context_set_transport_obj) },
    { MP_ROM_QSTR(MP_QSTR_transport),      MP_ROM_PTR(&rosette_context_transport_obj) },
    { MP_ROM_QSTR(MP_QSTR_create_node),    MP_ROM_PTR(&rosette_context_create_node_obj) },
};

MP_DEFINE_CONST_DICT(rosette_context_locals_dict, rosette_context_locals_dict_table);

/* -------------------------------------------------------------------------
 * Node method declarations (defined in rosette_mpy_node.cpp).
 * ---------------------------------------------------------------------- */

extern mp_obj_t rosette_node_create_publisher(size_t, const mp_obj_t*);
extern mp_obj_t rosette_node_create_subscription(size_t, const mp_obj_t*);
extern mp_obj_t rosette_node_name(mp_obj_t);
extern mp_obj_t rosette_node_namespace(mp_obj_t);

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(rosette_node_create_publisher_obj,    3, 5,
    rosette_node_create_publisher);
MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(rosette_node_create_subscription_obj, 3, 6,
    rosette_node_create_subscription);
static MP_DEFINE_CONST_FUN_OBJ_1(rosette_node_name_obj,      rosette_node_name);
static MP_DEFINE_CONST_FUN_OBJ_1(rosette_node_namespace_obj, rosette_node_namespace);

static const mp_rom_map_elem_t rosette_node_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_create_publisher),    MP_ROM_PTR(&rosette_node_create_publisher_obj) },
    { MP_ROM_QSTR(MP_QSTR_create_subscription), MP_ROM_PTR(&rosette_node_create_subscription_obj) },
    { MP_ROM_QSTR(MP_QSTR_name),           MP_ROM_PTR(&rosette_node_name_obj) },
    { MP_ROM_QSTR(MP_QSTR_node_namespace), MP_ROM_PTR(&rosette_node_namespace_obj) },
};

MP_DEFINE_CONST_DICT(rosette_node_locals_dict, rosette_node_locals_dict_table);

/* -------------------------------------------------------------------------
 * Publisher method objects.
 * ---------------------------------------------------------------------- */

extern mp_obj_t rosette_publisher_publish(mp_obj_t, mp_obj_t);

static MP_DEFINE_CONST_FUN_OBJ_2(rosette_publisher_publish_obj, rosette_publisher_publish);

static const mp_rom_map_elem_t rosette_publisher_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_publish), MP_ROM_PTR(&rosette_publisher_publish_obj) },
};

MP_DEFINE_CONST_DICT(rosette_publisher_locals_dict, rosette_publisher_locals_dict_table);

/* -------------------------------------------------------------------------
 * Subscription method objects.
 * ---------------------------------------------------------------------- */

extern mp_obj_t rosette_subscription_recv(mp_obj_t);
extern mp_obj_t rosette_subscription_queued_count(mp_obj_t);

static MP_DEFINE_CONST_FUN_OBJ_1(rosette_subscription_recv_obj,
    rosette_subscription_recv);
static MP_DEFINE_CONST_FUN_OBJ_1(rosette_subscription_queued_count_obj,
    rosette_subscription_queued_count);

static const mp_rom_map_elem_t rosette_subscription_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_recv),         MP_ROM_PTR(&rosette_subscription_recv_obj) },
    { MP_ROM_QSTR(MP_QSTR_queued_count), MP_ROM_PTR(&rosette_subscription_queued_count_obj) },
};

MP_DEFINE_CONST_DICT(rosette_subscription_locals_dict, rosette_subscription_locals_dict_table);

/* -------------------------------------------------------------------------
 * Stream protocol structs.
 * ---------------------------------------------------------------------- */

extern mp_uint_t rosette_subscription_read(mp_obj_t, void*, mp_uint_t, int*);
extern mp_uint_t rosette_subscription_write(mp_obj_t, const void*, mp_uint_t, int*);
extern mp_uint_t rosette_subscription_ioctl(mp_obj_t, mp_uint_t, uintptr_t, int*);
extern mp_uint_t rosette_publisher_read(mp_obj_t, void*, mp_uint_t, int*);
extern mp_uint_t rosette_publisher_write(mp_obj_t, const void*, mp_uint_t, int*);
extern mp_uint_t rosette_publisher_ioctl(mp_obj_t, mp_uint_t, uintptr_t, int*);

const mp_stream_p_t rosette_subscription_stream_p = {
    .read  = rosette_subscription_read,
    .write = rosette_subscription_write,
    .ioctl = rosette_subscription_ioctl,
};

const mp_stream_p_t rosette_publisher_stream_p = {
    .read  = rosette_publisher_read,
    .write = rosette_publisher_write,
    .ioctl = rosette_publisher_ioctl,
};
