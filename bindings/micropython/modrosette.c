/**
 * @file bindings/micropython/modrosette.c
 * @brief MicroPython module registration for Rosette.
 */

#include "py/obj.h"
#include "py/mphal.h"
#include "py/runtime.h"
#include "py/stream.h"
#include <sys/time.h>

#include "rosette_mpy_types.h"

/* Transport type — MakeNew defined in rosette_mpy_transport.cpp (C linkage). */
extern mp_obj_t rosette_transport_make_new(const mp_obj_type_t*, size_t, size_t, const mp_obj_t*);
extern const mp_obj_dict_t rosette_transport_locals_dict;
extern const mp_stream_p_t rosette_transport_stream_p;

MP_DEFINE_CONST_OBJ_TYPE(
    rosette_transport_type,
    MP_QSTR_Transport,
    MP_TYPE_FLAG_NONE,
    make_new, (const void*)rosette_transport_make_new,
    locals_dict, &rosette_transport_locals_dict
);

/* Schema factory function. */
extern mp_obj_t rosette_schema_make_from_text(size_t n_args, const mp_obj_t* args);

static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(
    rosette_schema_obj, 1, 4, rosette_schema_make_from_text);

static const mp_rom_map_elem_t rosette_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),    MP_ROM_QSTR(MP_QSTR__rosette) },
    { MP_ROM_QSTR(MP_QSTR_Transport),   MP_ROM_PTR((const mp_obj_type_t *)&rosette_transport_type) },
    { MP_ROM_QSTR(MP_QSTR_Context),     MP_ROM_PTR((const mp_obj_type_t *)&rosette_context_type) },
    { MP_ROM_QSTR(MP_QSTR_Schema),      MP_ROM_PTR((const mp_obj_type_t *)&rosette_schema_type) },
    { MP_ROM_QSTR(MP_QSTR_Node),        MP_ROM_PTR((const mp_obj_type_t *)&rosette_node_type) },
    { MP_ROM_QSTR(MP_QSTR_Publisher),   MP_ROM_PTR((const mp_obj_type_t *)&rosette_publisher_type) },
    { MP_ROM_QSTR(MP_QSTR_Subscription),MP_ROM_PTR((const mp_obj_type_t *)&rosette_subscription_type) },
    { MP_ROM_QSTR(MP_QSTR_schema),      MP_ROM_PTR(&rosette_schema_obj) },
};

static MP_DEFINE_CONST_DICT(rosette_module_globals, rosette_module_globals_table);

const mp_obj_module_t rosette_user_cmodule = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&rosette_module_globals,
};

MP_REGISTER_MODULE(MP_QSTR__rosette, rosette_user_cmodule);

__attribute__((weak))
int clock_gettime(int clk_id, struct timespec* tp)
{
    (void)clk_id;
    uint64_t now = mp_hal_time_ns();
    tp->tv_sec = now / 1000000000ull;
    tp->tv_nsec = now % 1000000000ull;
    return 0;
}