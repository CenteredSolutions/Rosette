/**
 * @file bindings/micropython/modrosette.c
 * @brief MicroPython module registration for Rosette.
 *
 * This file binds the top-level `rosette` module namespace into MicroPython's
 * module registry. Type implementations live in the C++ sources so they can
 * bridge cleanly to Rosette's micro-ROS backend, while the module surface
 * remains a normal user C module from MicroPython's point of view.
 */

#include "py/obj.h"
#include "py/runtime.h"

#include "rosette_mpy_types.h"

extern mp_obj_t rosette_schema_make_from_text(size_t n_args,
                                              const mp_obj_t* args);

static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(
    rosette_schema_obj, 1, 4, rosette_schema_make_from_text);

static const mp_rom_map_elem_t rosette_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_rosette) },
    { MP_ROM_QSTR(MP_QSTR_Context), MP_ROM_PTR((const mp_obj_type_t *)&rosette_context_type) },
    { MP_ROM_QSTR(MP_QSTR_Schema), MP_ROM_PTR((const mp_obj_type_t *)&rosette_schema_type) },
    { MP_ROM_QSTR(MP_QSTR_Message), MP_ROM_PTR((const mp_obj_type_t *)&rosette_message_type) },
    { MP_ROM_QSTR(MP_QSTR_Node), MP_ROM_PTR((const mp_obj_type_t *)&rosette_node_type) },
    { MP_ROM_QSTR(MP_QSTR_Publisher), MP_ROM_PTR((const mp_obj_type_t *)&rosette_publisher_type) },
    { MP_ROM_QSTR(MP_QSTR_Subscription), MP_ROM_PTR((const mp_obj_type_t *)&rosette_subscription_type) },
    { MP_ROM_QSTR(MP_QSTR_schema), MP_ROM_PTR(&rosette_schema_obj) },
};

static MP_DEFINE_CONST_DICT(rosette_module_globals, rosette_module_globals_table);

const mp_obj_module_t rosette_user_cmodule = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&rosette_module_globals,
};

MP_REGISTER_MODULE(MP_QSTR_rosette, rosette_user_cmodule);
