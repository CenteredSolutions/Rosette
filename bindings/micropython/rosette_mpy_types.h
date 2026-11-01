// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

#pragma once

/**
 * @file bindings/micropython/rosette_mpy_types.h
 * @brief Concrete MicroPython object layouts and schema descriptors for Rosette.
 *
 * This header defines the data structures shared by the MicroPython module
 * registration code, the stream-facing object implementations, and the C++
 * backend glue that binds Rosette's micro-ROS layer to MicroPython objects.
 *
 * The design keeps Python object ownership explicit:
 * - `Context` owns the socket object and transport configuration.
 * - `Node` references one `Context`.
 * - `Publisher` and `Subscription` reference one `Node` and one `Schema`.
 * - `Subscription` delivers received messages directly to a Python callback.
 *   Python code may build a queue or async task on top of that callback.
 */

#ifdef __cplusplus
extern "C" {
#endif

#include "py/obj.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

enum {
    ROSETTE_MPY_FIELD_BOOL = 0,
    ROSETTE_MPY_FIELD_INT8,
    ROSETTE_MPY_FIELD_UINT8,
    ROSETTE_MPY_FIELD_INT16,
    ROSETTE_MPY_FIELD_UINT16,
    ROSETTE_MPY_FIELD_INT32,
    ROSETTE_MPY_FIELD_UINT32,
    ROSETTE_MPY_FIELD_INT64,
    ROSETTE_MPY_FIELD_UINT64,
    ROSETTE_MPY_FIELD_FLOAT32,
    ROSETTE_MPY_FIELD_FLOAT64,
    ROSETTE_MPY_FIELD_STRING,
    ROSETTE_MPY_FIELD_FIXED_ARRAY,    /* kind=FIXED_ARRAY, element_kind=element type */
    ROSETTE_MPY_FIELD_DYNAMIC_ARRAY,  /* kind=DYNAMIC_ARRAY, element_kind=element type */
    ROSETTE_MPY_FIELD_NESTED_MESSAGE, /* kind=NESTED_MESSAGE, nested_schema points to schema */
};

enum {
    ROSETTE_MPY_MESSAGE_UNSUPPORTED = 0,
    ROSETTE_MPY_MESSAGE_STD_MSGS_FLOAT32,
};

/**
 * @brief One runtime field descriptor for a parsed `.msg` schema.
 *
 * The schema parser translates each message field into a compact descriptor
 * that records the field name, field kind, bounds, and nested-schema link when
 * the field itself is another message. The codec code uses these descriptors
 * to map between Python dictionaries and CDR payloads at runtime.
 */
typedef struct _rosette_mpy_schema_field_t {
    qstr    name;
    uint8_t kind;           /* outer kind: scalar, FIXED_ARRAY, DYNAMIC_ARRAY, NESTED_MESSAGE */
    uint8_t element_kind;   /* element kind when kind is an array; same as kind for scalars */
    uint16_t element_count; /* non-zero for FIXED_ARRAY */
    uint16_t string_capacity; /* reserved for bounded strings */
    qstr    nested_full_name; /* interned full_name qstr for the nested type (lazy resolve) */
    struct _mp_rosette_schema_obj_t* nested_schema; /* resolved lazily on first codec use */
} rosette_mpy_schema_field_t;

struct _mp_rosette_context_obj_t;
struct _mp_rosette_schema_obj_t;
struct _mp_rosette_node_obj_t;
struct _mp_rosette_publisher_obj_t;
struct _mp_rosette_subscription_obj_t;

/**
 * @brief `_rosette.Transport` object layout.
 *
 * The transport is a thin C base class subclassed in Python. The Python
 * subclass owns the socket and implements send/recv/sendto/recvfrom/close.
 * C code dispatches to those Python methods via mp_load_attr at call time.
 * The C struct only tracks flags that the C++ CustomTransport adapter needs
 * to check without calling back into Python on every hot-path query.
 */
typedef struct _mp_rosette_transport_obj_t {
    mp_obj_base_t base;
    mp_obj_t python_obj;
    bool socket_ready;
    bool connected;
    uint32_t read_budget_us;
    void* implementation;  /* MicroPythonSocketTransport C++ adapter */
} mp_rosette_transport_obj_t;

/**
 * @brief `rosette.Context` object layout.
 *
 * The context owns backend lifecycle and holds a reference to its Transport.
 * All socket I/O is delegated to the Python Transport object.
 */
typedef struct _mp_rosette_context_obj_t {
    mp_obj_base_t base;

    mp_obj_t transport_obj;   /* Python Transport object, mp_const_none until open() */
    mp_obj_t agent_ip_obj;    /* Agent endpoint selected by Context.open(). */
    mp_obj_t subscriptions;

    uint32_t client_key;
    uint32_t spin_timeout_us;
    uint16_t local_port;
    uint16_t agent_port;
    uint16_t executor_handles;
    uint8_t backend_start_error;
    uint8_t domain_id;
    char backend_start_detail[192];

    bool started;
    bool closed;

    void* backend;
} mp_rosette_context_obj_t;

/**
 * @brief `rosette.Schema` object layout.
 *
 * The schema stores the original message name plus a parsed descriptor array.
 * Field descriptors are allocated once and then reused by both the encode and
 * decode paths.
 */
typedef struct _mp_rosette_schema_obj_t {
    mp_obj_base_t base;

    qstr full_name;
    qstr package_name;
    qstr short_name;

    rosette_mpy_schema_field_t* fields;
    size_t field_count;
    uint16_t max_serialized_size;
    uint8_t  message_kind;
    /**
     * True when this schema was constructed from user-supplied .msg text and
     * has no statically-linked ROSIDL generated type.  Dynamic schemas use the
     * generic CDR codec and rcl_publish_serialized_message rather than the
     * generated C message path.
     */
    bool     is_dynamic;
} mp_rosette_schema_obj_t;

/**
 * @brief `rosette.Node` object layout.
 *
 * The node is a light naming wrapper over a started context. It owns the node
 * name and namespace strings as Python objects and holds the backend handle
 * that creates publishers and subscriptions.
 */
typedef struct _mp_rosette_node_obj_t {
    mp_obj_base_t base;

    mp_obj_t context_obj;
    mp_obj_t name_obj;
    mp_obj_t namespace_obj;

    void* backend;
} mp_rosette_node_obj_t;

/**
 * @brief `rosette.Publisher` object layout.
 *
 * The publisher keeps references to its parent node and schema so publish
 * operations can encode Python objects according to the runtime descriptor
 * before handing the CDR payload to Rosette's backend.
 */
typedef struct _mp_rosette_publisher_obj_t {
    mp_obj_base_t base;

    mp_obj_t node_obj;
    mp_obj_t schema_obj;
    mp_obj_t topic_obj;

    uint16_t depth;
    bool reliable;

    void* backend;
} mp_rosette_publisher_obj_t;

/**
 * @brief `rosette.Subscription` object layout.
 *
 * The subscription delivers received messages directly to `callback_obj`.
 * Python code may wrap this with a queue, asyncio task, or any other pattern.
 */
typedef struct _mp_rosette_subscription_obj_t {
    mp_obj_base_t base;

    mp_obj_t node_obj;
    mp_obj_t schema_obj;
    mp_obj_t topic_obj;
    mp_obj_t callback_obj;

    uint16_t depth;
    bool reliable;

    void* backend;
} mp_rosette_subscription_obj_t;

extern const mp_obj_type_t rosette_transport_type;
extern const mp_obj_full_type_t rosette_context_type;
extern const mp_obj_full_type_t rosette_schema_type;
extern const mp_obj_full_type_t rosette_node_type;
extern const mp_obj_full_type_t rosette_publisher_type;
extern const mp_obj_full_type_t rosette_subscription_type;

#ifdef __cplusplus
}  // extern "C"
#endif
