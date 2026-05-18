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
 * - `Subscription` stores raw payload frames in a bounded ring buffer so the
 *   receive path remains deterministic and Python object allocation happens in
 *   `recv()` rather than in the transport callback.
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
    ROSETTE_MPY_FIELD_FIXED_ARRAY,
    ROSETTE_MPY_FIELD_NESTED_MESSAGE,
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
    qstr name;
    uint8_t kind;
    uint8_t element_kind;
    uint16_t element_count;
    uint16_t string_capacity;
    struct _mp_rosette_schema_obj_t* nested_schema;
} rosette_mpy_schema_field_t;

/**
 * @brief One received CDR payload stored in a subscription ring buffer.
 *
 * The ring buffer stores wire-format payload bytes plus the payload length. A
 * slot is considered readable when `occupied` is true.
 */
typedef struct _rosette_mpy_frame_t {
    uint8_t* data;
    size_t length;
    bool occupied;
} rosette_mpy_frame_t;

struct _mp_rosette_context_obj_t;
struct _mp_rosette_schema_obj_t;
struct _mp_rosette_message_obj_t;
struct _mp_rosette_node_obj_t;
struct _mp_rosette_publisher_obj_t;
struct _mp_rosette_subscription_obj_t;

/**
 * @brief Transport state handed to the micro-ROS custom transport callbacks.
 *
 * The transport callbacks need a stable pointer back to the owning
 * `Context` object so they can access the MicroPython socket object and the
 * cached socket methods used for packet I/O.
 */
typedef struct _rosette_mpy_transport_t {
    struct _mp_rosette_context_obj_t* owner;
    bool socket_ready;
    uint32_t read_budget_us;
    void* implementation;
} rosette_mpy_transport_t;

/**
 * @brief `rosette.Context` object layout.
 *
 * The context is the binding's top-level owner. It retains the socket module
 * and socket instance as ordinary MicroPython objects so the dynamic native
 * module path can stay within the supported object runtime rather than relying
 * on internal NIC structs.
 */
typedef struct _mp_rosette_context_obj_t {
    mp_obj_base_t base;

    mp_obj_t socket_module_obj;
    mp_obj_t socket_obj;
    mp_obj_t agent_ip_obj;
    mp_obj_t remote_addr_obj;
    mp_obj_t local_addr_obj;

    mp_obj_t meth_bind;
    mp_obj_t meth_connect;
    mp_obj_t meth_send;
    mp_obj_t meth_sendto;
    mp_obj_t meth_recv;
    mp_obj_t meth_recvfrom;
    mp_obj_t meth_setblocking;
    mp_obj_t meth_close;

    mp_obj_t subscriptions;

    uint32_t client_key;
    uint32_t spin_timeout_ns;
    uint16_t agent_port;
    uint16_t local_port;
    uint16_t executor_handles;
    uint8_t backend_start_error;
    uint8_t domain_id;

    bool started;
    bool closed;
    bool connected;
    bool use_connected_udp;
    bool connect_fallback_eperm;

    rosette_mpy_transport_t transport;
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
    uint8_t message_kind;
} mp_rosette_schema_obj_t;

/**
 * @brief Runtime message instance produced by a `Schema` object.
 *
 * Message instances carry their schema and a dictionary of field values. The
 * dictionary keeps MicroPython-side construction flexible while the schema
 * descriptor gives the backend enough structure to convert selected generated
 * ROSIDL message types into native C messages.
 */
typedef struct _mp_rosette_message_obj_t {
    mp_obj_base_t base;

    mp_obj_t schema_obj;
    mp_obj_t values_obj;
} mp_rosette_message_obj_t;

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
 * The subscription keeps one bounded ring buffer of received payloads. The
 * stream and poll hooks consult the ring indices directly so `select.poll()`
 * can observe readiness without decoding messages during the readiness check.
 */
typedef struct _mp_rosette_subscription_obj_t {
    mp_obj_base_t base;

    mp_obj_t node_obj;
    mp_obj_t schema_obj;
    mp_obj_t topic_obj;

    rosette_mpy_frame_t* ring;
    size_t ring_capacity;
    size_t ring_head;
    size_t ring_tail;
    size_t queued_count;
    size_t max_payload_size;

    uint16_t depth;
    bool reliable;

    void* backend;
} mp_rosette_subscription_obj_t;

extern const mp_obj_full_type_t rosette_context_type;
extern const mp_obj_full_type_t rosette_schema_type;
extern const mp_obj_full_type_t rosette_message_type;
extern const mp_obj_full_type_t rosette_node_type;
extern const mp_obj_full_type_t rosette_publisher_type;
extern const mp_obj_full_type_t rosette_subscription_type;

#ifdef __cplusplus
}  // extern "C"
#endif
