// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

#pragma once

/**
 * @file bindings/micropython/rosette_mpy_backend.hpp
 * @brief C++ backend entry points for the Rosette MicroPython binding.
 *
 * The MicroPython-facing object implementations live at the C API boundary,
 * while the underlying ROS transport and entity management lives in Rosette's
 * C++ micro-ROS backend. This header defines the narrow interface between the
 * two layers so object layout, transport wiring, and schema-driven message
 * conversion can evolve without forcing every source file to know about every
 * Rosette class.
 */

#include "rosette_mpy_types.h"

#include <stddef.h>
#include <stdint.h>

namespace rosette::bindings::micropython {

/**
 * @brief QoS settings shared by publisher and subscription construction.
 *
 * The MicroPython API exposes only the QoS controls that materially change
 * basic interoperability: queue depth and reliability.
 */
struct EndpointOptions {
    uint16_t depth = 1u;
    bool reliable = false;
};

// -----------------------------------------------------------------------
// Schema CDR codec — defined in rosette_mpy_schema.cpp
// -----------------------------------------------------------------------

/**
 * @brief Encodes a Python message value into CDR using runtime schema descriptors.
 *
 * Writes the 4-byte CDR encapsulation header followed by each field in schema
 * declaration order.  Returns the number of bytes written, or 0 on overflow.
 *
 * @param[in]  schema   Parsed schema descriptor for the message.
 * @param[in]  payload  Python dict, Message, or bare scalar.
 * @param[out] buf      Destination buffer.
 * @param[in]  buf_size Capacity of `buf`.
 * @return Bytes written (>= 4), or 0 on overflow.
 */
size_t SchemaCdrEncode(const mp_rosette_schema_obj_t* schema,
                       mp_obj_t payload,
                       uint8_t* buf,
                       size_t   buf_size);

/**
 * @brief Decodes a CDR payload into a Python Message using runtime schema descriptors.
 *
 * @param[in] schema  Parsed schema descriptor for the expected message type.
 * @param[in] buf     Raw CDR bytes including the 4-byte encapsulation header.
 * @param[in] length  Valid byte count in `buf`.
 * @return New Message object, or None when the payload is malformed.
 */
mp_obj_t SchemaCdrDecode(const mp_rosette_schema_obj_t* schema,
                         const uint8_t* buf,
                         size_t         length);

/**
 * @brief Starts the Rosette micro-ROS backend for one MicroPython context.
 *
 * This function creates the `milkros::micro_ros::Context`, configures it to
 * use the MicroPython-owned transport callbacks, and starts the XRCE session.
 *
 * @param[in,out] self MicroPython `Context` object that owns the transport and
 * backend pointer.
 * @return `true` when the backend started successfully and the context is
 * ready to create nodes.
 */
bool ContextStart(mp_rosette_context_obj_t* self) noexcept;

/**
 * @brief Returns diagnostic text for the last backend startup attempt.
 *
 * Real micro-ROS builds translate Rosette's typed `ContextStartError` into a
 * stable message suitable for MicroPython exceptions. Stub builds return
 * `ok` after their no-op startup path so smoke tests keep the same reporting
 * surface.
 *
 * @param[in] self MicroPython `Context` object whose backend startup status
 * should be described.
 * @return Static diagnostic string for the most recent startup result.
 */
const char* ContextStartErrorMessage(const mp_rosette_context_obj_t* self) noexcept;

/**
 * @brief Finalizes the Rosette micro-ROS backend owned by one context.
 *
 * This releases the backend context and any entity state that depends on it.
 *
 * @param[in,out] self MicroPython `Context` object whose backend is being
 * finalized.
 */
void ContextFini(mp_rosette_context_obj_t* self) noexcept;

/**
 * @brief Advances the shared Rosette executor owned by one context.
 *
 * The binding's explicit `tick()` call and the poll integration path both use
 * this function to make bounded progress on subscriptions, timers, and other
 * cooperative work.
 *
 * @param[in,out] self MicroPython `Context` object whose backend should
 * advance.
 * @return `true` when the backend tick completed successfully.
 */
bool ContextTick(mp_rosette_context_obj_t* self) noexcept;

/**
 * @brief Creates the backend node owned by one MicroPython `Node` object.
 *
 * @param[in,out] self MicroPython `Node` object to initialize.
 * @return `true` when the node backend was created successfully.
 */
bool NodeInit(mp_rosette_node_obj_t* self) noexcept;

/**
 * @brief Finalizes the backend node owned by one MicroPython `Node` object.
 *
 * @param[in,out] self MicroPython `Node` object whose backend should be
 * released.
 */
void NodeFini(mp_rosette_node_obj_t* self) noexcept;

/**
 * @brief Creates the backend publisher owned by one `Publisher` object.
 *
 * @param[in,out] self Publisher object to initialize.
 * @param[in] options QoS settings translated from the Python constructor call.
 * @return `true` when the publisher backend was created successfully.
 */
bool PublisherInit(mp_rosette_publisher_obj_t* self,
                   const EndpointOptions& options) noexcept;

/**
 * @brief Finalizes the backend publisher owned by one `Publisher` object.
 *
 * @param[in,out] self Publisher object whose backend should be released.
 */
void PublisherFini(mp_rosette_publisher_obj_t* self) noexcept;

/**
 * @brief Publishes one Python payload through the backend publisher.
 *
 * The backend converts the Python object into the generated C message selected
 * by the publisher's schema before handing it to `rcl_publish()`.
 *
 * @param[in,out] self Publisher object that owns the backend handle.
 * @param[in] payload Python payload to convert and publish.
 * @return `true` when the backend accepted the payload for transmission.
 */
bool PublisherPublish(mp_rosette_publisher_obj_t* self,
                      mp_obj_t payload);

/**
 * @brief Initialises per-object fields for one `Subscription` before the
 *        native backend handle is created.
 *
 * @param[in,out] self Subscription object to initialize.
 * @param[in] options QoS settings translated from the Python constructor call.
 * @return `true` always (exists for symmetry with publisher init).
 */
bool SubscriptionInit(mp_rosette_subscription_obj_t* self,
                      const EndpointOptions& options) noexcept;

/**
 * @brief Creates the native backend subscription for one binding object.
 *
 * The Python-facing subscription object owns callback and queue state while
 * this backend handle owns the `rcl_subscription_t` used by the shared context
 * pump. Dynamic schemas are received as serialized CDR payloads and decoded by
 * the runtime schema codec.
 *
 * @param[in,out] self Subscription object to attach to the backend.
 * @param[in] options QoS settings translated from the Python constructor call.
 * @return `true` when the subscription can receive data.
 */
bool SubscriptionBackendInit(mp_rosette_subscription_obj_t* self,
                             const EndpointOptions& options) noexcept;

/**
 * @brief Finalizes the backend subscription owned by one `Subscription` object.
 *
 * @param[in,out] self Subscription object whose backend should be released.
 */
void SubscriptionFini(mp_rosette_subscription_obj_t* self) noexcept;

/**
 * @brief Releases the native backend subscription owned by one object.
 *
 * @param[in,out] self Subscription object whose native handle should be
 * finalized.
 */
void SubscriptionBackendFini(mp_rosette_subscription_obj_t* self) noexcept;

/**
 * @brief Drains currently available serialized samples for one subscription.
 *
 * The context pump calls this after each executor tick. Each received CDR
 * payload is decoded through the subscription schema and delivered to the
 * Python callback immediately. Messages with no callback registered are
 * silently dropped.
 *
 * @param[in,out] self Subscription whose pending samples should be processed.
 * @return `true` when draining completed without a hard backend failure.
 */
bool SubscriptionDispatchPending(mp_rosette_subscription_obj_t* self) noexcept;

/**
 * @brief Creates the C++ CustomTransport adapter for a Transport object.
 *
 * @param[in,out] self Transport object that will own the adapter.
 * @return `true` when the adapter was allocated successfully.
 */
bool TransportInit(mp_rosette_transport_obj_t* self) noexcept;

/**
 * @brief Finalizes the C++ CustomTransport adapter owned by a Transport object.
 *
 * @param[in,out] self Transport object whose adapter should be destroyed.
 */
void TransportFini(mp_rosette_transport_obj_t* self) noexcept;

/**
 * @brief Returns whether the transport currently treats its socket as connected.
 *
 * @param[in] self Transport object to inspect.
 * @return `true` when the transport can use `send()` / `recv()` directly.
 */
bool TransportConnected(const mp_rosette_transport_obj_t* self) noexcept;

}  // namespace rosette::bindings::micropython
