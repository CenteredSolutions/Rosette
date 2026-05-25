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
 * @brief Creates the backend subscription owned by one `Subscription` object.
 *
 * The backend callback path writes raw CDR payloads into the subscription's
 * ring buffer.
 *
 * @param[in,out] self Subscription object to initialize.
 * @param[in] options QoS settings translated from the Python constructor call.
 * @return `true` when the subscription backend was created successfully.
 */
bool SubscriptionInit(mp_rosette_subscription_obj_t* self,
                      const EndpointOptions& options) noexcept;

/**
 * @brief Finalizes the backend subscription owned by one `Subscription` object.
 *
 * @param[in,out] self Subscription object whose backend should be released.
 */
void SubscriptionFini(mp_rosette_subscription_obj_t* self) noexcept;

/**
 * @brief Enqueues one received CDR payload into a subscription ring buffer.
 *
 * The transport and backend callback path uses this helper to push serialized
 * payload bytes into the oldest free ring slot. The queue is bounded so the
 * implementation can remain deterministic on small boards.
 *
 * @param[in,out] self Subscription whose ring buffer receives the payload.
 * @param[in] payload Serialized message payload bytes.
 * @param[in] payload_size Number of bytes available in `payload`.
 * @return `true` when the payload was stored successfully.
 */
bool SubscriptionEnqueueFrame(mp_rosette_subscription_obj_t* self,
                              const uint8_t* payload,
                              size_t payload_size) noexcept;

/**
 * @brief Returns whether the subscription currently has queued payloads.
 *
 * @param[in] self Subscription whose queue state should be inspected.
 * @return `true` when `recv()` can consume one payload immediately.
 */
bool SubscriptionReadable(const mp_rosette_subscription_obj_t* self) noexcept;

/**
 * @brief Creates the MicroPython-socket-backed custom transport owned by a context.
 *
 * The context constructor creates the Python socket object first, then uses
 * this helper to allocate the C++ adapter that implements Rosette's
 * `CustomTransport` interface on top of that socket.
 *
 * @param[in,out] self Context that owns the socket and transport state.
 * @return `true` when the transport adapter was allocated successfully.
 */
bool TransportInit(mp_rosette_context_obj_t* self) noexcept;

/**
 * @brief Finalizes the transport adapter owned by a context.
 *
 * @param[in,out] self Context whose transport adapter should be destroyed.
 */
void TransportFini(mp_rosette_context_obj_t* self) noexcept;

/**
 * @brief Returns whether the transport currently has a connected UDP socket.
 *
 * @param[in] self Context whose transport state should be inspected.
 * @return `true` when the transport can use `send()` / `recv()` directly.
 */
bool TransportConnected(const mp_rosette_context_obj_t* self) noexcept;

/**
 * @brief Sends one UDP packet through the transport adapter.
 *
 * @param[in,out] self Context that owns the transport.
 * @param[in] payload Python bytes-like packet payload.
 * @return Python integer number of bytes reported as transmitted.
 */
mp_obj_t TransportSendPacket(mp_rosette_context_obj_t* self,
                             mp_obj_t payload);

/**
 * @brief Receives one UDP packet through the transport adapter.
 *
 * @param[in,out] self Context that owns the transport.
 * @param[in] max_bytes_in Python integer maximum packet size.
 * @return Received packet bytes, or `None` when no packet is ready.
 */
mp_obj_t TransportRecvPacket(mp_rosette_context_obj_t* self,
                             mp_obj_t max_bytes_in);

}  // namespace rosette::bindings::micropython
