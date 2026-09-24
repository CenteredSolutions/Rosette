/**
 * @file rmw_rosette/topic_naming.hpp
 * @brief ROS 2 DDS topic-name and type-name mangling.
 *
 * An rmw implementation owns the mapping between ROS 2 topic/service
 * names and the RTPS topic names actually placed on the wire — the same
 * convention rmw_fastrtps_cpp and rmw_cyclonedds_cpp apply internally.
 * rcl expands and validates names before calling rmw, so every name that
 * reaches these helpers is fully qualified (it starts with `/` and already
 * contains the node namespace and any remapping). Mangling is therefore a
 * pure prefix/suffix operation:
 *
 *   User topics:        rt<fully-qualified topic>              rt/ns/chatter
 *   Service request:    rq<fully-qualified service>Request     rq/ns/add_twoRequest
 *   Service response:   rr<fully-qualified service>Reply       rr/ns/add_twoReply
 *
 * Type names follow the convention rmw_fastrtps_cpp uses so real ROS 2
 * nodes' SEDP matching accepts this participant's endpoints:
 *   <pkg>::msg::dds_::<Type>_
 *
 * All composition is bounded-buffer, no heap, no std::string, matching the
 * rest of rosette::rtps.
 */

#pragma once

#include "rosette/rtps/endpoint_match_table.hpp"  // rtps::TopicName, RTPS_MAX_TOPIC_NAME_LEN

#include <cstddef>
#include <cstring>

namespace rmw_rosette {

namespace detail {

/// @brief Appends `src` at `pos`, truncating to keep `dst` NUL-terminated within `cap`.
inline void str_append(char* dst, size_t cap, size_t& pos, const char* src) noexcept {
    if (!dst || !src || cap == 0) return;
    while (*src && pos + 1 < cap) { dst[pos++] = *src++; }
    dst[pos] = '\0';
}

/**
 * @brief Writes `prefix + fqn + suffix`, the shape of every ROS 2 DDS topic name.
 *
 * @param[out] out    Destination buffer.
 * @param[in]  cap    Capacity of `out`, including the terminating NUL.
 * @param[in]  prefix Two-letter mangling prefix without a slash (`"rt"`, `"rq"`, `"rr"`).
 * @param[in]  fqn    Fully-qualified ROS name; its leading `/` becomes the separator.
 * @param[in]  suffix Appended verbatim (`""`, `"Request"`, `"Reply"`).
 */
inline void compose_mangled(char* out, size_t cap, const char* prefix, const char* fqn,
                            const char* suffix) noexcept {
    size_t pos = 0;
    str_append(out, cap, pos, prefix);
    if (!fqn || fqn[0] != '/') str_append(out, cap, pos, "/");
    str_append(out, cap, pos, fqn ? fqn : "");
    str_append(out, cap, pos, suffix);
}

}  // namespace detail

/**
 * @brief Composes the RTPS topic name for a ROS 2 user topic.
 *
 * `rmw_create_publisher()`/`rmw_create_subscription()` place the result in
 * `WriterConfig::topicName`/`ReaderConfig::topicName`, which SEDP announces.
 *
 * @param[out] out   Destination buffer; truncated rather than overflowed.
 * @param[in]  cap   Capacity of `out`, including the terminating NUL.
 * @param[in]  topic Fully-qualified ROS 2 topic name as rcl passes it
 *   (for example `/chatter` or `/robot1/cmd_vel`).
 *
 * Verification:
 * - Correct means: `/chatter` maps to `rt/chatter` and `/ns/chatter` to
 *   `rt/ns/chatter`, matching the names rmw_fastrtps_cpp announces.
 * - Covered by: tests/ros2/run_rmw_interop_loopback.sh (endpoint matching
 *   against rmw_fastrtps_cpp).
 *
 * References:
 * - ROS 2 design, "Topic and Service name mapping to DDS"
 *   (design.ros2.org/articles/topic_and_service_names.html), "ROS Specific
 *   Namespace Prefix"
 */
inline void compose_topic_name(char* out, size_t cap, const char* topic) noexcept {
    detail::compose_mangled(out, cap, "rt", topic, "");
}

/**
 * @brief Composes the RTPS topic name for a service's request channel.
 *
 * @param[out] out     Destination buffer; truncated rather than overflowed.
 * @param[in]  cap     Capacity of `out`, including the terminating NUL.
 * @param[in]  service Fully-qualified ROS 2 service name.
 *
 * Verification:
 * - Correct means: `/add_two` maps to `rq/add_twoRequest`.
 *
 * References:
 * - ROS 2 design, "Topic and Service name mapping to DDS"
 */
inline void compose_service_request_topic(char* out, size_t cap, const char* service) noexcept {
    detail::compose_mangled(out, cap, "rq", service, "Request");
}

/**
 * @brief Composes the RTPS topic name for a service's reply channel.
 *
 * @param[out] out     Destination buffer; truncated rather than overflowed.
 * @param[in]  cap     Capacity of `out`, including the terminating NUL.
 * @param[in]  service Fully-qualified ROS 2 service name.
 *
 * Verification:
 * - Correct means: `/add_two` maps to `rr/add_twoReply`.
 *
 * References:
 * - ROS 2 design, "Topic and Service name mapping to DDS"
 */
inline void compose_service_reply_topic(char* out, size_t cap, const char* service) noexcept {
    detail::compose_mangled(out, cap, "rr", service, "Reply");
}

/**
 * @brief Composes the DDS-XTypes wire type name for a ROS 2 message type.
 *
 * `rmw_create_publisher()`/`rmw_create_subscription()` place the result
 * in `WriterConfig::typeName`/`ReaderConfig::typeName`, which SEDP
 * announces to the graph; a real ROS 2 node's rmw only matches an
 * endpoint whose type name follows this exact convention.
 *
 * Writes `<message_namespace> + "::dds_::" + <message_name> + "_"`, with
 * every `"__"` in the namespace rewritten to `"::"`. Introspection metadata
 * spells the namespace `"std_msgs__msg"` while microxrcedds callbacks spell
 * it `"std_msgs::msg"`; both produce `"std_msgs::msg::dds_::String_"`.
 *
 * @param[out] out Destination buffer.
 * @param[in] cap Capacity of `out`, including the terminating NUL.
 * @param[in] message_namespace Package and subfolder, `"__"`- or `"::"`-separated.
 * @param[in] message_name Bare message type name.
 *
 * Verification:
 * - Correct means: output equals the type name rmw_fastrtps_cpp announces.
 * - Covered by: rmw_rosette/test/test_introspection_c_codec.cpp
 *   (ComposesDdsTypeName).
 *
 * References:
 * - ROS 2 design, "Topic and Service name mapping to DDS"
 */
inline void compose_dds_type_name(char* out, size_t cap,
                                   const char* message_namespace,
                                   const char* message_name) noexcept {
    size_t pos = 0;
    for (const char* p = message_namespace ? message_namespace : ""; *p; ++p) {
        if (p[0] == '_' && p[1] == '_') {
            detail::str_append(out, cap, pos, "::");
            ++p;
        } else {
            const char c[2] = {*p, '\0'};
            detail::str_append(out, cap, pos, c);
        }
    }
    detail::str_append(out, cap, pos, "::dds_::");
    detail::str_append(out, cap, pos, message_name ? message_name : "");
    detail::str_append(out, cap, pos, "_");
}

}  // namespace rmw_rosette
