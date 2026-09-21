/**
 * @file rmw_rosette/topic_naming.hpp
 * @brief ROS 2 DDS topic-name and type-name mangling.
 *
 * An rmw implementation owns the mapping between ROS 2 topic/service
 * names and the RTPS topic names actually placed on the wire — the same
 * convention rmw_fastrtps_cpp and rmw_cyclonedds_cpp apply internally.
 * All composition here is bounded-buffer, no heap, no std::string,
 * matching the rest of rosette::rtps.
 *
 *   User topics:        rt/<namespace>/<topic>
 *   Service request:    rq/<namespace>/<service>Request
 *   Service response:   rr/<namespace>/<service>Reply
 *
 * Type names follow the DDS-XTypes convention rmw_fastrtps_cpp uses so
 * real ROS 2 nodes' SEDP matching accepts this participant's endpoints:
 *   <pkg>::msg::dds_::<Type>_
 */

#pragma once

#include "rosette/rtps/endpoint_match_table.hpp"  // rtps::TopicName, RTPS_MAX_TOPIC_NAME_LEN

#include <cstddef>
#include <cstring>

namespace rmw_rosette {

namespace detail {

inline void str_append(char* dst, size_t cap, size_t& pos, const char* src) noexcept {
    if (!dst || !src || cap == 0) return;
    while (*src && pos + 1 < cap) { dst[pos++] = *src++; }
    dst[pos] = '\0';
}

inline void ensure_slash(char* dst, size_t cap, size_t& pos) noexcept {
    if (pos > 0 && dst[pos - 1] != '/' && pos + 1 < cap) {
        dst[pos++] = '/';
        dst[pos] = '\0';
    }
}

/// Appends `ns` (with trailing slash), skipping empty or root ("/") namespaces.
inline void append_namespace(char* dst, size_t cap, size_t& pos, const char* ns) noexcept {
    if (!ns || ns[0] == '\0' || (ns[0] == '/' && ns[1] == '\0')) return;
    const char* p = ns;
    while (*p == '/') ++p;
    str_append(dst, cap, pos, p);
    ensure_slash(dst, cap, pos);
}

}  // namespace detail

/**
 * @brief Composes the RTPS topic name for a ROS 2 user topic.
 *
 * `rmw_create_publisher()`/`rmw_create_subscription()` call this to turn
 * the ROS-level topic name and node namespace they receive into the
 * actual RTPS `topicName` a `WriterConfig`/`ReaderConfig` carries, so
 * real ROS 2 nodes' SEDP endpoint discovery recognizes this participant.
 *
 * Writes `"rt/" + <namespace, with leading/trailing slashes normalized> +
 * <topic>` into `out`, truncating at `cap` rather than overflowing.
 *
 * @param[out] out Destination buffer.
 * @param[in] cap Capacity of `out`, including the terminating NUL.
 * @param[in] ns Node namespace; `nullptr`, `""`, and `"/"` are all
 *   treated as no namespace.
 * @param[in] topic ROS 2 topic name.
 */
inline void compose_topic_name(char* out, size_t cap,
                                const char* ns, const char* topic) noexcept {
    size_t pos = 0;
    detail::str_append(out, cap, pos, "rt/");
    detail::append_namespace(out, cap, pos, ns);
    detail::str_append(out, cap, pos, topic ? topic : "");
}

/**
 * @brief Composes the RTPS topic name for a service's request channel.
 *
 * Mirrors compose_topic_name(), producing `"rq/" + <namespace> +
 * <service> + "Request"` for the reader/writer pair a service uses to
 * receive incoming calls.
 *
 * @param[out] out Destination buffer.
 * @param[in] cap Capacity of `out`, including the terminating NUL.
 * @param[in] ns Node namespace; `nullptr`, `""`, and `"/"` are all
 *   treated as no namespace.
 * @param[in] service ROS 2 service name.
 */
inline void compose_service_request_topic(char* out, size_t cap,
                                           const char* ns, const char* service) noexcept {
    size_t pos = 0;
    detail::str_append(out, cap, pos, "rq/");
    detail::append_namespace(out, cap, pos, ns);
    detail::str_append(out, cap, pos, service ? service : "");
    detail::str_append(out, cap, pos, "Request");
}

/**
 * @brief Composes the RTPS topic name for a service's reply channel.
 *
 * The mirror image of compose_service_request_topic(): `"rr/" +
 * <namespace> + <service> + "Reply"`.
 *
 * @param[out] out Destination buffer.
 * @param[in] cap Capacity of `out`, including the terminating NUL.
 * @param[in] ns Node namespace; `nullptr`, `""`, and `"/"` are all
 *   treated as no namespace.
 * @param[in] service ROS 2 service name.
 */
inline void compose_service_reply_topic(char* out, size_t cap,
                                         const char* ns, const char* service) noexcept {
    size_t pos = 0;
    detail::str_append(out, cap, pos, "rr/");
    detail::append_namespace(out, cap, pos, ns);
    detail::str_append(out, cap, pos, service ? service : "");
    detail::str_append(out, cap, pos, "Reply");
}

/**
 * @brief Composes the DDS-XTypes wire type name for a ROS 2 message type.
 *
 * `rmw_create_publisher()`/`rmw_create_subscription()` place the result
 * in `WriterConfig::typeName`/`ReaderConfig::typeName`, which SEDP
 * announces to the graph; a real ROS 2 node's rmw only matches an
 * endpoint whose type name follows this exact convention.
 *
 * Writes `<message_namespace> + "::dds_::" + <message_name> + "_"`. The
 * two inputs come straight from a `message_type_support_callbacks_t`
 * (type_support.hpp): `message_namespace_` is `"<pkg>::<subfolder>"`
 * (e.g. `"std_msgs::msg"`) and `message_name_` is the bare type name
 * (e.g. `"String"`), per rosidl_typesupport_microxrcedds's codegen
 * template.
 *
 * @param[out] out Destination buffer.
 * @param[in] cap Capacity of `out`, including the terminating NUL.
 * @param[in] message_namespace `message_type_support_callbacks_t::message_namespace_`.
 * @param[in] message_name `message_type_support_callbacks_t::message_name_`.
 */
inline void compose_dds_type_name(char* out, size_t cap,
                                   const char* message_namespace,
                                   const char* message_name) noexcept {
    size_t pos = 0;
    detail::str_append(out, cap, pos, message_namespace ? message_namespace : "");
    detail::str_append(out, cap, pos, "::dds_::");
    detail::str_append(out, cap, pos, message_name ? message_name : "");
    detail::str_append(out, cap, pos, "_");
}

}  // namespace rmw_rosette
