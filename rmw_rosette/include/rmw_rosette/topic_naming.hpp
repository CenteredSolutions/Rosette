/**
 * @file rmw_rosette/topic_naming.hpp
 * @brief ROS 2 DDS topic-name and type-name mangling.
 *
 * Ported from the archived include/rosette/ros/topic_names.hpp (see
 * archive/rtps-stack) into the rmw layer, since this is exactly the piece
 * of "ROS 2 wire convention" knowledge an rmw implementation is expected to
 * own — rmw_fastrtps_cpp and rmw_cyclonedds_cpp both do the same mangling
 * internally. No heap, no std::string, matching the rest of rosette::rtps.
 *
 *   User topics:        rt/<namespace>/<topic>
 *   Service request:    rq/<namespace>/<service>Request
 *   Service response:   rr/<namespace>/<service>Reply
 *
 * Type names follow the DDS-XTypes convention rmw_fastrtps_cpp uses so
 * real ROS 2 nodes' SEDP matching accepts us:
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

/// Composes "rt/<namespace>/<topic>" into `out` (size RTPS_MAX_TOPIC_NAME_LEN).
inline void compose_topic_name(char* out, size_t cap,
                                const char* ns, const char* topic) noexcept {
    size_t pos = 0;
    detail::str_append(out, cap, pos, "rt/");
    detail::append_namespace(out, cap, pos, ns);
    detail::str_append(out, cap, pos, topic ? topic : "");
}

/// Composes "rq/<namespace>/<service>Request".
inline void compose_service_request_topic(char* out, size_t cap,
                                           const char* ns, const char* service) noexcept {
    size_t pos = 0;
    detail::str_append(out, cap, pos, "rq/");
    detail::append_namespace(out, cap, pos, ns);
    detail::str_append(out, cap, pos, service ? service : "");
    detail::str_append(out, cap, pos, "Request");
}

/// Composes "rr/<namespace>/<service>Reply".
inline void compose_service_reply_topic(char* out, size_t cap,
                                         const char* ns, const char* service) noexcept {
    size_t pos = 0;
    detail::str_append(out, cap, pos, "rr/");
    detail::append_namespace(out, cap, pos, ns);
    detail::str_append(out, cap, pos, service ? service : "");
    detail::str_append(out, cap, pos, "Reply");
}

// TODO: type-name mangling ("<pkg>::msg::dds_::<Type>_") needs the
// package/message name apart from the C symbol name that
// rosidl_message_type_support_t carries. Confirm exactly what string
// rosidl_typesupport_microxrcedds_c exposes for this (it may already be
// the mangled DDS form, in which case this function is a passthrough) —
// see the type_support.hpp TODO.
inline void compose_dds_type_name(char* /*out*/, size_t /*cap*/,
                                   const void* /*rosidl_message_type_support*/) noexcept {
    // Deliberately unimplemented — depends on the rosidl typesupport
    // struct layout, see type_support.hpp.
}

}  // namespace rmw_rosette
