/**
 * @file ros/topic_names.hpp
 * @brief ROS 2 DDS topic-name and type-name composition utilities.
 *
 * All compose_* functions follow the ROS 2 DDS naming conventions:
 *
 *   User topics:        rt/<namespace>/<topic>
 *   Service request:    rq/<namespace>/<service>Request
 *   Service response:   rr/<namespace>/<service>Reply
 *   Action sub-topics:  rq|rr|rt/<namespace>/<action>/_action/<suffix>
 *
 * Low-level string helpers live in milkros::detail and operate on a
 * fixed-size char buffer with a position cursor — no heap, no std::string.
 */

#pragma once

#include "rosette/rtps/endpoint_match_table.hpp"  // rtps::TopicName, RTPS_MAX_TOPIC_NAME_LEN

#include <stddef.h>
#include <string.h>

namespace milkros {

// ============================================================
// detail — internal string primitives
// ============================================================

namespace detail {

/// Append src to dst[pos..cap-1], null-terminate.
inline void str_append(char* dst, size_t cap,
                        size_t& pos, const char* src) noexcept {
    if (!dst || !src || cap == 0u) return;
    while (*src && pos + 1u < cap) { dst[pos++] = *src++; }
    dst[pos] = '\0';
}

/// Append a path segment, collapsing runs of '/' and stripping
/// leading and trailing slashes.
inline void path_append(char* dst, size_t cap,
                         size_t& pos, const char* src) noexcept {
    if (!src) return;
    while (*src == '/') ++src;               // strip leading slashes
    while (*src && pos + 1u < cap) {
        if (*src == '/' && (pos == 0u || dst[pos - 1u] == '/')) {
            ++src;                           // collapse double slashes
            continue;
        }
        dst[pos++] = *src++;
    }
    while (pos > 0u && dst[pos - 1u] == '/') --pos;  // strip trailing slash
    dst[pos] = '\0';
}

/// Append a slash separator if dst doesn't already end with one.
inline void ensure_slash(char* dst, size_t cap, size_t& pos) noexcept {
    if (pos > 0u && dst[pos - 1u] != '/' && pos + 1u < cap) {
        dst[pos++] = '/';
        dst[pos]   = '\0';
    }
}

/// Write <base><suffix> into out[0..cap-1], null-terminated.
/// Used for building action sub-service type names.
inline void build_type_name(char* out, size_t cap,
                              const char* base,
                              const char* suffix) noexcept {
    size_t pos = 0u;
    str_append(out, cap, pos, base);
    str_append(out, cap, pos, suffix);
}

/// Append the namespace component (with trailing slash) to a topic buffer.
/// Does nothing when ns is empty or "/".
inline void append_namespace(char* dst, size_t cap,
                               size_t& pos, const char* ns) noexcept {
    if (!ns || ns[0] == '\0' || (ns[0] == '/' && ns[1] == '\0')) return;
    path_append(dst, cap, pos, ns);
    ensure_slash(dst, cap, pos);
}

} // namespace detail

// ============================================================
// Public compose_* free functions
// ============================================================

/**
 * @brief Compose a standard user topic name: rt/<namespace>/<topic>
 *
 * If topic already starts with "rt/" it is used as-is.
 */
inline void compose_topic_name(rtps::TopicName& out,
                                const char* node_namespace,
                                const char* topic) noexcept {
    if (!topic) { out.assign("rt/unnamed"); return; }
    if (strncmp(topic, "rt/", 3u) == 0) { out.assign(topic); return; }

    char buf[RTPS_MAX_TOPIC_NAME_LEN] = {};
    size_t pos = 0u;
    detail::str_append(buf, sizeof(buf), pos, "rt/");
    detail::append_namespace(buf, sizeof(buf), pos, node_namespace);
    detail::path_append(buf, sizeof(buf), pos, topic);
    out.assign(buf);
}

/**
 * @brief Compose a service request topic: rq/<namespace>/<service>Request
 */
inline void compose_service_request_topic(rtps::TopicName& out,
                                           const char* node_namespace,
                                           const char* service_name) noexcept {
    char buf[RTPS_MAX_TOPIC_NAME_LEN] = {};
    size_t pos = 0u;
    detail::str_append(buf, sizeof(buf), pos, "rq/");
    detail::append_namespace(buf, sizeof(buf), pos, node_namespace);
    detail::path_append(buf, sizeof(buf), pos, service_name);
    detail::str_append(buf, sizeof(buf), pos, "Request");
    out.assign(buf);
}

/**
 * @brief Compose a service response topic: rr/<namespace>/<service>Reply
 */
inline void compose_service_response_topic(rtps::TopicName& out,
                                            const char* node_namespace,
                                            const char* service_name) noexcept {
    char buf[RTPS_MAX_TOPIC_NAME_LEN] = {};
    size_t pos = 0u;
    detail::str_append(buf, sizeof(buf), pos, "rr/");
    detail::append_namespace(buf, sizeof(buf), pos, node_namespace);
    detail::path_append(buf, sizeof(buf), pos, service_name);
    detail::str_append(buf, sizeof(buf), pos, "Reply");
    out.assign(buf);
}

/**
 * @brief Compose an action sub-topic name.
 *
 * @param out             Output TopicName.
 * @param node_namespace  Node namespace (may be empty or "/").
 * @param action_name     Base action name (e.g. "fibonacci").
 * @param suffix          Sub-topic suffix, with or without leading '/'.
 *                        For service sub-topics pass the bare name
 *                        (e.g. "send_goalRequest"); for topic sub-topics
 *                        pass the path (e.g. "_action/feedback").
 * @param use_rq          Prefix with "rq/" (service request side).
 * @param use_rr          Prefix with "rr/" (service response side).
 *                        If neither flag is set, prefix with "rt/" (topic).
 *
 * Examples (namespace="/my_ns", action_name="fibonacci"):
 *   use_rq=true,  suffix="send_goalRequest"  → rq/my_ns/fibonacci/_action/send_goalRequest
 *   use_rr=true,  suffix="send_goalReply"    → rr/my_ns/fibonacci/_action/send_goalReply
 *   neither,      suffix="_action/feedback"  → rt/my_ns/fibonacci/_action/feedback
 */
inline void compose_action_subtopic(rtps::TopicName& out,
                                     const char* node_namespace,
                                     const char* action_name,
                                     const char* suffix,
                                     bool use_rq,
                                     bool use_rr) noexcept {
    const char* prefix = use_rq ? "rq/" : (use_rr ? "rr/" : "rt/");

    char buf[RTPS_MAX_TOPIC_NAME_LEN] = {};
    size_t pos = 0u;
    detail::str_append(buf, sizeof(buf), pos, prefix);
    detail::append_namespace(buf, sizeof(buf), pos, node_namespace);
    detail::path_append(buf, sizeof(buf), pos, action_name);

    // Insert separator before suffix if suffix doesn't begin with '/'.
    if (suffix && suffix[0] != '/') {
        detail::ensure_slash(buf, sizeof(buf), pos);
    }
    detail::str_append(buf, sizeof(buf), pos, suffix);
    out.assign(buf);
}

} // namespace milkros
