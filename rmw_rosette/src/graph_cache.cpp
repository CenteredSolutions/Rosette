// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file graph_cache.cpp
 * @brief GraphCache: endpoint name storage and ROS 2 name demangling.
 */

#include "rmw_rosette/graph_cache.hpp"

#include <cstring>
#include <new>

namespace rtps = rosette::rtps;

namespace rmw_rosette {
namespace {

constexpr char kTopicPrefix[] = "rt/";
constexpr char kRequestPrefix[] = "rq/";
constexpr char kReplyPrefix[] = "rr/";
constexpr char kRequestSuffix[] = "Request";
constexpr char kReplySuffix[] = "Reply";
constexpr char kTypeNamespace[] = "dds_::";
constexpr char kRequestTypeSuffix[] = "_Request_";
constexpr char kResponseTypeSuffix[] = "_Response_";

bool starts_with(const std::string& s, const char* prefix) {
    return s.rfind(prefix, 0) == 0;
}

bool ends_with(const std::string& s, const char* suffix) {
    const size_t n = std::strlen(suffix);
    return s.size() >= n && s.compare(s.size() - n, n, suffix) == 0;
}

/**
 * @brief Removes the `dds_::` namespace and writes `::` as `/`.
 *
 * @param[in] type DDS type name with the trailing marker already removed.
 * @return The ROS type name, or empty when `type` is not a ROS 2 type.
 */
std::string ros_type_from(const std::string& type) {
    const size_t at = type.find(kTypeNamespace);
    if (at == std::string::npos) {
        return {};
    }

    std::string out = type;
    out.erase(at, std::strlen(kTypeNamespace));

    size_t pos = 0;
    while ((pos = out.find("::", pos)) != std::string::npos) {
        out.replace(pos, 2, "/");
        ++pos;
    }

    return out;
}

/// @brief Demangles `pkg::msg::dds_::Name_`; empty when it is not a ROS 2 message type.
std::string demangle_message_type(const std::string& type) {
    if (!ends_with(type, "_")) {
        return {};
    }

    return ros_type_from(type.substr(0, type.size() - 1));
}

/// @brief Demangles a request or response type to its service type; empty for any other type.
std::string demangle_service_type(const std::string& type) {
    for (const char* suffix : {kRequestTypeSuffix, kResponseTypeSuffix}) {
        if (ends_with(type, suffix)) {
            return ros_type_from(type.substr(0, type.size() - std::strlen(suffix)));
        }
    }

    return {};
}

/// @brief Demangles `rq/<name>Request` or `rr/<name>Reply`; empty for any other topic.
std::string demangle_service_topic(const std::string& topic) {
    const char* suffix = nullptr;
    if (starts_with(topic, kRequestPrefix)) {
        suffix = kRequestSuffix;
    } else if (starts_with(topic, kReplyPrefix)) {
        suffix = kReplySuffix;
    }

    if (suffix == nullptr || !ends_with(topic, suffix)) {
        return {};
    }

    const size_t prefix_len = std::strlen(kRequestPrefix);
    const size_t suffix_len = std::strlen(suffix);
    if (topic.size() <= prefix_len + suffix_len) {
        return {};
    }

    return "/" + topic.substr(prefix_len, topic.size() - prefix_len - suffix_len);
}

void on_endpoint_observed(void* self, const rtps::Guid& guid, const char* topic, const char* type,
                          bool is_writer) noexcept {
    // An endpoint that cannot be stored stays out of the listing; discovery goes on.
    try {
        (void)static_cast<GraphCache*>(self)->add(guid, topic, type, is_writer);
    } catch (const std::bad_alloc&) {
        return;
    }
}

void on_endpoint_gone(void* self, const rtps::Guid& guid) noexcept {
    (void)static_cast<GraphCache*>(self)->remove(guid);
}

void on_participant_gone(void* self, const rtps::GuidPrefix& prefix) noexcept {
    (void)static_cast<GraphCache*>(self)->remove_participant(prefix);
}

}  // namespace

GraphCache::GraphCache() noexcept
    : observer_{this, on_endpoint_observed, on_endpoint_gone, on_participant_gone} {}

GraphCache::GuidBytes GraphCache::key_of(const rtps::Guid& guid) noexcept {
    static_assert(sizeof(rtps::Guid) == sizeof(GuidBytes), "a GUID is 16 octets, prefix first");
    GuidBytes key;
    std::memcpy(key.data(), &guid, key.size());
    return key;
}

void GraphCache::changed() const noexcept {
    if (on_change_ != nullptr) {
        on_change_(on_change_user_);
    }
}

bool GraphCache::add(const rtps::Guid& guid, const char* topic, const char* type, bool is_writer) {
    const GuidBytes key = key_of(guid);
    const auto      found = endpoints_.find(key);
    if (found != endpoints_.end() && found->second.is_writer == is_writer
        && found->second.topic == topic && found->second.type == type) {
        return false;
    }

    Endpoint endpoint;
    endpoint.topic = topic;
    endpoint.type = type;
    endpoint.is_writer = is_writer;
    endpoints_[key] = std::move(endpoint);
    changed();
    return true;
}

bool GraphCache::remove(const rtps::Guid& guid) noexcept {
    if (endpoints_.erase(key_of(guid)) == 0u) {
        return false;
    }

    changed();
    return true;
}

bool GraphCache::remove_participant(const rtps::GuidPrefix& prefix) noexcept {
    // Keys sort by prefix first, so one participant's endpoints are adjacent.
    GuidBytes low{};
    std::memcpy(low.data(), prefix.value, sizeof(prefix.value));

    bool removed = false;
    auto it = endpoints_.lower_bound(low);
    while (it != endpoints_.end()
           && std::memcmp(it->first.data(), prefix.value, sizeof(prefix.value)) == 0) {
        it = endpoints_.erase(it);
        removed = true;
    }

    if (removed) {
        changed();
    }

    return removed;
}

NamesAndTypes GraphCache::topic_names_and_types(bool no_demangle) const {
    NamesAndTypes out;
    for (const auto& entry : endpoints_) {
        const Endpoint& endpoint = entry.second;
        if (no_demangle) {
            out[endpoint.topic].insert(endpoint.type);
            continue;
        }

        if (!starts_with(endpoint.topic, kTopicPrefix)) {
            continue;
        }

        const std::string type = demangle_message_type(endpoint.type);
        if (!type.empty()) {
            out[endpoint.topic.substr(2)].insert(type);
        }
    }

    return out;
}

NamesAndTypes GraphCache::service_names_and_types() const {
    NamesAndTypes out;
    for (const auto& entry : endpoints_) {
        const std::string name = demangle_service_topic(entry.second.topic);
        const std::string type = demangle_service_type(entry.second.type);
        if (!name.empty() && !type.empty()) {
            out[name].insert(type);
        }
    }

    return out;
}

}  // namespace rmw_rosette
