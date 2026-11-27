// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file graph_cache.cpp
 * @brief GraphCache: endpoint name storage and ROS 2 name demangling.
 */

#include "rmw_rosette/graph_cache.hpp"

#include <cstring>
#include <new>
#include <utility>

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

/// @brief Demangles a message type, or keeps a type that is not a ROS 2 message type as
/// announced, as rmw_fastrtps_cpp's `_demangle_if_ros_type` does.
std::string demangle_if_ros_type(const std::string& type) {
    std::string demangled = demangle_message_type(type);
    return demangled.empty() ? type : demangled;
}

/// @brief Demangles `<prefix><name><suffix>` to `/<name>`; empty for any other topic.
std::string demangle_service_topic(const std::string& topic, const char* prefix,
                                   const char* suffix) {
    const size_t prefix_len = std::strlen(prefix);
    const size_t suffix_len = std::strlen(suffix);
    if (!starts_with(topic, prefix) || !ends_with(topic, suffix)
        || topic.size() <= prefix_len + suffix_len) {
        return {};
    }

    return "/" + topic.substr(prefix_len, topic.size() - prefix_len - suffix_len);
}

/// @brief Demangles `rq/<name>Request` or `rr/<name>Reply`; empty for any other topic.
std::string demangle_service_topic(const std::string& topic) {
    std::string name = demangle_service_topic(topic, kRequestPrefix, kRequestSuffix);
    return name.empty() ? demangle_service_topic(topic, kReplyPrefix, kReplySuffix) : name;
}

void on_endpoint_observed(void* self, const rtps::Guid& guid, const char* topic, const char* type,
                          bool is_writer, rtps::ReliabilityKind reliability,
                          rtps::DurabilityKind durability) noexcept {
    // An endpoint that cannot be stored stays out of the listing; discovery goes on.
    try {
        (void)static_cast<GraphCache*>(self)->add(guid, topic, type, is_writer, reliability,
                                                  durability);
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

void on_participant_refreshing(void* self, const rtps::GuidPrefix& prefix) noexcept {
    static_cast<GraphCache*>(self)->begin_refresh(prefix);
}

void on_participant_refreshed(void* self, const rtps::GuidPrefix& prefix) noexcept {
    (void)static_cast<GraphCache*>(self)->end_refresh(prefix);
}

}  // namespace

GraphCache::GraphCache() noexcept
    : observer_{this,
                on_endpoint_observed,
                on_endpoint_gone,
                on_participant_gone,
                on_participant_refreshing,
                on_participant_refreshed} {}

GraphCache::GuidBytes GraphCache::key_of(const rtps::Guid& guid) noexcept {
    static_assert(sizeof(rtps::Guid) == sizeof(GuidBytes), "a GUID is 16 octets, prefix first");
    GuidBytes key;
    std::memcpy(key.data(), &guid, key.size());
    return key;
}

GraphCache::PrefixBytes GraphCache::key_of(const rtps::GuidPrefix& prefix) noexcept {
    PrefixBytes key;
    std::memcpy(key.data(), prefix.value, key.size());
    return key;
}

void GraphCache::changed() const noexcept {
    if (on_change_ != nullptr) {
        on_change_(on_change_user_);
    }
}

bool GraphCache::add(const rtps::Guid& guid, const char* topic, const char* type, bool is_writer,
                     rtps::ReliabilityKind reliability, rtps::DurabilityKind durability) {
    const GuidBytes key = key_of(guid);
    const auto      found = endpoints_.find(key);
    if (found != endpoints_.end()) {
        // Confirmed before the copy below, which may throw: an endpoint the
        // replay announced is never forgotten by end_refresh(), even when
        // its new names could not be stored.
        const Endpoint& known = found->second;
        found->second.confirmed = true;
        if (known.is_writer == is_writer && known.topic == topic && known.type == type
            && known.reliability == reliability && known.durability == durability) {
            return false;
        }
    }

    Endpoint endpoint;
    endpoint.topic = topic;
    endpoint.type = type;
    endpoint.is_writer = is_writer;
    endpoint.reliability = reliability;
    endpoint.durability = durability;
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

GraphCache::Endpoints::iterator GraphCache::first_of(const rtps::GuidPrefix& prefix) noexcept {
    // Keys sort by prefix first, so one participant's endpoints are adjacent.
    GuidBytes low{};
    std::memcpy(low.data(), prefix.value, sizeof(prefix.value));
    return endpoints_.lower_bound(low);
}

bool GraphCache::belongs_to(Endpoints::const_iterator it,
                            const rtps::GuidPrefix&   prefix) const noexcept {
    return it != endpoints_.end()
           && std::memcmp(it->first.data(), prefix.value, sizeof(prefix.value)) == 0;
}

bool GraphCache::remove_participant(const rtps::GuidPrefix& prefix) noexcept {
    bool removed = false;
    auto it = first_of(prefix);
    while (belongs_to(it, prefix)) {
        it = endpoints_.erase(it);
        removed = true;
    }

    if (nodes_.erase(key_of(prefix)) != 0u) {
        removed = true;
    }

    if (removed) {
        changed();
    }

    return removed;
}

void GraphCache::begin_refresh(const rtps::GuidPrefix& prefix) noexcept {
    for (auto it = first_of(prefix); belongs_to(it, prefix); ++it) {
        it->second.confirmed = false;
    }
}

bool GraphCache::end_refresh(const rtps::GuidPrefix& prefix) noexcept {
    bool removed = false;
    auto it = first_of(prefix);
    while (belongs_to(it, prefix)) {
        if (it->second.confirmed) {
            ++it;
            continue;
        }

        it = endpoints_.erase(it);
        removed = true;
    }

    if (removed) {
        changed();
    }

    return removed;
}

bool GraphCache::set_participant_nodes(const rtps::GuidPrefix& prefix,
                                       std::vector<NodeEntities> nodes) {
    const PrefixBytes key = key_of(prefix);
    const auto        found = nodes_.find(key);
    if (found == nodes_.end() ? nodes.empty() : found->second == nodes) {
        return false;
    }

    if (nodes.empty()) {
        nodes_.erase(found);
    } else {
        nodes_[key] = std::move(nodes);
    }

    changed();
    return true;
}

std::vector<NodeName> GraphCache::node_names() const {
    std::vector<NodeName> out;
    for (const auto& participant : nodes_) {
        for (const NodeEntities& entities : participant.second) {
            out.push_back(entities.node);
        }
    }

    return out;
}

const NodeEntities* GraphCache::find_node(const NodeName& node) const noexcept {
    for (const auto& participant : nodes_) {
        for (const NodeEntities& entities : participant.second) {
            if (entities.node == node) {
                return &entities;
            }
        }
    }

    return nullptr;
}

const NodeName* GraphCache::owner_of(const rtps::Guid& guid) const noexcept {
    // A participant lists only its own endpoints, so only its entry is searched.
    const auto participant = nodes_.find(key_of(guid.prefix));
    if (participant == nodes_.end()) {
        return nullptr;
    }

    for (const NodeEntities& entities : participant->second) {
        for (const rtps::Guid& endpoint : entities.endpoints) {
            if (endpoint == guid) {
                return &entities.node;
            }
        }
    }

    return nullptr;
}

NamesAndTypes GraphCache::names_and_types_of(const std::vector<rtps::Guid>& endpoints,
                                             EndpointRole role, bool no_demangle) const {
    const bool is_writer = role == EndpointRole::publisher;
    NamesAndTypes out;
    for (const rtps::Guid& guid : endpoints) {
        const auto found = endpoints_.find(key_of(guid));
        if (found == endpoints_.end() || found->second.is_writer != is_writer) {
            continue;
        }

        const Endpoint& endpoint = found->second;
        std::string name;
        std::string type;
        if (role == EndpointRole::server || role == EndpointRole::client) {
            name = role == EndpointRole::server
                       ? demangle_service_topic(endpoint.topic, kRequestPrefix, kRequestSuffix)
                       : demangle_service_topic(endpoint.topic, kReplyPrefix, kReplySuffix);
            type = demangle_service_type(endpoint.type);
        } else if (no_demangle) {
            name = endpoint.topic;
            type = endpoint.type;
        } else if (starts_with(endpoint.topic, kTopicPrefix)) {
            name = endpoint.topic.substr(2);
            type = demangle_if_ros_type(endpoint.type);
        }

        if (!name.empty() && !type.empty()) {
            out[name].insert(type);
        }
    }

    return out;
}

std::vector<TopicEndpoint> GraphCache::endpoints_on(const char* topic, bool is_writer,
                                                    bool no_demangle) const {
    std::vector<TopicEndpoint> out;
    for (const auto& entry : endpoints_) {
        const Endpoint& endpoint = entry.second;
        if (endpoint.is_writer != is_writer || endpoint.topic != topic) {
            continue;
        }

        TopicEndpoint info;
        std::memcpy(&info.guid, entry.first.data(), entry.first.size());
        info.type = no_demangle ? endpoint.type : demangle_if_ros_type(endpoint.type);
        info.reliability = endpoint.reliability;
        info.durability = endpoint.durability;
        out.push_back(std::move(info));
    }

    return out;
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

        out[endpoint.topic.substr(2)].insert(demangle_if_ros_type(endpoint.type));
    }

    return out;
}

size_t GraphCache::count(const char* topic, bool is_writer) const noexcept {
    size_t n = 0u;
    for (const auto& entry : endpoints_) {
        if (entry.second.is_writer == is_writer && entry.second.topic == topic) {
            ++n;
        }
    }

    return n;
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
