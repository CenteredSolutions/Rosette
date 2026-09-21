/**
 * @file rmw_rosette/graph_cache.hpp
 * @brief Backing store for rmw_get_{node,topic,service}_names_and_types etc.
 *
 * rtps::EndpointMatchTable (endpoint_match_table.hpp) already tracks every
 * discovered DiscoveredWriterData/DiscoveredReaderData, but its writers_/
 * readers_ tables are private, built purely for matching, with no
 * iteration API for the outside world. GraphCache is a small parallel
 * table purpose-built for the query surface rmw_graph.cpp needs instead.
 *
 * GraphCache itself only stores and iterates entries; the caller is
 * responsible for feeding it from discovery events (RTPSParticipant's
 * on_endpoint_match/on_endpoint_unmatch and on_spdp_new_peer/removed
 * callback trampolines — see participant.hpp) and for deciding whether
 * this participant's own local publishers/subscriptions register
 * themselves directly or arrive the same way remote endpoints do, via
 * SEDP's loopback to the local participant.
 *
 * No heap: fixed capacity, same posture as the rest of rosette::rtps.
 */

#pragma once

#include "rosette/rtps/endpoint_match_table.hpp"
#include "rosette/rtps/types.hpp"

#include <cstdint>

#ifndef RMW_ROSETTE_MAX_GRAPH_ENTRIES
#  define RMW_ROSETTE_MAX_GRAPH_ENTRIES 32
#endif

namespace rmw_rosette {

struct GraphEndpointEntry {
    rtps::Guid       guid;
    rtps::TopicName  topic_name;
    rtps::TopicName  type_name;
    bool             is_writer = false;
    bool             valid     = false;
};

/// Flat table of every endpoint (local or remote) known to the graph, for
/// `rmw_get_topic_names_and_types()`/`rmw_count_publishers()`/
/// `rmw_count_subscribers()` to query via for_each().
class GraphCache {
public:
    /**
     * @brief Records one endpoint in the first free slot.
     *
     * @param[in] guid RTPS GUID of the endpoint.
     * @param[in] topic Mangled RTPS topic name (see topic_naming.hpp).
     * @param[in] type DDS wire type name (see compose_dds_type_name()).
     * @param[in] is_writer `true` for a writer/publisher, `false` for a
     *   reader/subscription.
     *
     * Silently drops the entry when the table is full — a full table
     * degrades graph queries (some entries become invisible) rather than
     * failing the endpoint creation that triggered this call.
     */
    void add(const rtps::Guid& guid, const rtps::TopicName& topic,
             const rtps::TopicName& type, bool is_writer) noexcept {
        GraphEndpointEntry* slot = find_free();
        if (!slot) return;
        slot->guid = guid;
        slot->topic_name = topic;
        slot->type_name = type;
        slot->is_writer = is_writer;
        slot->valid = true;
    }

    /**
     * @brief Removes every entry matching a GUID.
     *
     * @param[in] guid RTPS GUID of the endpoint (or, for a departed
     *   participant, matched against each entry's GuidPrefix by the
     *   caller before invoking this per-endpoint).
     */
    void remove(const rtps::Guid& guid) noexcept {
        for (auto& e : entries_) {
            if (e.valid && e.guid == guid) e = GraphEndpointEntry{};
        }
    }

    /**
     * @brief Invokes `fn` once for every valid entry.
     *
     * @param[in] fn Callable taking `const GraphEndpointEntry&`.
     */
    template <typename Fn>
    void for_each(Fn&& fn) const noexcept {
        for (const auto& e : entries_) {
            if (e.valid) fn(e);
        }
    }

private:
    GraphEndpointEntry* find_free() noexcept {
        for (auto& e : entries_) {
            if (!e.valid) return &e;
        }
        return nullptr;
    }

    GraphEndpointEntry entries_[RMW_ROSETTE_MAX_GRAPH_ENTRIES] = {};
};

}  // namespace rmw_rosette
