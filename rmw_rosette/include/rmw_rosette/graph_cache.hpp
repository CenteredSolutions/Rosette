/**
 * @file rmw_rosette/graph_cache.hpp
 * @brief Backing store for rmw_get_{node,topic,service}_names_and_types etc.
 *
 * rtps::EndpointMatchTable (endpoint_match_table.hpp) already tracks every
 * discovered DiscoveredWriterData/DiscoveredReaderData, but its writers_/
 * readers_ tables are private — it's built purely for matching, with no
 * iteration API for the outside world, and reaching in would mean
 * changing an already-conformance-tested class for an rmw-only need.
 *
 * Instead this keeps its own small parallel cache, fed by the same
 * discovery events RTPSParticipant already dispatches internally
 * (on_endpoint_match/on_endpoint_unmatch, on_spdp_new_peer/removed — see
 * participant.hpp's private callback trampolines). rmw_init.cpp's Context
 * setup will need its own copies of those callbacks that update both the
 * EndpointMatchTable *and* this cache, or (cleaner) EndpointMatchTable
 * gains an optional second observer. Not decided yet — flagging the
 * tension rather than picking prematurely.
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

/// Flat table of every remote endpoint seen so far, for
/// rmw_get_topic_names_and_types / rmw_count_publishers /
/// rmw_count_subscribers. Local endpoints (this process's own publishers/
/// subscriptions) should be included too — TODO: decide whether
/// rmw_create_publisher/subscription register themselves here directly or
/// whether SEDP loops discovery back locally (check whether
/// SedpWriter/SedpReader already do this for the local participant).
class GraphCache {
public:
    void add(const rtps::Guid& guid, const rtps::TopicName& topic,
             const rtps::TopicName& type, bool is_writer) noexcept {
        GraphEndpointEntry* slot = find_free();
        if (!slot) return;  // TODO: log-and-drop vs. grow; capacity is fixed for now.
        slot->guid = guid;
        slot->topic_name = topic;
        slot->type_name = type;
        slot->is_writer = is_writer;
        slot->valid = true;
    }

    void remove(const rtps::Guid& guid) noexcept {
        for (auto& e : entries_) {
            if (e.valid && e.guid == guid) e = GraphEndpointEntry{};
        }
    }

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
