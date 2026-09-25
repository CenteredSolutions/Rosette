/**
 * @file rtps/endpoint_match_table.cpp
 * @brief EndpointMatchTable: local and remote endpoint records and matching
 *        (§8.5.5.1, OMG formal/2022-04-01).
 */

#include "endpoint_match_table.hpp"

#include "log.hpp"

namespace rtps {

uint64_t topic_type_key(const char* topic, const char* type) noexcept
{
    constexpr uint64_t kOffsetBasis = 0xCBF29CE484222325ull;
    constexpr uint64_t kPrime       = 0x00000100000001B3ull;
    uint64_t h = kOffsetBasis;
    auto mix = [&h](const char* s) noexcept {
        for (; s && *s; ++s) {
            h ^= static_cast<uint8_t>(*s);
            h *= kPrime;
        }
        h *= kPrime;      // A zero octet after each name: XOR with 0, then multiply.
    };
    mix(topic);
    mix(type);
    return h;
}

// ------------------------------------------------------------
// Record lookup and storage
// ------------------------------------------------------------

EndpointMatchTable::LocalEndpoint* EndpointMatchTable::find(LocalEndpoint* table, uint32_t n,
                                                            const Guid& g) noexcept
{
    LocalEndpoint* free_slot = nullptr;
    for (uint32_t i = 0; i < n; ++i) {
        if (table[i].guid == g) return &table[i];
        if (!free_slot && !table[i].valid()) free_slot = &table[i];
    }
    return g == GUID_UNKNOWN ? free_slot : nullptr;
}

EndpointMatchTable::RemoteEndpoint* EndpointMatchTable::find(RemoteEndpoint* table, uint32_t n,
                                                             const Guid& g) noexcept
{
    RemoteEndpoint* free_slot = nullptr;
    for (uint32_t i = 0; i < n; ++i) {
        if (table[i].guid == g) return &table[i];
        if (!free_slot && !table[i].valid()) free_slot = &table[i];
    }
    return g == GUID_UNKNOWN ? free_slot : nullptr;
}

void EndpointMatchTable::store_remote(RemoteEndpoint& slot, const Guid& guid, uint64_t key,
                                      ReliabilityKind reliability, DurabilityKind durability,
                                      bool expects_inline_qos, const Locator* locators,
                                      uint8_t locator_count) noexcept
{
    slot = RemoteEndpoint{};
    slot.guid             = guid;
    slot.key              = key;
    slot.reliability      = static_cast<uint8_t>(reliability);
    slot.durability       = static_cast<uint8_t>(durability);
    slot.expectsInlineQos = expects_inline_qos;
    const uint8_t n = locator_count < RTPS_MAX_REMOTE_ENDPOINT_LOCATORS
                      ? locator_count : RTPS_MAX_REMOTE_ENDPOINT_LOCATORS;
    for (uint8_t i = 0; i < n; ++i) slot.locators[i] = locators[i];
    slot.locatorCount = n;
}

// ------------------------------------------------------------
// Matching
// ------------------------------------------------------------

void EndpointMatchTable::add_writer(const DiscoveredWriterData& wd) noexcept
{
    if (!wd.is_valid()) return;
    const uint64_t key = topic_type_key(wd.topicName.str, wd.typeName.str);

    if (is_local(wd.writerGuid)) {
        LocalEndpoint* w = find(local_writers_, RTPS_MAX_LOCAL_ENDPOINTS, wd.writerGuid);
        if (!w) w = find(local_writers_, RTPS_MAX_LOCAL_ENDPOINTS, GUID_UNKNOWN);
        if (!w) {
            RTPS_LOG_W("match", "local writer table full (RTPS_MAX_LOCAL_ENDPOINTS=%d)",
                       RTPS_MAX_LOCAL_ENDPOINTS);
            return;
        }
        *w = LocalEndpoint{wd.writerGuid, wd.topicName, wd.typeName, key,
                           wd.reliabilityKind, wd.durabilityKind, false};
        const MatchedEndpoint writer{w->guid, nullptr, 0u, false};
        for (const LocalEndpoint& r : local_readers_) {
            if (!r.valid() || r.topicName != w->topicName || r.typeName != w->typeName) continue;
            if (!qos_compatible(w->reliability, w->durability, r.reliability, r.durability)) continue;
            report(writer, MatchedEndpoint{r.guid, nullptr, 0u, r.expectsInlineQos}, w->topicName);
        }
        for (const RemoteEndpoint& r : remote_readers_) {
            if (!r.valid() || r.key != key) continue;
            if (!qos_compatible(w->reliability, w->durability,
                                r.reliability_kind(), r.durability_kind())) continue;
            report(writer, r.matched(), w->topicName);
        }
        return;
    }

    RemoteEndpoint* w = find(remote_writers_, RTPS_MAX_DISCOVERED_WRITERS, wd.writerGuid);
    if (!w) w = find(remote_writers_, RTPS_MAX_DISCOVERED_WRITERS, GUID_UNKNOWN);
    if (!w) {
        RTPS_LOG_W("match", "remote writer table full (RTPS_MAX_DISCOVERED_WRITERS=%d)",
                   RTPS_MAX_DISCOVERED_WRITERS);
        return;
    }
    store_remote(*w, wd.writerGuid, key, wd.reliabilityKind, wd.durabilityKind, false,
                 wd.unicastLocatorList, wd.unicastLocatorCount);
    for (const LocalEndpoint& r : local_readers_) {
        if (!r.valid() || r.topicName != wd.topicName || r.typeName != wd.typeName) continue;
        if (!qos_compatible(wd.reliabilityKind, wd.durabilityKind,
                            r.reliability, r.durability)) continue;
        report(w->matched(), MatchedEndpoint{r.guid, nullptr, 0u, r.expectsInlineQos},
               r.topicName);
    }
}

void EndpointMatchTable::add_reader(const DiscoveredReaderData& rd) noexcept
{
    if (!rd.is_valid()) return;
    const uint64_t key = topic_type_key(rd.topicName.str, rd.typeName.str);

    if (is_local(rd.readerGuid)) {
        LocalEndpoint* r = find(local_readers_, RTPS_MAX_LOCAL_ENDPOINTS, rd.readerGuid);
        if (!r) r = find(local_readers_, RTPS_MAX_LOCAL_ENDPOINTS, GUID_UNKNOWN);
        if (!r) {
            RTPS_LOG_W("match", "local reader table full (RTPS_MAX_LOCAL_ENDPOINTS=%d)",
                       RTPS_MAX_LOCAL_ENDPOINTS);
            return;
        }
        *r = LocalEndpoint{rd.readerGuid, rd.topicName, rd.typeName, key,
                           rd.reliabilityKind, rd.durabilityKind, rd.expectsInlineQos};
        const MatchedEndpoint reader{r->guid, nullptr, 0u, r->expectsInlineQos};
        for (const LocalEndpoint& w : local_writers_) {
            if (!w.valid() || w.topicName != r->topicName || w.typeName != r->typeName) continue;
            if (!qos_compatible(w.reliability, w.durability, r->reliability, r->durability)) continue;
            report(MatchedEndpoint{w.guid, nullptr, 0u, false}, reader, r->topicName);
        }
        for (const RemoteEndpoint& w : remote_writers_) {
            if (!w.valid() || w.key != key) continue;
            if (!qos_compatible(w.reliability_kind(), w.durability_kind(),
                                r->reliability, r->durability)) continue;
            report(w.matched(), reader, r->topicName);
        }
        return;
    }

    RemoteEndpoint* r = find(remote_readers_, RTPS_MAX_DISCOVERED_READERS, rd.readerGuid);
    if (!r) r = find(remote_readers_, RTPS_MAX_DISCOVERED_READERS, GUID_UNKNOWN);
    if (!r) {
        RTPS_LOG_W("match", "remote reader table full (RTPS_MAX_DISCOVERED_READERS=%d)",
                   RTPS_MAX_DISCOVERED_READERS);
        return;
    }
    store_remote(*r, rd.readerGuid, key, rd.reliabilityKind, rd.durabilityKind,
                 rd.expectsInlineQos, rd.unicastLocatorList, rd.unicastLocatorCount);
    for (const LocalEndpoint& w : local_writers_) {
        if (!w.valid() || w.topicName != rd.topicName || w.typeName != rd.typeName) continue;
        if (!qos_compatible(w.reliability, w.durability,
                            rd.reliabilityKind, rd.durabilityKind)) continue;
        report(MatchedEndpoint{w.guid, nullptr, 0u, false}, r->matched(), w.topicName);
    }
}

void EndpointMatchTable::report(const MatchedEndpoint& writer, const MatchedEndpoint& reader,
                                const TopicName& topic) noexcept
{
    MatchRecord* free_slot = nullptr;
    for (MatchRecord& m : matches_) {
        if (m.writer_guid == writer.guid && m.reader_guid == reader.guid) return;
        if (!free_slot && !m.active()) free_slot = &m;
    }
    // Without a free record the pair is not reported, so every reported
    // match has a matching unmatch.
    if (!free_slot) {
        RTPS_LOG_W("match", "match record table full (RTPS_MAX_MATCH_RECORDS=%lu)",
                   (unsigned long)kMaxMatches);
        return;
    }
    free_slot->writer_guid = writer.guid;
    free_slot->reader_guid = reader.guid;
    if (on_match_) on_match_(writer, reader, topic, user_data_);
}

// ------------------------------------------------------------
// Removal
// ------------------------------------------------------------

void EndpointMatchTable::dissolve(const Guid& guid, bool is_writer) noexcept
{
    for (MatchRecord& m : matches_) {
        if (!m.active()) continue;
        if ((is_writer ? m.writer_guid : m.reader_guid) != guid) continue;
        const MatchRecord gone = m;
        m = MatchRecord{};
        if (on_unmatch_) on_unmatch_(gone.writer_guid, gone.reader_guid, user_data_);
    }
}

void EndpointMatchTable::remove_writer(const Guid& writer_guid) noexcept
{
    dissolve(writer_guid, /*is_writer=*/true);
    if (LocalEndpoint* w = find(local_writers_, RTPS_MAX_LOCAL_ENDPOINTS, writer_guid))
        *w = LocalEndpoint{};
    if (RemoteEndpoint* w = find(remote_writers_, RTPS_MAX_DISCOVERED_WRITERS, writer_guid))
        *w = RemoteEndpoint{};
}

void EndpointMatchTable::remove_reader(const Guid& reader_guid) noexcept
{
    dissolve(reader_guid, /*is_writer=*/false);
    if (LocalEndpoint* r = find(local_readers_, RTPS_MAX_LOCAL_ENDPOINTS, reader_guid))
        *r = LocalEndpoint{};
    if (RemoteEndpoint* r = find(remote_readers_, RTPS_MAX_DISCOVERED_READERS, reader_guid))
        *r = RemoteEndpoint{};
}

void EndpointMatchTable::remove_participant(const GuidPrefix& gone_prefix) noexcept
{
    // Copy each GUID: removal clears the record it lives in.
    for (const RemoteEndpoint& w : remote_writers_) {
        if (!w.valid() || w.guid.prefix != gone_prefix) continue;
        const Guid guid = w.guid;
        remove_writer(guid);
    }
    for (const RemoteEndpoint& r : remote_readers_) {
        if (!r.valid() || r.guid.prefix != gone_prefix) continue;
        const Guid guid = r.guid;
        remove_reader(guid);
    }
}

}  // namespace rtps
