/**
 * @file rtps/endpoint_match_table.cpp
 * @brief EndpointMatchTable: local and remote endpoint records and matching
 *        (§8.5.5.1, OMG formal/2022-04-01).
 */

#include "endpoint_match_table.hpp"

#include "log.hpp"

namespace rtps {

uint32_t name_key(const char* name) noexcept
{
    constexpr uint32_t kOffsetBasis = 0x811C9DC5u;
    constexpr uint32_t kPrime       = 0x01000193u;
    uint32_t h = kOffsetBasis;
    for (; name && *name; ++name) {
        h ^= static_cast<uint8_t>(*name);
        h *= kPrime;
    }
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

void EndpointMatchTable::store_remote(RemoteEndpoint& slot, const Guid& guid,
                                      uint32_t topic_key, uint32_t type_key,
                                      ReliabilityKind reliability, DurabilityKind durability,
                                      bool expects_inline_qos, const Locator* locators,
                                      uint8_t locator_count) noexcept
{
    slot = RemoteEndpoint{};
    slot.guid             = guid;
    slot.topicKey         = topic_key;
    slot.typeKey          = type_key;
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
    if (!wd.topicName.fits() || !wd.typeName.fits()) {
        RTPS_LOG_W("match", "writer name longer than RTPS_MAX_TOPIC_NAME_LEN - 1, ignored");
        return;
    }
    const uint32_t topic_key = name_key(wd.topicName.str);
    const uint32_t type_key  = name_key(wd.typeName.str);

    if (is_local(wd.writerGuid)) {
        LocalEndpoint* w = find(local_writers_, RTPS_MAX_LOCAL_ENDPOINTS, wd.writerGuid);
        if (!w) w = find(local_writers_, RTPS_MAX_LOCAL_ENDPOINTS, GUID_UNKNOWN);
        if (!w) {
            RTPS_LOG_W("match", "local writer table full (RTPS_MAX_LOCAL_ENDPOINTS=%d)",
                       RTPS_MAX_LOCAL_ENDPOINTS);
            return;
        }
        const bool is_change = !w->valid() || w->topicName != wd.topicName ||
                               w->typeName != wd.typeName;
        *w = LocalEndpoint{wd.writerGuid, wd.topicName, wd.typeName, topic_key, type_key,
                           wd.reliabilityKind, wd.durabilityKind, false};
        if (is_change) changed();
        const MatchedEndpoint writer{w->guid, nullptr, 0u, false};
        for (const LocalEndpoint& r : local_readers_) {
            if (!r.valid() || r.topicName != w->topicName || r.typeName != w->typeName) continue;
            if (!qos_compatible(w->reliability, w->durability, r.reliability, r.durability)) continue;
            report(writer, MatchedEndpoint{r.guid, nullptr, 0u, r.expectsInlineQos}, w->topicName);
        }
        for (const RemoteEndpoint& r : remote_readers_) {
            if (!r.valid() || r.topicKey != topic_key || r.typeKey != type_key) continue;
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
    const bool is_change = !w->valid() || w->topicKey != topic_key || w->typeKey != type_key;
    store_remote(*w, wd.writerGuid, topic_key, type_key, wd.reliabilityKind,
                 wd.durabilityKind, false, wd.unicastLocatorList, wd.unicastLocatorCount);
    if (is_change) changed();
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
    if (!rd.topicName.fits() || !rd.typeName.fits()) {
        RTPS_LOG_W("match", "reader name longer than RTPS_MAX_TOPIC_NAME_LEN - 1, ignored");
        return;
    }
    const uint32_t topic_key = name_key(rd.topicName.str);
    const uint32_t type_key  = name_key(rd.typeName.str);

    if (is_local(rd.readerGuid)) {
        LocalEndpoint* r = find(local_readers_, RTPS_MAX_LOCAL_ENDPOINTS, rd.readerGuid);
        if (!r) r = find(local_readers_, RTPS_MAX_LOCAL_ENDPOINTS, GUID_UNKNOWN);
        if (!r) {
            RTPS_LOG_W("match", "local reader table full (RTPS_MAX_LOCAL_ENDPOINTS=%d)",
                       RTPS_MAX_LOCAL_ENDPOINTS);
            return;
        }
        const bool is_change = !r->valid() || r->topicName != rd.topicName ||
                               r->typeName != rd.typeName;
        *r = LocalEndpoint{rd.readerGuid, rd.topicName, rd.typeName, topic_key, type_key,
                           rd.reliabilityKind, rd.durabilityKind, rd.expectsInlineQos};
        if (is_change) changed();
        const MatchedEndpoint reader{r->guid, nullptr, 0u, r->expectsInlineQos};
        for (const LocalEndpoint& w : local_writers_) {
            if (!w.valid() || w.topicName != r->topicName || w.typeName != r->typeName) continue;
            if (!qos_compatible(w.reliability, w.durability, r->reliability, r->durability)) continue;
            report(MatchedEndpoint{w.guid, nullptr, 0u, false}, reader, r->topicName);
        }
        for (const RemoteEndpoint& w : remote_writers_) {
            if (!w.valid() || w.topicKey != topic_key || w.typeKey != type_key) continue;
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
    const bool is_change = !r->valid() || r->topicKey != topic_key || r->typeKey != type_key;
    store_remote(*r, rd.readerGuid, topic_key, type_key, rd.reliabilityKind,
                 rd.durabilityKind, rd.expectsInlineQos, rd.unicastLocatorList,
                 rd.unicastLocatorCount);
    if (is_change) changed();
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
    if (writer_guid == GUID_UNKNOWN) return;
    dissolve(writer_guid, /*is_writer=*/true);
    bool forgotten = false;
    if (LocalEndpoint* w = find(local_writers_, RTPS_MAX_LOCAL_ENDPOINTS, writer_guid)) {
        *w = LocalEndpoint{};
        forgotten = true;
    }
    if (RemoteEndpoint* w = find(remote_writers_, RTPS_MAX_DISCOVERED_WRITERS, writer_guid)) {
        *w = RemoteEndpoint{};
        forgotten = true;
    }
    if (forgotten) changed();
}

void EndpointMatchTable::remove_reader(const Guid& reader_guid) noexcept
{
    if (reader_guid == GUID_UNKNOWN) return;
    dissolve(reader_guid, /*is_writer=*/false);
    bool forgotten = false;
    if (LocalEndpoint* r = find(local_readers_, RTPS_MAX_LOCAL_ENDPOINTS, reader_guid)) {
        *r = LocalEndpoint{};
        forgotten = true;
    }
    if (RemoteEndpoint* r = find(remote_readers_, RTPS_MAX_DISCOVERED_READERS, reader_guid)) {
        *r = RemoteEndpoint{};
        forgotten = true;
    }
    if (forgotten) changed();
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

// ------------------------------------------------------------
// Graph queries
// ------------------------------------------------------------

uint32_t EndpointMatchTable::count(const char* topic, bool writers) const noexcept
{
    if (!topic) return 0u;
    const LocalEndpoint*  locals  = writers ? local_writers_ : local_readers_;
    const RemoteEndpoint* remotes = writers ? remote_writers_ : remote_readers_;
    const uint32_t n_remote = writers ? RTPS_MAX_DISCOVERED_WRITERS : RTPS_MAX_DISCOVERED_READERS;
    const uint32_t topic_key = name_key(topic);
    uint32_t n = 0u;
    for (uint32_t i = 0; i < RTPS_MAX_LOCAL_ENDPOINTS; ++i) {
        if (locals[i].valid() && strcmp(locals[i].topicName.str, topic) == 0) ++n;
    }
    for (uint32_t i = 0; i < n_remote; ++i) {
        if (remotes[i].valid() && remotes[i].topicKey == topic_key) ++n;
    }
    return n;
}

}  // namespace rtps
