/**
 * @file rtps/endpoint_match_table.cpp
 * @brief EndpointMatchTable: local and remote endpoint records and matching
 *        (§8.5.5.1, OMG formal/2022-04-01).
 */

#include "endpoint_match_table.hpp"

#include "log.hpp"

namespace rtps {

const LocalSubnets EndpointMatchTable::kNoSubnets{};

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

EndpointMatchTable::RemoteEndpoint* EndpointMatchTable::find_remote(RemoteEndpoint* table,
                                                                    uint32_t n,
                                                                    const Guid& g) noexcept
{
    RemoteEndpoint* free_slot = nullptr;
    for (uint32_t i = 0; i < n; ++i) {
        if (!table[i].valid()) {
            if (!free_slot) free_slot = &table[i];
            continue;
        }
        if (table[i].entityId == g.entityId && origins_[table[i].origin].prefix == g.prefix) {
            return &table[i];
        }
    }
    return g == GUID_UNKNOWN ? free_slot : nullptr;
}

bool EndpointMatchTable::origin_holds(const RemoteOrigin& o, const Locator* locators,
                                      uint8_t locator_count) const noexcept {
    if (o.list == kNoList) {
        return locator_count == 0u;
    }

    const RemoteLocatorList& l = lists_[o.list];
    if (l.count != locator_count) {
        return false;
    }

    for (uint8_t k = 0; k < locator_count; ++k) {
        if (l.locators[k] != locators[k]) {
            return false;
        }
    }

    return true;
}

uint8_t EndpointMatchTable::free_list() const noexcept {
    for (uint8_t i = 0; i < RTPS_MAX_REMOTE_LOCATOR_LISTS; ++i) {
        if (lists_[i].count == 0u) {
            return i;
        }
    }

    return kNoList;
}

void EndpointMatchTable::release_origin(uint8_t origin) noexcept {
    RemoteOrigin& o = origins_[origin];
    --o.users;
    if ((o.users == 0u) && (o.list != kNoList)) {
        lists_[o.list] = RemoteLocatorList{};
        o.list = kNoList;
    }
}

void EndpointMatchTable::drop_participant_defaults(const GuidPrefix& prefix, const Locator* kept,
                                                   uint8_t& count) const noexcept {
    const Locator* defaults = nullptr;
    uint8_t default_count = 0u;
    if ((count == 0u) || (participant_defaults_ == nullptr)
        || !participant_defaults_(prefix, defaults, default_count, user_data_)) {
        return;
    }

    // A proxy sends to one locator, the preferred entry of its list, so an
    // endpoint whose preferred entry is its participant's is reached the
    // same way through the defaults. Comparing that entry, not the whole
    // list, also holds for a peer reactivated from its record, whose active
    // slot keeps only that default until its next SPDP announcement
    // restores the rest in place.
    const Locator* own = preferred_unicast_locator(kept, count);
    const Locator* shared = preferred_unicast_locator(defaults, default_count);
    if ((shared != nullptr) && (*own == *shared)) {
        count = 0u;
    }
}

uint8_t EndpointMatchTable::find_origin(const GuidPrefix& prefix, const Locator* locators,
                                        uint8_t locator_count) const noexcept {
    uint8_t free_origin = kNoOrigin;
    for (uint8_t i = 0; i < RTPS_MAX_REMOTE_ORIGINS; ++i) {
        const RemoteOrigin& o = origins_[i];
        if (o.users == 0u) {
            if (free_origin == kNoOrigin) {
                free_origin = i;
            }

            continue;
        }

        if ((o.prefix == prefix) && origin_holds(o, locators, locator_count)) {
            return i;
        }
    }

    return free_origin;
}

void EndpointMatchTable::store_remote(RemoteEndpoint& slot, const Guid& guid, uint8_t origin,
                                      const Locator* locators, uint8_t locator_count,
                                      uint32_t topic_key, uint32_t type_key,
                                      ReliabilityKind reliability, DurabilityKind durability,
                                      bool expects_inline_qos) noexcept {
    // An endpoint moving to another origin releases the one it named first,
    // so a locator list it was the last user of is free for the new origin.
    const uint8_t previous = slot.origin;
    if (previous != origin) {
        if (previous != kNoOrigin) {
            release_origin(previous);
        }

        RemoteOrigin& o = origins_[origin];
        if (o.users == 0u) {
            o.prefix = guid.prefix;
            o.list = locator_count > 0u ? free_list() : kNoList;
            if (o.list != kNoList) {
                RemoteLocatorList& l = lists_[o.list];
                l.count = locator_count;
                for (uint8_t k = 0; k < locator_count; ++k) {
                    l.locators[k] = locators[k];
                }
            }
        }

        ++o.users;
    }

    slot.entityId = guid.entityId;
    slot.topicKey = topic_key;
    slot.typeKey = type_key;
    slot.origin = origin;
    slot.reliability = static_cast<uint8_t>(reliability);
    slot.durability = static_cast<uint8_t>(durability);
    slot.expectsInlineQos = expects_inline_qos;
}

void EndpointMatchTable::clear_remote(RemoteEndpoint& r) noexcept
{
    if (r.valid()) {
        release_origin(r.origin);
    }

    r = RemoteEndpoint{};
}

// ------------------------------------------------------------
// Capacity: which remote records are kept when a table is full
// ------------------------------------------------------------

bool EndpointMatchTable::pairs_with_local(const TopicName& topic, const TopicName& type,
                                          ReliabilityKind reliability, DurabilityKind durability,
                                          bool remote_is_writer) const noexcept
{
    const LocalEndpoint* locals = remote_is_writer ? local_readers_ : local_writers_;
    for (uint32_t i = 0; i < RTPS_MAX_LOCAL_ENDPOINTS; ++i) {
        const LocalEndpoint& l = locals[i];
        if (!l.valid() || l.topicName != topic || l.typeName != type) continue;
        const bool compatible =
            remote_is_writer ? qos_compatible(reliability, durability, l.reliability, l.durability)
                             : qos_compatible(l.reliability, l.durability, reliability, durability);
        if (compatible) return true;
    }
    return false;
}

bool EndpointMatchTable::in_match(const Guid& guid, bool is_writer) const noexcept
{
    for (const MatchRecord& m : matches_) {
        if (m.active() && (is_writer ? m.writer_guid : m.reader_guid) == guid) return true;
    }
    return false;
}

EndpointMatchTable::RemoteEndpoint* EndpointMatchTable::displace_unmatched(RemoteEndpoint* table,
                                                                           uint32_t n,
                                                                           bool is_writer) noexcept
{
    for (uint32_t i = 0; i < n; ++i) {
        if (!table[i].valid() || in_match(guid_of(table[i]), is_writer)) continue;
        // The caller stores the new record here and reports the change, which
        // also covers the record forgotten.
        clear_remote(table[i]);
        return &table[i];
    }
    return nullptr;
}

uint8_t EndpointMatchTable::evict_unmatched_origin(bool with_list) noexcept {
    bool matched[RTPS_MAX_REMOTE_ORIGINS] = {};
    for (const RemoteEndpoint& w : remote_writers_) {
        if (w.valid() && in_match(guid_of(w), /*is_writer=*/true)) {
            matched[w.origin] = true;
        }
    }

    for (const RemoteEndpoint& r : remote_readers_) {
        if (r.valid() && in_match(guid_of(r), /*is_writer=*/false)) {
            matched[r.origin] = true;
        }
    }

    for (uint8_t i = 0; i < RTPS_MAX_REMOTE_ORIGINS; ++i) {
        const RemoteOrigin& o = origins_[i];
        if ((o.users == 0u) || matched[i] || (with_list && (o.list == kNoList))) {
            continue;
        }

        // As in displace_unmatched(), the caller's store reports the change.
        for (RemoteEndpoint& w : remote_writers_) {
            if (w.origin == i) {
                clear_remote(w);
            }
        }

        for (RemoteEndpoint& r : remote_readers_) {
            if (r.origin == i) {
                clear_remote(r);
            }
        }

        return i;
    }

    return kNoOrigin;
}

EndpointMatchTable::RemoteEndpoint* EndpointMatchTable::admit_remote(
    RemoteEndpoint* table, uint32_t n, bool is_writer, const Guid& guid, const Locator* locators,
    uint8_t locator_count, bool completes_match, uint8_t& origin) noexcept {
    RemoteEndpoint* slot = find_remote(table, n, guid);
    if (slot == nullptr) {
        slot = find_remote(table, n, GUID_UNKNOWN);
    }

    // A free origin is usable only with a free list when the endpoint keeps
    // locators; the endpoint's own list counts as free when it is the last
    // user of the origin it leaves (store_remote() releases that one first).
    origin = find_origin(guid.prefix, locators, locator_count);
    const bool frees_own_list = (slot != nullptr) && slot->valid()
                                && (origins_[slot->origin].users == 1u)
                                && (origins_[slot->origin].list != kNoList);
    const bool list_missing = (origin != kNoOrigin) && (origins_[origin].users == 0u)
                              && (locator_count > 0u) && (free_list() == kNoList)
                              && !frees_own_list;
    if (!completes_match) {
        return (origin == kNoOrigin) || list_missing ? nullptr : slot;
    }

    // Evicting an origin can free records in this table (and may clear the
    // endpoint's own earlier record, whose slot then stays usable), so the
    // origin is found first and the record second. Evicting one with a list
    // frees an origin and a list together.
    if ((origin == kNoOrigin) || list_missing) {
        origin = evict_unmatched_origin(/*with_list=*/locator_count > 0u);
    }

    if (origin == kNoOrigin) {
        return nullptr;
    }

    if (slot == nullptr) {
        slot = find_remote(table, n, GUID_UNKNOWN);
    }

    if (slot == nullptr) {
        slot = displace_unmatched(table, n, is_writer);
    }

    // Displacing a record can release the origin just found when that record
    // was its only user; store_remote() then claims it again, with the list
    // that release freed when it needs one.
    return slot;
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
            report(writer, matched(r), w->topicName);
        }
        return;
    }

    Locator kept[RTPS_MAX_REMOTE_ENDPOINT_LOCATORS];
    uint8_t kept_count = subnets_->select(kept, RTPS_MAX_REMOTE_ENDPOINT_LOCATORS,
                                          wd.unicastLocatorList, wd.unicastLocatorCount);
    drop_participant_defaults(wd.writerGuid.prefix, kept, kept_count);
    unmatch_if_relocated(remote_writers_, RTPS_MAX_DISCOVERED_WRITERS, /*is_writer=*/true,
                         wd.writerGuid, kept, kept_count);
    const bool completes_match = pairs_with_local(wd.topicName, wd.typeName, wd.reliabilityKind,
                                                  wd.durabilityKind, /*remote_is_writer=*/true);
    uint8_t origin = kNoOrigin;
    RemoteEndpoint* w = admit_remote(remote_writers_, RTPS_MAX_DISCOVERED_WRITERS,
                                     /*is_writer=*/true, wd.writerGuid, kept, kept_count,
                                     completes_match, origin);
    if (!w) {
        if (completes_match) {
            RTPS_LOG_W(
                "match",
                "no room for remote writer, topic=%s not matched: every "
                "stored writer (RTPS_MAX_DISCOVERED_WRITERS=%d) or origin "
                "(RTPS_MAX_REMOTE_ORIGINS=%d, RTPS_MAX_REMOTE_LOCATOR_LISTS=%d) is in a match",
                wd.topicName.str, RTPS_MAX_DISCOVERED_WRITERS, RTPS_MAX_REMOTE_ORIGINS,
                RTPS_MAX_REMOTE_LOCATOR_LISTS);
        } else {
            RTPS_LOG_D("match", "remote writer table full, topic=%s not kept", wd.topicName.str);
        }
        return;
    }
    const bool is_change = !w->valid() || w->topicKey != topic_key || w->typeKey != type_key;
    store_remote(*w, wd.writerGuid, origin, kept, kept_count, topic_key, type_key,
                 wd.reliabilityKind, wd.durabilityKind, false);
    if (is_change) changed();
    for (const LocalEndpoint& r : local_readers_) {
        if (!r.valid() || r.topicName != wd.topicName || r.typeName != wd.typeName) continue;
        if (!qos_compatible(wd.reliabilityKind, wd.durabilityKind,
                            r.reliability, r.durability)) continue;
        report(matched(*w), MatchedEndpoint{r.guid, nullptr, 0u, r.expectsInlineQos},
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
            report(matched(w), reader, r->topicName);
        }
        return;
    }

    Locator kept[RTPS_MAX_REMOTE_ENDPOINT_LOCATORS];
    uint8_t kept_count = subnets_->select(kept, RTPS_MAX_REMOTE_ENDPOINT_LOCATORS,
                                          rd.unicastLocatorList, rd.unicastLocatorCount);
    drop_participant_defaults(rd.readerGuid.prefix, kept, kept_count);
    unmatch_if_relocated(remote_readers_, RTPS_MAX_DISCOVERED_READERS, /*is_writer=*/false,
                         rd.readerGuid, kept, kept_count);
    const bool completes_match = pairs_with_local(rd.topicName, rd.typeName, rd.reliabilityKind,
                                                  rd.durabilityKind, /*remote_is_writer=*/false);
    uint8_t origin = kNoOrigin;
    RemoteEndpoint* r = admit_remote(remote_readers_, RTPS_MAX_DISCOVERED_READERS,
                                     /*is_writer=*/false, rd.readerGuid, kept, kept_count,
                                     completes_match, origin);
    if (!r) {
        if (completes_match) {
            RTPS_LOG_W(
                "match",
                "no room for remote reader, topic=%s not matched: every "
                "stored reader (RTPS_MAX_DISCOVERED_READERS=%d) or origin "
                "(RTPS_MAX_REMOTE_ORIGINS=%d, RTPS_MAX_REMOTE_LOCATOR_LISTS=%d) is in a match",
                rd.topicName.str, RTPS_MAX_DISCOVERED_READERS, RTPS_MAX_REMOTE_ORIGINS,
                RTPS_MAX_REMOTE_LOCATOR_LISTS);
        } else {
            RTPS_LOG_D("match", "remote reader table full, topic=%s not kept", rd.topicName.str);
        }
        return;
    }
    const bool is_change = !r->valid() || r->topicKey != topic_key || r->typeKey != type_key;
    store_remote(*r, rd.readerGuid, origin, kept, kept_count, topic_key, type_key,
                 rd.reliabilityKind, rd.durabilityKind, rd.expectsInlineQos);
    if (is_change) changed();
    for (const LocalEndpoint& w : local_writers_) {
        if (!w.valid() || w.topicName != rd.topicName || w.typeName != rd.typeName) continue;
        if (!qos_compatible(w.reliability, w.durability,
                            rd.reliabilityKind, rd.durabilityKind)) continue;
        report(MatchedEndpoint{w.guid, nullptr, 0u, false}, matched(*r), w.topicName);
    }
}

void EndpointMatchTable::report(const MatchedEndpoint& writer, const MatchedEndpoint& reader,
                                const TopicName& topic) noexcept
{
    MatchRecord* free_slot = nullptr;
    uint32_t waiting = 0u;
    for (MatchRecord& m : matches_) {
        if (m.writer_guid == writer.guid && m.reader_guid == reader.guid) {
            return;
        }

        if (!free_slot && !m.active()) {
            free_slot = &m;
        }

        if (m.active() && m.waiting) {
            ++waiting;
        }
    }

    // Without a free record the pair is not reported, so every reported
    // match has a matching unmatch.
    if (!free_slot) {
        RTPS_LOG_W("match", "match record table full (RTPS_MAX_MATCH_RECORDS=%lu)",
                   (unsigned long)kMaxMatches);
        return;
    }

    // A proxy freed since the last rebind belongs to the oldest waiting pair
    // that can take it, not to a pair reported after it. MayMatch tells
    // before anything binds whether the pair would wait, so a pair turned
    // away at the waiting limit holds no proxy and needs no unmatch.
    const bool may_bind =
        oldest_bindable() == nullptr
        && (may_match_ == nullptr || may_match_(writer.guid, reader.guid, user_data_));
    if (!may_bind && waiting >= RTPS_MAX_WAITING_MATCHES) {
        RTPS_LOG_W("match", "waiting matches full (RTPS_MAX_WAITING_MATCHES=%lu): topic=%s",
                   (unsigned long)RTPS_MAX_WAITING_MATCHES, topic.str);
        return;
    }

    free_slot->writer_guid = writer.guid;
    free_slot->reader_guid = reader.guid;
    free_slot->ticket = next_ticket_++;
    free_slot->waiting = !may_bind || (on_match_ && !on_match_(writer, reader, topic, user_data_));
}

bool EndpointMatchTable::describe(const MatchRecord& m, MatchedEndpoint& writer,
                                  MatchedEndpoint& reader, const TopicName*& topic) noexcept {
    topic = nullptr;
    if (is_local(m.writer_guid)) {
        const LocalEndpoint* w = find(local_writers_, RTPS_MAX_LOCAL_ENDPOINTS, m.writer_guid);
        if (w == nullptr) {
            return false;
        }

        writer = MatchedEndpoint{w->guid, nullptr, 0u, false};
        topic = &w->topicName;
    } else {
        const RemoteEndpoint* w =
            find_remote(remote_writers_, RTPS_MAX_DISCOVERED_WRITERS, m.writer_guid);
        if (w == nullptr || !w->valid()) {
            return false;
        }

        writer = matched(*w);
    }

    if (is_local(m.reader_guid)) {
        const LocalEndpoint* r = find(local_readers_, RTPS_MAX_LOCAL_ENDPOINTS, m.reader_guid);
        if (r == nullptr) {
            return false;
        }

        reader = MatchedEndpoint{r->guid, nullptr, 0u, r->expectsInlineQos};
        topic = &r->topicName;
    } else {
        const RemoteEndpoint* r =
            find_remote(remote_readers_, RTPS_MAX_DISCOVERED_READERS, m.reader_guid);
        if (r == nullptr || !r->valid()) {
            return false;
        }

        reader = matched(*r);
    }

    return topic != nullptr;
}

EndpointMatchTable::MatchRecord* EndpointMatchTable::oldest_bindable() noexcept {
    if (may_match_ == nullptr) {
        return nullptr;
    }

    MatchRecord* oldest = nullptr;
    for (MatchRecord& m : matches_) {
        if (!m.active() || !m.waiting) {
            continue;
        }

        // Tickets wrap; a ticket is older when it lies less than half the
        // counter's range behind the other.
        const bool older = oldest == nullptr || static_cast<int16_t>(m.ticket - oldest->ticket) < 0;
        if (older && may_match_(m.writer_guid, m.reader_guid, user_data_)) {
            oldest = &m;
        }
    }

    return oldest;
}

bool EndpointMatchTable::match_oldest_waiting() noexcept {
    MatchRecord* oldest = oldest_bindable();
    MatchedEndpoint writer{};
    MatchedEndpoint reader{};
    const TopicName* topic = nullptr;
    if (oldest == nullptr || on_match_ == nullptr || !describe(*oldest, writer, reader, topic)) {
        return false;
    }

    oldest->waiting = !on_match_(writer, reader, *topic, user_data_);
    return !oldest->waiting;
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

void EndpointMatchTable::unmatch_if_relocated(RemoteEndpoint* table, uint32_t n, bool is_writer,
                                              const Guid& guid, const Locator* locators,
                                              uint8_t count) noexcept {
    const RemoteEndpoint* stored = find_remote(table, n, guid);
    if ((stored == nullptr) || !stored->valid()) {
        return;
    }

    if (origin_holds(origins_[stored->origin], locators, count)) {
        return;
    }

    RTPS_LOG_I("match", "remote %s re-announced with new locators, matching it again",
               is_writer ? "writer" : "reader");
    dissolve(guid, is_writer);
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
    if (RemoteEndpoint* w = find_remote(remote_writers_, RTPS_MAX_DISCOVERED_WRITERS, writer_guid)) {
        clear_remote(*w);
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
    if (RemoteEndpoint* r = find_remote(remote_readers_, RTPS_MAX_DISCOVERED_READERS, reader_guid)) {
        clear_remote(*r);
        forgotten = true;
    }
    if (forgotten) changed();
}

bool EndpointMatchTable::has_match_with(const GuidPrefix& prefix,
                                        const Guid& except_local) const noexcept {
    for (const MatchRecord& m : matches_) {
        if (!m.active()) {
            continue;
        }

        const bool counts = m.writer_guid != except_local && m.reader_guid != except_local;
        if (counts && (m.writer_guid.prefix == prefix || m.reader_guid.prefix == prefix)) {
            return true;
        }
    }

    return false;
}

void EndpointMatchTable::remove_participant(const GuidPrefix& gone_prefix) noexcept
{
    // Copy each GUID: removal clears the record it lives in.
    for (const RemoteEndpoint& w : remote_writers_) {
        if (!w.valid() || origins_[w.origin].prefix != gone_prefix) continue;
        remove_writer(guid_of(w));
    }
    for (const RemoteEndpoint& r : remote_readers_) {
        if (!r.valid() || origins_[r.origin].prefix != gone_prefix) continue;
        remove_reader(guid_of(r));
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

[[gnu::cold]] bool EndpointMatchTable::local_writer_data(const Guid& guid,
                                                         DiscoveredWriterData& out) const noexcept {
    for (const LocalEndpoint& w : local_writers_) {
        if (!w.valid() || w.guid != guid) {
            continue;
        }

        out.writerGuid = w.guid;
        out.topicName = w.topicName;
        out.typeName = w.typeName;
        out.reliabilityKind = w.reliability;
        out.durabilityKind = w.durability;
        return true;
    }

    return false;
}

[[gnu::cold]] bool EndpointMatchTable::local_reader_data(const Guid& guid,
                                                         DiscoveredReaderData& out) const noexcept {
    for (const LocalEndpoint& r : local_readers_) {
        if (!r.valid() || r.guid != guid) {
            continue;
        }

        out.readerGuid = r.guid;
        out.topicName = r.topicName;
        out.typeName = r.typeName;
        out.reliabilityKind = r.reliability;
        out.durabilityKind = r.durability;
        out.expectsInlineQos = r.expectsInlineQos;
        return true;
    }

    return false;
}

}  // namespace rtps
