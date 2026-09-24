/**
 * @file rtps/endpoint_match_table.hpp
 * @brief Endpoint discovery data types and matching — §8.5.4–§8.5.5
 *        (OMG formal/2022-04-01).
 *
 * Defines the discovered endpoint data structures and the EndpointMatchTable
 * which implements the QoS-compatibility-based matching rules from §8.5.5.1.
 *
 * Discovered endpoint data (§8.5.4)
 * -----------------------------------
 * DiscoveredWriterData (§8.5.4.2, Table 8.78, p. 166) and
 * DiscoveredReaderData (§8.5.4.2, Table 8.79, p. 166) carry the endpoint
 * attributes announced via SEDP.  Only the fields needed for matching and
 * proxy wiring on this embedded target are stored.
 *
 * Matching rules (§8.5.5.1, p. 169)
 * ------------------------------------
 * Two endpoints match when ALL of the following hold:
 *
 *   1. topicName must be equal (case-sensitive). — §8.5.5.1, p. 169.
 *   2. typeName must be equal (case-sensitive). — §8.5.5.1, p. 169.
 *   3. Durability QoS compatibility (§8.5.5.1, p. 169; §7.6.3.4):
 *        writer.durabilityKind >= reader.durabilityKind
 *        (VOLATILE=0 < TRANSIENT_LOCAL=1 < TRANSIENT=2 < PERSISTENT=3)
 *   4. Reliability QoS compatibility (§8.5.5.1, p. 169; §7.6.3.3):
 *        if reader.reliability == RELIABLE
 *            then writer.reliability must also be RELIABLE.
 *        A BEST_EFFORT writer is incompatible with a RELIABLE reader.
 *
 * Callbacks
 * ---------
 * on_match(writer_guid, reader_guid):
 *   Fired when a compatible (writer, reader) pair is identified.
 *   The caller uses this to call StatefulWriter::matched_reader_add()
 *   and StatefulReader::matched_writer_add().
 *
 * on_unmatch(writer_guid, reader_guid):
 *   Fired when a previously matched pair is broken (peer removed).
 *
 * Spec references (all OMG formal/2022-04-01)
 * -------------------------------------------
 *   DiscoveredWriterData       — §8.5.4.2, Table 8.78, p. 166
 *   DiscoveredReaderData       — §8.5.4.2, Table 8.79, p. 166
 *   Endpoint matching          — §8.5.5.1, p. 169
 *   Durability compatibility   — §7.6.3.4, p. 84 (QoS spec)
 *   Reliability compatibility  — §7.6.3.3, p. 83 (QoS spec)
 *   SEDP builtin endpoints     — §8.5.4.3, Table 8.80, p. 167
 */

#pragma once

#include "log.hpp"
#include "types.hpp"
#include <string.h>

namespace rtps {

// ============================================================
// Topic / type name storage
//
// No std::string.  Names are stored as fixed-length null-terminated
// char arrays.  The spec places no hard limit on name length;
// 64 characters covers all expected embedded-target topic names.
// ============================================================

#ifndef RTPS_MAX_TOPIC_NAME_LEN
#  define RTPS_MAX_TOPIC_NAME_LEN 64
#endif

struct TopicName {
    char str[RTPS_MAX_TOPIC_NAME_LEN] = {};

    void assign(const char* s) noexcept {
        if (!s) { str[0] = '\0'; return; }
        // strncpy with guaranteed null termination.
        uint32_t i = 0;
        while (i < RTPS_MAX_TOPIC_NAME_LEN - 1u && s[i]) {
            str[i] = s[i]; ++i;
        }
        str[i] = '\0';
    }

    bool operator==(const TopicName& o) const noexcept {
        // Case-sensitive comparison — §8.5.5.1, p. 169.
        return memcmp(str, o.str, RTPS_MAX_TOPIC_NAME_LEN) == 0;
    }
    bool operator!=(const TopicName& o) const noexcept { return !(*this == o); }
    bool empty() const noexcept { return str[0] == '\0'; }
};

// ============================================================
// §8.5.4.2  DiscoveredWriterData — Table 8.78, p. 166
//
// Attributes announced by a remote writer via SEDP.
// writerProxy carries the locators needed to address the writer.
// ============================================================

struct DiscoveredWriterData {
    Guid            writerGuid         = GUID_UNKNOWN;
    TopicName       topicName          = {};
    TopicName       typeName           = {};
    ReliabilityKind reliabilityKind    = ReliabilityKind::BEST_EFFORT;
    DurabilityKind  durabilityKind     = DurabilityKind::VOLATILE;
    TopicKind       topicKind          = TopicKind::NO_KEY;

    // Locators at which the writer can be reached for data.
    // §8.5.4.2, Table 8.78: unicastLocatorList.
    Locator  unicastLocatorList[RTPS_MAX_LOCATORS_PER_PARTICIPANT] = {};
    uint8_t  unicastLocatorCount = 0;

    bool is_valid() const noexcept { return writerGuid != GUID_UNKNOWN; }
};

// ============================================================
// §8.5.4.2  DiscoveredReaderData — Table 8.79, p. 166
// ============================================================

struct DiscoveredReaderData {
    Guid            readerGuid         = GUID_UNKNOWN;
    TopicName       topicName          = {};
    TopicName       typeName           = {};
    ReliabilityKind reliabilityKind    = ReliabilityKind::BEST_EFFORT;
    DurabilityKind  durabilityKind     = DurabilityKind::VOLATILE;
    TopicKind       topicKind          = TopicKind::NO_KEY;
    bool            expectsInlineQos   = false;

    // Locators at which ACKNACK / NACK_FRAG are sent to the reader.
    Locator  unicastLocatorList[RTPS_MAX_LOCATORS_PER_PARTICIPANT] = {};
    uint8_t  unicastLocatorCount = 0;

    bool is_valid() const noexcept { return readerGuid != GUID_UNKNOWN; }
};

// ============================================================
// QoS compatibility check — §8.5.5.1, p. 169
//
// Returns true when the offered (writer) QoS is compatible with
// the requested (reader) QoS.
//
// Rule 1 — Reliability (§7.6.3.3, p. 83):
//   RELIABLE reader requires RELIABLE writer.
//   BEST_EFFORT reader accepts any writer.
//
// Rule 2 — Durability (§7.6.3.4, p. 84):
//   writer.durability >= reader.durability
//   (VOLATILE < TRANSIENT_LOCAL < TRANSIENT < PERSISTENT)
// ============================================================

inline bool qos_compatible(const DiscoveredWriterData& writer,
                            const DiscoveredReaderData& reader) noexcept
{
    // Reliability — §7.6.3.3.
    if (reader.reliabilityKind == ReliabilityKind::RELIABLE &&
        writer.reliabilityKind != ReliabilityKind::RELIABLE) {
        return false;
    }

    // Durability — §7.6.3.4.
    if (static_cast<uint32_t>(writer.durabilityKind) <
        static_cast<uint32_t>(reader.durabilityKind)) {
        return false;
    }

    return true;
}

// ============================================================
// EndpointMatchTable
//
// §8.5.5.1, p. 169 — maintains tables of known writers and readers
// and evaluates compatibility when either side is added.
//
// Matching algorithm:
//   On add_writer(wd):
//     for each known reader rd:
//       if wd.topicName == rd.topicName
//          && wd.typeName == rd.typeName
//          && qos_compatible(wd, rd):
//             fire on_match(wd.writerGuid, rd.readerGuid)
//
//   On add_reader(rd):
//     for each known writer wd:
//       if wd.topicName == rd.topicName  (symmetric)
//          && qos_compatible(wd, rd):
//             fire on_match(wd.writerGuid, rd.readerGuid)
//
//   On remove_writer(guid) / remove_reader(guid):
//     for each currently matched pair involving guid:
//       fire on_unmatch(writer_guid, reader_guid)
//
// Callbacks
// ---------
// on_match and on_unmatch are plain function pointers.  The caller
// (RTPSParticipant) wires StatefulWriter::matched_reader_add() and
// StatefulReader::matched_writer_add() from these callbacks.
// ============================================================

#ifndef RTPS_MAX_DISCOVERED_WRITERS
#  define RTPS_MAX_DISCOVERED_WRITERS 8
#endif
#ifndef RTPS_MAX_DISCOVERED_READERS
#  define RTPS_MAX_DISCOVERED_READERS 8
#endif
// Writer-reader pairs tracked at once. With a local prefix set, only pairs
// that involve a local endpoint are tracked, so this scales with local
// endpoints times their matched peers rather than with the whole graph.
#ifndef RTPS_MAX_MATCH_RECORDS
#  define RTPS_MAX_MATCH_RECORDS (RTPS_MAX_DISCOVERED_WRITERS * RTPS_MAX_DISCOVERED_READERS)
#endif

class EndpointMatchTable {
public:

    /// Called when a writer–reader pair is found to be compatible.
    using OnMatch   = void (*)(const DiscoveredWriterData&,
                               const DiscoveredReaderData&,
                               void* user_data) noexcept;

    /// Called when a previously matched pair is dissolved.
    using OnUnmatch = void (*)(const Guid& writer_guid,
                               const Guid& reader_guid,
                               void* user_data) noexcept;

    EndpointMatchTable(OnMatch   on_match   = nullptr,
                       OnUnmatch on_unmatch = nullptr,
                       void*     user_data  = nullptr) noexcept
        : on_match_(on_match)
        , on_unmatch_(on_unmatch)
        , user_data_(user_data)
    {}

    // ----------------------------------------------------------
    // set_local_prefix()
    // Restrict matching to pairs in which at least one endpoint belongs
    // to the participant with this prefix. Remote-to-remote pairs are
    // still discovered but neither recorded nor reported, since no local
    // proxy exists to wire. RTPSParticipant sets its own prefix.
    // ----------------------------------------------------------
    void set_local_prefix(const GuidPrefix& prefix) noexcept {
        local_prefix_ = prefix;
        has_local_prefix_ = true;
    }

    // ----------------------------------------------------------
    // add_writer()
    //
    // Register a newly discovered remote writer and check for
    // compatible local readers.
    //
    // §8.5.5.1, p. 169:
    //   "When a Writer is discovered, the Participant checks its
    //    Reader endpoints for compatibility."
    //
    // @param wd  Discovered writer data from SEDP.
    // ----------------------------------------------------------
    void add_writer(const DiscoveredWriterData& wd) noexcept {
        if (!wd.is_valid()) return;

        // Insert or update in writer table.
        DiscoveredWriterData* slot = find_writer(wd.writerGuid);
        if (!slot) slot = alloc_writer_slot();
        if (!slot) return;
        *slot = wd;

        // Match against all known readers.
        if (!on_match_) return;
        for (uint32_t i = 0; i < RTPS_MAX_DISCOVERED_READERS; ++i) {
            if (!readers_[i].is_valid()) continue;
            if (!topics_match(wd, readers_[i])) continue;
            if (!qos_compatible(wd, readers_[i])) continue;
            if (!involves_local(wd.writerGuid, readers_[i].readerGuid)) continue;
            if (!record_match(wd.writerGuid, readers_[i].readerGuid)) continue;
            on_match_(wd, readers_[i], user_data_);
        }
    }

    // ----------------------------------------------------------
    // add_reader()
    //
    // Register a newly discovered remote reader and check for
    // compatible local or remote writers.
    //
    // §8.5.5.1, p. 169:
    //   "When a Reader is discovered, the Participant checks its
    //    Writer endpoints for compatibility."
    //
    // @param rd  Discovered reader data from SEDP.
    // ----------------------------------------------------------
    void add_reader(const DiscoveredReaderData& rd) noexcept {
        if (!rd.is_valid()) return;

        DiscoveredReaderData* slot = find_reader(rd.readerGuid);
        if (!slot) slot = alloc_reader_slot();
        if (!slot) return;
        *slot = rd;

        if (!on_match_) return;
        for (uint32_t i = 0; i < RTPS_MAX_DISCOVERED_WRITERS; ++i) {
            if (!writers_[i].is_valid()) continue;
            if (!topics_match(writers_[i], rd)) continue;
            if (!qos_compatible(writers_[i], rd)) continue;
            if (!involves_local(writers_[i].writerGuid, rd.readerGuid)) continue;
            if (!record_match(writers_[i].writerGuid, rd.readerGuid)) continue;
            on_match_(writers_[i], rd, user_data_);
        }
    }

    // ----------------------------------------------------------
    // remove_writer()
    //
    // Evict a writer and fire on_unmatch for all pairs it was part of.
    //
    // §8.5.5.1, p. 169:
    //   "When a Writer is removed, all matched Reader-Writer pairs
    //    involving that Writer are dissolved."
    // ----------------------------------------------------------
    void remove_writer(const Guid& writer_guid) noexcept {
        // Fire on_unmatch for every recorded match involving this writer.
        if (on_unmatch_) {
            for (uint32_t i = 0; i < kMaxMatches; ++i) {
                if (matches_[i].writer_guid == writer_guid &&
                    matches_[i].reader_guid != GUID_UNKNOWN) {
                    on_unmatch_(writer_guid, matches_[i].reader_guid, user_data_);
                    matches_[i] = MatchRecord{};
                }
            }
        }
        // Remove from writer table.
        DiscoveredWriterData* slot = find_writer(writer_guid);
        if (slot) *slot = DiscoveredWriterData{};
    }

    // ----------------------------------------------------------
    // remove_reader()
    // ----------------------------------------------------------
    void remove_reader(const Guid& reader_guid) noexcept {
        if (on_unmatch_) {
            for (uint32_t i = 0; i < kMaxMatches; ++i) {
                if (matches_[i].reader_guid == reader_guid &&
                    matches_[i].writer_guid != GUID_UNKNOWN) {
                    on_unmatch_(matches_[i].writer_guid, reader_guid, user_data_);
                    matches_[i] = MatchRecord{};
                }
            }
        }
        DiscoveredReaderData* slot = find_reader(reader_guid);
        if (slot) *slot = DiscoveredReaderData{};
    }

    // ----------------------------------------------------------
    // remove_participant()
    //
    // Bulk-remove all endpoints (writers and readers) whose GUID
    // prefix matches gone_prefix.  Fires on_unmatch for every
    // affected match record before clearing the endpoint entries.
    //
    // Called from RTPSParticipant::on_spdp_peer_removed() when a
    // remote participant's leaseDuration expires — §8.5.3.2, p. 160:
    //   "If a participant does not receive a refresh within
    //    leaseDuration, the Participant removes all information about
    //    the remote Participant and its endpoints."
    //
    // @param gone_prefix  GuidPrefix of the departed participant.
    // ----------------------------------------------------------
    void remove_participant(const GuidPrefix& gone_prefix) noexcept {
        // Collect all writer GUIDs belonging to this participant.
        // We must fire on_unmatch before clearing the endpoint entries
        // so the callback can still read the match record.
        for (uint32_t i = 0; i < RTPS_MAX_DISCOVERED_WRITERS; ++i) {
            if (!writers_[i].is_valid()) continue;
            if (writers_[i].writerGuid.prefix != gone_prefix) continue;
            remove_writer(writers_[i].writerGuid);
        }
        for (uint32_t i = 0; i < RTPS_MAX_DISCOVERED_READERS; ++i) {
            if (!readers_[i].is_valid()) continue;
            if (readers_[i].readerGuid.prefix != gone_prefix) continue;
            remove_reader(readers_[i].readerGuid);
        }
    }

    // ----------------------------------------------------------
    // Lookup helpers — used by RTPSParticipant to retrieve locators
    // for proxy initialisation.
    // ----------------------------------------------------------
    const DiscoveredWriterData* lookup_writer(const Guid& g) const noexcept {
        return const_cast<EndpointMatchTable*>(this)->find_writer(g);
    }
    const DiscoveredReaderData* lookup_reader(const Guid& g) const noexcept {
        return const_cast<EndpointMatchTable*>(this)->find_reader(g);
    }

private:

    // ----------------------------------------------------------
    // Topic / type name comparison — §8.5.5.1, p. 169.
    // Both must be equal (case-sensitive string comparison).
    // ----------------------------------------------------------
    static bool topics_match(const DiscoveredWriterData& w,
                             const DiscoveredReaderData& r) noexcept {
        return (w.topicName == r.topicName) &&
               (w.typeName  == r.typeName);
    }

    // ----------------------------------------------------------
    // Match recording: avoid firing on_match twice for the same pair.
    // ----------------------------------------------------------
    struct MatchRecord {
        Guid writer_guid = GUID_UNKNOWN;
        Guid reader_guid = GUID_UNKNOWN;
        bool active() const noexcept { return writer_guid != GUID_UNKNOWN; }
    };

    static constexpr uint32_t kMaxMatches = RTPS_MAX_MATCH_RECORDS;

    MatchRecord matches_[kMaxMatches] = {};

    bool already_matched(const Guid& wg, const Guid& rg) const noexcept {
        for (uint32_t i = 0; i < kMaxMatches; ++i) {
            if (matches_[i].writer_guid == wg &&
                matches_[i].reader_guid == rg) return true;
        }
        return false;
    }

    // Returns true when the pair is newly recorded; false when it was
    // already matched or the record table is full (the pair is then not
    // reported, so every reported match has a matching unmatch).
    bool record_match(const Guid& wg, const Guid& rg) noexcept {
        if (already_matched(wg, rg)) return false;
        for (uint32_t i = 0; i < kMaxMatches; ++i) {
            if (!matches_[i].active()) {
                matches_[i].writer_guid = wg;
                matches_[i].reader_guid = rg;
                return true;
            }
        }
        RTPS_LOG_W("match", "match record table full (RTPS_MAX_MATCH_RECORDS=%lu)",
                   (unsigned long)kMaxMatches);
        return false;
    }

    bool involves_local(const Guid& wg, const Guid& rg) const noexcept {
        return !has_local_prefix_ ||
               wg.prefix == local_prefix_ || rg.prefix == local_prefix_;
    }

    // Tables
    DiscoveredWriterData writers_[RTPS_MAX_DISCOVERED_WRITERS] = {};
    DiscoveredReaderData readers_[RTPS_MAX_DISCOVERED_READERS] = {};

    DiscoveredWriterData* find_writer(const Guid& g) noexcept {
        for (uint32_t i = 0; i < RTPS_MAX_DISCOVERED_WRITERS; ++i)
            if (writers_[i].writerGuid == g) return &writers_[i];
        return nullptr;
    }
    DiscoveredWriterData* alloc_writer_slot() noexcept {
        for (uint32_t i = 0; i < RTPS_MAX_DISCOVERED_WRITERS; ++i)
            if (!writers_[i].is_valid()) return &writers_[i];
        return nullptr;
    }
    DiscoveredReaderData* find_reader(const Guid& g) noexcept {
        for (uint32_t i = 0; i < RTPS_MAX_DISCOVERED_READERS; ++i)
            if (readers_[i].readerGuid == g) return &readers_[i];
        return nullptr;
    }
    DiscoveredReaderData* alloc_reader_slot() noexcept {
        for (uint32_t i = 0; i < RTPS_MAX_DISCOVERED_READERS; ++i)
            if (!readers_[i].is_valid()) return &readers_[i];
        return nullptr;
    }

    OnMatch    on_match_;
    OnUnmatch  on_unmatch_;
    void*      user_data_;
    GuidPrefix local_prefix_ = {};
    bool       has_local_prefix_ = false;
};

} // namespace rtps
