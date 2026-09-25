/**
 * @file rtps/endpoint_match_table.hpp
 * @brief Endpoint discovery data and matching — §8.5.4–§8.5.5
 *        (OMG formal/2022-04-01).
 *
 * DiscoveredWriterData and DiscoveredReaderData (§8.5.4.2) carry an
 * endpoint's attributes as SEDP announces them, or as the participant
 * describes one of its own endpoints. EndpointMatchTable keeps what matching
 * needs and reports every compatible (writer, reader) pair that involves a
 * local endpoint.
 *
 * Storage
 * -------
 * Only pairs with a local endpoint are ever wired, so the table keeps two
 * kinds of record:
 *
 *   Local endpoints   full topic and type names, QoS. At most
 *                     RTPS_MAX_LOCAL_ENDPOINTS writers and as many readers.
 *   Remote endpoints  GUID, QoS, the locators the endpoint announced (none
 *                     means its participant's defaults), and a 64-bit hash
 *                     of topic and type (topic_type_key()). No names.
 *
 * A remote endpoint is compared by name, exactly, against the local
 * endpoints when it is announced, while its full names are at hand. A local
 * endpoint created later is compared against stored remote endpoints by
 * hash; with a 64-bit FNV-1a key a false match needs a collision between
 * two distinct topic/type pairs (about 2^-64 per pair).
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
 * on_match(writer, reader, topic): a compatible pair was found; the owner
 *   (RTPSParticipant) adds the matching proxies.
 * on_unmatch(writer_guid, reader_guid): a reported pair was dissolved.
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

/**
 * @brief True when an offered (writer) QoS satisfies a requested (reader) QoS.
 *
 * @param[in] writer_reliability Writer's reliability kind.
 * @param[in] writer_durability  Writer's durability kind.
 * @param[in] reader_reliability Reader's reliability kind.
 * @param[in] reader_durability  Reader's durability kind.
 * @return `false` when a RELIABLE reader meets a BEST_EFFORT writer, or the
 *   writer's durability is below the reader's; `true` otherwise.
 *
 * Verification:
 * - Correct means: the request/offer rules of DDS 1.4 §2.2.3 hold.
 * - Covered by: TC-SEDP-8.5.5.1-002, TC-SEDP-8.5.5.1-003.
 *
 * References:
 * - OMG DDSI-RTPS 2.5 §8.5.5.1; OMG DDS 1.4 §2.2.3.4 (DURABILITY),
 *   §2.2.3.14 (RELIABILITY)
 */
inline bool qos_compatible(ReliabilityKind writer_reliability,
                           DurabilityKind  writer_durability,
                           ReliabilityKind reader_reliability,
                           DurabilityKind  reader_durability) noexcept
{
    if (reader_reliability == ReliabilityKind::RELIABLE &&
        writer_reliability != ReliabilityKind::RELIABLE) {
        return false;
    }
    return static_cast<uint32_t>(writer_durability) >=
           static_cast<uint32_t>(reader_durability);
}

/// @brief qos_compatible() for two discovered endpoints.
inline bool qos_compatible(const DiscoveredWriterData& writer,
                           const DiscoveredReaderData& reader) noexcept
{
    return qos_compatible(writer.reliabilityKind, writer.durabilityKind,
                          reader.reliabilityKind, reader.durabilityKind);
}

/**
 * @brief 64-bit FNV-1a hash of a topic name and a type name.
 *
 * The key a remote endpoint is remembered by (see the file comment). The
 * two names are hashed with a zero octet between them, so ("ab", "c") and
 * ("a", "bc") differ.
 *
 * @param[in] topic Null-terminated topic name.
 * @param[in] type  Null-terminated type name.
 * @return The key; equal names always give equal keys.
 *
 * References:
 * - Fowler, Noll, Vo: FNV-1a, 64-bit parameters (IETF draft-eastlake-fnv)
 */
uint64_t topic_type_key(const char* topic, const char* type) noexcept;

#ifndef RTPS_MAX_DISCOVERED_WRITERS
#  define RTPS_MAX_DISCOVERED_WRITERS 8
#endif
#ifndef RTPS_MAX_DISCOVERED_READERS
#  define RTPS_MAX_DISCOVERED_READERS 8
#endif
#ifndef RTPS_MAX_LOCAL_ENDPOINTS
#  define RTPS_MAX_LOCAL_ENDPOINTS 8
#endif
/// Locators kept per remote endpoint, as a WriterProxy or ReaderProxy keeps.
#ifndef RTPS_MAX_REMOTE_ENDPOINT_LOCATORS
#  define RTPS_MAX_REMOTE_ENDPOINT_LOCATORS 2
#endif
// Writer-reader pairs tracked at once. Only pairs that involve a local
// endpoint are tracked, so this scales with local endpoints times their
// matched peers rather than with the whole graph.
#ifndef RTPS_MAX_MATCH_RECORDS
#  define RTPS_MAX_MATCH_RECORDS (RTPS_MAX_DISCOVERED_WRITERS * RTPS_MAX_DISCOVERED_READERS)
#endif

/// One side of a matched pair, as on_match reports it.
struct MatchedEndpoint {
    Guid           guid             = GUID_UNKNOWN;
    /// Announced unicast locators; none (count 0) means the endpoint is
    /// reached at its participant's default unicast locators (§8.5.3.2).
    const Locator* locators         = nullptr;
    uint8_t        locatorCount     = 0;
    bool           expectsInlineQos = false;   ///< Readers only.
};

/**
 * @brief Endpoints known to a participant, and the pairs among them that match.
 *
 * See the file comment for what is stored and how matching works. The
 * table is filled by SEDP (remote endpoints) and by the participant (its
 * own endpoints); it tells local from remote by the participant's GUID
 * prefix (set_local_prefix()), and matches nothing until that is set.
 */
class EndpointMatchTable {
public:

    /// A writer-reader pair became compatible. `topic` is the topic name of
    /// the pair's local endpoint.
    using OnMatch   = void (*)(const MatchedEndpoint& writer,
                               const MatchedEndpoint& reader,
                               const TopicName&       topic,
                               void*                  user_data) noexcept;

    /// A previously reported pair was dissolved.
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

    /// @brief Sets the GUID prefix that marks an endpoint as local.
    void set_local_prefix(const GuidPrefix& prefix) noexcept {
        local_prefix_ = prefix;
        has_local_prefix_ = true;
    }

    /**
     * @brief Records a writer and reports every local pair it now completes.
     *
     * A local writer is compared with every local reader by name and with
     * every remote reader by topic_type_key(); a remote writer is compared
     * with every local reader by name. Adding a known GUID updates its
     * record and reports only pairs not reported before.
     *
     * @param[in] wd The writer; for a remote writer, as SEDP announced it.
     *
     * Verification:
     * - Correct means: every compatible pair with a local endpoint is
     *   reported exactly once until it is dissolved; remote-remote pairs
     *   never are.
     * - Covered by: TC-SEDP-8.5.5.1-001..005, TC-SEDP-8.5.4-002.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.5.5.1
     */
    void add_writer(const DiscoveredWriterData& wd) noexcept;

    /// @brief As add_writer(), for a reader.
    void add_reader(const DiscoveredReaderData& rd) noexcept;

    /// @brief Forgets a writer and reports every pair it was part of as dissolved.
    void remove_writer(const Guid& writer_guid) noexcept;

    /// @brief Forgets a reader and reports every pair it was part of as dissolved.
    void remove_reader(const Guid& reader_guid) noexcept;

    /**
     * @brief Forgets every endpoint of a departed participant (§8.5.3.2).
     *
     * @param[in] gone_prefix GUID prefix of the participant whose lease
     *   expired or that disposed itself.
     */
    void remove_participant(const GuidPrefix& gone_prefix) noexcept;

private:

    struct LocalEndpoint {
        Guid            guid        = GUID_UNKNOWN;
        TopicName       topicName   = {};
        TopicName       typeName    = {};
        uint64_t        key         = 0;
        ReliabilityKind reliability = ReliabilityKind::BEST_EFFORT;
        DurabilityKind  durability  = DurabilityKind::VOLATILE;
        bool            expectsInlineQos = false;
        bool valid() const noexcept { return guid != GUID_UNKNOWN; }
    };

    struct RemoteEndpoint {
        Guid     guid             = GUID_UNKNOWN;
        uint64_t key              = 0;
        uint8_t  reliability      = 0;   ///< ReliabilityKind.
        uint8_t  durability       = 0;   ///< DurabilityKind.
        bool     expectsInlineQos = false;
        uint8_t  locatorCount     = 0;
        Locator  locators[RTPS_MAX_REMOTE_ENDPOINT_LOCATORS] = {};
        bool valid() const noexcept { return guid != GUID_UNKNOWN; }
        ReliabilityKind reliability_kind() const noexcept {
            return static_cast<ReliabilityKind>(reliability);
        }
        DurabilityKind durability_kind() const noexcept {
            return static_cast<DurabilityKind>(durability);
        }
        MatchedEndpoint matched() const noexcept {
            return MatchedEndpoint{guid, locators, locatorCount, expectsInlineQos};
        }
    };

    struct MatchRecord {
        Guid writer_guid = GUID_UNKNOWN;
        Guid reader_guid = GUID_UNKNOWN;
        bool active() const noexcept { return writer_guid != GUID_UNKNOWN; }
    };

    static constexpr uint32_t kMaxMatches = RTPS_MAX_MATCH_RECORDS;

    bool is_local(const Guid& g) const noexcept {
        return has_local_prefix_ && g.prefix == local_prefix_;
    }

    static LocalEndpoint* find(LocalEndpoint* table, uint32_t n, const Guid& g) noexcept;
    static RemoteEndpoint* find(RemoteEndpoint* table, uint32_t n, const Guid& g) noexcept;
    static void store_remote(RemoteEndpoint& slot, const Guid& guid, uint64_t key,
                             ReliabilityKind reliability, DurabilityKind durability,
                             bool expects_inline_qos, const Locator* locators,
                             uint8_t locator_count) noexcept;

    /// Reports the pair unless it was reported before or no record is free.
    void report(const MatchedEndpoint& writer, const MatchedEndpoint& reader,
                const TopicName& topic) noexcept;
    /// Reports every recorded pair involving `guid` as dissolved and forgets it.
    void dissolve(const Guid& guid, bool is_writer) noexcept;

    LocalEndpoint  local_writers_[RTPS_MAX_LOCAL_ENDPOINTS]     = {};
    LocalEndpoint  local_readers_[RTPS_MAX_LOCAL_ENDPOINTS]     = {};
    RemoteEndpoint remote_writers_[RTPS_MAX_DISCOVERED_WRITERS] = {};
    RemoteEndpoint remote_readers_[RTPS_MAX_DISCOVERED_READERS] = {};
    MatchRecord    matches_[kMaxMatches]                        = {};

    OnMatch    on_match_;
    OnUnmatch  on_unmatch_;
    void*      user_data_;
    GuidPrefix local_prefix_ = {};
    bool       has_local_prefix_ = false;
};

} // namespace rtps
