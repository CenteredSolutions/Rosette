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
 *                     means its participant's defaults), and a 32-bit key
 *                     of the topic name and one of the type name
 *                     (name_key()). No names.
 *
 * A remote endpoint is compared by name, exactly, against the local
 * endpoints when it is announced, while its full names are at hand. A local
 * endpoint created later is compared against stored remote endpoints by
 * key. A false match needs the topic keys and the type keys of two distinct
 * endpoints to collide together (about 2^-64 per pair). Graph queries that
 * select by topic alone (count()) compare the topic key only, so a false
 * count needs a 32-bit collision (about 2^-32 per pair of topic names); a
 * count can be wrong in that case, a match cannot.
 *
 * Capacity
 * --------
 * A network can announce far more endpoints than a microcontroller can
 * record, and the announcements a participant needs are rarely the first to
 * arrive. While a remote table has room, every announcement is kept, so a
 * local endpoint created later still finds its peers and count() sees the
 * whole graph. When the table is full, an announcement that completes a
 * match with a local endpoint takes the slot of a remote record that is in
 * no match; an announcement that completes none is not kept. A record in a
 * match is never displaced, so matching never loses a pair it reported.
 * The table therefore needs room only for the remote endpoints the local
 * ones match; beyond that, what it keeps of the rest decides only what
 * count() sees and what a later local endpoint can match without a fresh
 * announcement.
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
 * on_change(): the set of known endpoints changed (one was recorded, its
 *   topic or type changed, or it was forgotten); the owner tells graph
 *   observers to re-query.
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
// No std::string: a name is a fixed-size NUL-terminated array. The spec
// sets no length limit, and ROS 2 service topics embed the node name
// (rq/<node>/set_parameters_atomicallyRequest), so 128 octets are kept by
// default. A name that does not fit is never truncated silently: the
// TopicName records that it was cut, the participant refuses to create a
// local endpoint with such a name, and the match table ignores a remote
// endpoint announced with one, so a cut name can never match by its prefix.
// ============================================================

#ifndef RTPS_MAX_TOPIC_NAME_LEN
#  define RTPS_MAX_TOPIC_NAME_LEN 128
#endif

struct TopicName {
    char str[RTPS_MAX_TOPIC_NAME_LEN] = {};
    bool truncated = false;   ///< The assigned name did not fit.

    /**
     * @brief Stores a NUL-terminated name.
     *
     * @param[in] s Name to store; null stores the empty name.
     * @return `true` when the whole name fits; `false` when it was cut to
     *   RTPS_MAX_TOPIC_NAME_LEN - 1 octets (fits() is then false).
     */
    bool assign(const char* s) noexcept {
        return assign(s, s ? static_cast<uint32_t>(strlen(s)) : 0u);
    }

    /**
     * @brief Stores a name of `len` octets that need not be NUL-terminated.
     *
     * @param[in] s   Name octets, as a PL_CDR string carries them.
     * @param[in] len Octets of `s`, without any terminating NUL.
     * @return As assign(const char*).
     */
    bool assign(const char* s, uint32_t len) noexcept {
        const uint32_t n = len < RTPS_MAX_TOPIC_NAME_LEN ? len : RTPS_MAX_TOPIC_NAME_LEN - 1u;
        if (n > 0u) memcpy(str, s, n);
        str[n] = '\0';
        truncated = n < len;
        return !truncated;
    }

    /// @brief True when the stored name is the whole name that was assigned.
    bool fits() const noexcept { return !truncated; }

    bool operator==(const TopicName& o) const noexcept {
        // Case-sensitive comparison — §8.5.5.1, p. 169.
        return strcmp(str, o.str) == 0;
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
 * @brief 32-bit FNV-1a hash of a topic or type name.
 *
 * The key a remote endpoint's topic and type are remembered by (see the
 * file comment). A 32-bit multiply is one instruction on every supported
 * core, including the Cortex-M0+.
 *
 * @param[in] name Null-terminated name; null hashes as the empty name.
 * @return The key; equal names always give equal keys.
 *
 * Verification:
 * - Correct means: the FNV-1a test vectors hold (the empty name gives the
 *   offset basis 0x811C9DC5, "a" gives 0xE40C292C).
 * - Covered by: TC-SEDP-8.5.5.1-007.
 *
 * References:
 * - Fowler, Noll, Vo: FNV-1a, 32-bit parameters (IETF draft-eastlake-fnv)
 */
uint32_t name_key(const char* name) noexcept;

// Remote writers and readers recorded from SEDP. When a table is full, only
// an endpoint that completes a match with a local one is admitted, in place
// of one that completes none (see Capacity in the file comment).
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

    /// The set of known endpoints changed.
    using OnChange  = void (*)(void* user_data) noexcept;

    EndpointMatchTable(OnMatch   on_match   = nullptr,
                       OnUnmatch on_unmatch = nullptr,
                       void*     user_data  = nullptr,
                       OnChange  on_change  = nullptr) noexcept
        : on_match_(on_match)
        , on_unmatch_(on_unmatch)
        , on_change_(on_change)
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
     * every remote reader by topic and type key (name_key()); a remote
     * writer is compared with every local reader by name. Adding a known
     * GUID updates its record and reports only pairs not reported before.
     * A writer whose topic or type name did not fit a TopicName is ignored.
     * When the remote writer table is full, a remote writer that pairs with
     * a local reader displaces a remote writer in no match, and one that
     * pairs with none is not kept (see Capacity in the file comment).
     * on_change fires when the writer is new or its topic or type changed.
     *
     * @param[in] wd The writer; for a remote writer, as SEDP announced it.
     *
     * Verification:
     * - Correct means: every compatible pair with a local endpoint is
     *   reported exactly once until it is dissolved; remote-remote pairs
     *   never are; a full table still admits a remote endpoint that
     *   completes a match unless every stored record is in one.
     * - Covered by: TC-SEDP-8.5.5.1-001..005, TC-SEDP-8.5.5.1-008,
     *   TC-SEDP-8.5.5.1-009, TC-SEDP-8.5.4-002.
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

    /**
     * @brief Counts the known writers or readers on one topic.
     *
     * Answers the graph queries rmw_count_publishers() and
     * rmw_count_subscribers() ask: every endpoint on the topic counts,
     * local or remote, whatever its type or QoS. Local endpoints are
     * compared by name; remote endpoints by topic key, which is exact
     * unless two topic names share a 32-bit key (see the file comment).
     *
     * @param[in] topic   Null-terminated topic name, as SEDP announces it
     *   (for ROS, the mangled name, e.g. "rt/chatter").
     * @param[in] writers `true` to count writers; `false` to count readers.
     * @return Endpoints on `topic` in the table. A remote endpoint the
     *   table had no room for (RTPS_MAX_DISCOVERED_WRITERS/READERS), or
     *   displaced to make room for one that completes a match, is not
     *   counted.
     *
     * Verification:
     * - Correct means: local and remote endpoints on the topic are counted
     *   once each, endpoints on other topics are not, and a forgotten
     *   endpoint no longer counts.
     * - Covered by: TC-SEDP-8.5.4-014.
     *
     * References:
     * - rmw/rmw.h (Humble), `rmw_count_publishers`, `rmw_count_subscribers`
     */
    uint32_t count(const char* topic, bool writers) const noexcept;

private:

    struct LocalEndpoint {
        Guid            guid        = GUID_UNKNOWN;
        TopicName       topicName   = {};
        TopicName       typeName    = {};
        uint32_t        topicKey    = 0;   ///< name_key(topicName).
        uint32_t        typeKey     = 0;   ///< name_key(typeName).
        ReliabilityKind reliability = ReliabilityKind::BEST_EFFORT;
        DurabilityKind  durability  = DurabilityKind::VOLATILE;
        bool            expectsInlineQos = false;
        bool valid() const noexcept { return guid != GUID_UNKNOWN; }
    };

    struct RemoteEndpoint {
        Guid     guid             = GUID_UNKNOWN;
        uint32_t topicKey         = 0;   ///< name_key() of the topic name.
        uint32_t typeKey          = 0;   ///< name_key() of the type name.
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
    static void store_remote(RemoteEndpoint& slot, const Guid& guid, uint32_t topic_key,
                             uint32_t type_key, ReliabilityKind reliability, DurabilityKind durability,
                             bool expects_inline_qos, const Locator* locators,
                             uint8_t locator_count) noexcept;

    /**
     * @brief True when a remote endpoint would complete a match with a local one.
     *
     * Decides whether an announcement may displace a stored remote record
     * when its table is full (see Capacity in the file comment). The test is
     * the one matching applies: equal topic and type names and compatible
     * QoS (§8.5.5.1), against local endpoints of the opposite kind.
     *
     * @param[in] topic            The remote endpoint's topic name.
     * @param[in] type             The remote endpoint's type name.
     * @param[in] reliability      The remote endpoint's reliability kind.
     * @param[in] durability       The remote endpoint's durability kind.
     * @param[in] remote_is_writer `true` for a remote writer (compared with
     *   local readers); `false` for a remote reader (with local writers).
     * @return `true` when some local endpoint pairs with it.
     *
     * Verification:
     * - Correct means: true exactly when adding the endpoint to a table with
     *   room would report at least one pair.
     * - Covered by: TC-SEDP-8.5.5.1-008.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.5.5.1
     */
    bool pairs_with_local(const TopicName& topic, const TopicName& type,
                          ReliabilityKind reliability, DurabilityKind durability,
                          bool remote_is_writer) const noexcept;

    /// @brief True when a reported pair names `guid` as its writer (`is_writer`) or reader.
    bool in_match(const Guid& guid, bool is_writer) const noexcept;

    /**
     * @brief Frees the first remote record that is in no match.
     *
     * The record is cleared without notifying anyone: the caller stores the
     * announcement that needed the slot there, and the on_change that store
     * fires covers the forgotten record too. No pair is dissolved, since a
     * record in no match has none.
     *
     * @param[in,out] table     remote_writers_ or remote_readers_.
     * @param[in]     n         Entries in `table`.
     * @param[in]     is_writer `true` when `table` holds writers.
     * @return The cleared slot, or null when every stored record is in a match.
     *
     * Verification:
     * - Correct means: a record in a match is never cleared.
     * - Covered by: TC-SEDP-8.5.5.1-008, TC-SEDP-8.5.5.1-009.
     */
    RemoteEndpoint* displace_unmatched(RemoteEndpoint* table, uint32_t n, bool is_writer) noexcept;

    /// Reports the pair unless it was reported before or no record is free.
    void report(const MatchedEndpoint& writer, const MatchedEndpoint& reader,
                const TopicName& topic) noexcept;
    /// Reports every recorded pair involving `guid` as dissolved and forgets it.
    void dissolve(const Guid& guid, bool is_writer) noexcept;
    /// @brief Tells the owner the set of known endpoints changed.
    void changed() noexcept { if (on_change_) on_change_(user_data_); }

    LocalEndpoint  local_writers_[RTPS_MAX_LOCAL_ENDPOINTS]     = {};
    LocalEndpoint  local_readers_[RTPS_MAX_LOCAL_ENDPOINTS]     = {};
    RemoteEndpoint remote_writers_[RTPS_MAX_DISCOVERED_WRITERS] = {};
    RemoteEndpoint remote_readers_[RTPS_MAX_DISCOVERED_READERS] = {};
    MatchRecord    matches_[kMaxMatches]                        = {};

    OnMatch    on_match_;
    OnUnmatch  on_unmatch_;
    OnChange   on_change_;
    void*      user_data_;
    GuidPrefix local_prefix_ = {};
    bool       has_local_prefix_ = false;
};

} // namespace rtps
