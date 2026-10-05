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
 *   Remote endpoints  16 octets: EntityId, QoS, a 32-bit key of the topic
 *                     name and one of the type name (name_key()), and the
 *                     index of the endpoint's origin. No names.
 *   Origins           a GUID prefix and the UDP locators an endpoint
 *                     announced (none means its participant's defaults).
 *                     At most RTPS_MAX_REMOTE_ORIGINS.
 *
 * The endpoints of one participant share its GUID prefix and, in every stack
 * this one interoperates with, announce the same locators (Fast DDS and
 * Rosette list the participant's defaults, Cyclone DDS lists none), so they
 * share one origin, and a remote endpoint costs 16 octets however many
 * locators its participant announced. An endpoint that announces locators of
 * its own takes an origin of its own. At most
 * RTPS_MAX_REMOTE_ENDPOINT_LOCATORS UDP locators are kept, the most
 * reachable first: SedpReader ranks an announced list as it decodes it
 * (LocalSubnets::offer()), and add_writer() and add_reader() narrow it
 * (LocalSubnets::select()).
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
 * no match; an announcement that completes none is not kept. Likewise, when
 * it needs an origin and none is free, it frees an origin none of whose
 * endpoints is in a match, forgetting those endpoints. A record in a match
 * is never displaced, so matching never loses a pair it reported.
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

#include <string.h>
#include "locator_ranking.hpp"
#include "types.hpp"

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
/// UDP locators kept per origin, as the SPDP peer record keeps RTPS_MAX_PEER_LOCATORS.
/// Equal limits let an endpoint announcing its participant's defaults compare
/// equal to them, so its proxies follow the peer record (see
/// RTPSParticipant::resolve_endpoint_locators()).
#ifndef RTPS_MAX_REMOTE_ENDPOINT_LOCATORS
#  define RTPS_MAX_REMOTE_ENDPOINT_LOCATORS 2
#endif
/// Distinct (GUID prefix, announced locators) pairs among the stored remote
/// endpoints. One per remote participant covers every stack this one
/// interoperates with (see Storage in the file comment).
#ifndef RTPS_MAX_REMOTE_ORIGINS
#  define RTPS_MAX_REMOTE_ORIGINS RTPS_MAX_PEERS
#endif
static_assert(RTPS_MAX_REMOTE_ORIGINS >= 1 && RTPS_MAX_REMOTE_ORIGINS < 255,
              "RTPS_MAX_REMOTE_ORIGINS must fit a one-octet index below the none value 0xFF");
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
    /// Owned by the table's origin, which stays in place while the pair is
    /// reported: the table dissolves the pair before it releases the origin
    /// (unmatch_if_relocated()), so a proxy may refer to them.
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

    /**
     * @brief Sets the addresses remote endpoint locator lists are ranked against.
     *
     * @param[in] subnets This participant's addresses; must outlive the
     *   table. Until it is set, lists are ranked against no addresses, which
     *   keeps every usable locator in announced order.
     */
    void set_local_subnets(const LocalSubnets& subnets) noexcept { subnets_ = &subnets; }

    /// @brief The addresses remote endpoint locator lists are ranked against.
    const LocalSubnets& local_subnets() const noexcept { return *subnets_; }

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

    /**
     * @brief Fills the discovery data of a local writer the table holds.
     *
     * Lets SEDP rebuild a local writer's announcement from the record kept
     * here for matching instead of storing its own copy (see
     * LocalEndpointSource in sedp.hpp). The table keeps no locators, so
     * `out` carries none; the caller adds the participant's.
     *
     * @param[in]  guid A local writer's GUID.
     * @param[out] out  Its GUID, topic and type names, reliability, and
     *   durability; written only when the writer is found.
     * @return `false` when the table holds no local writer with that GUID.
     *
     * Verification:
     * - Correct means: the fields equal those of the DiscoveredWriterData
     *   add_writer() recorded.
     * - Covered by: TC-SEDP-8.5.4-017.
     */
    bool local_writer_data(const Guid& guid, DiscoveredWriterData& out) const noexcept;

    /// @brief As local_writer_data(), for a local reader; also fills expectsInlineQos.
    bool local_reader_data(const Guid& guid, DiscoveredReaderData& out) const noexcept;

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

    /// Origin index of a free RemoteEndpoint.
    static constexpr uint8_t kNoOrigin = 0xFFu;

    /// A remote endpoint's participant prefix and announced UDP locators,
    /// shared by every stored endpoint that has both in common.
    struct RemoteOrigin {
        GuidPrefix prefix       = {};
        uint16_t   users        = 0;   ///< Stored endpoints naming this origin; 0 = free.
        uint8_t    locatorCount = 0;
        Locator    locators[RTPS_MAX_REMOTE_ENDPOINT_LOCATORS] = {};
    };

    struct RemoteEndpoint {
        EntityId entityId         = entity_id::UNKNOWN;
        uint32_t topicKey         = 0;   ///< name_key() of the topic name.
        uint32_t typeKey          = 0;   ///< name_key() of the type name.
        uint8_t  origin           = kNoOrigin;   ///< Index into origins_.
        uint8_t  reliability      = 0;   ///< ReliabilityKind.
        uint8_t  durability       = 0;   ///< DurabilityKind.
        bool     expectsInlineQos = false;
        bool valid() const noexcept { return origin != kNoOrigin; }
        ReliabilityKind reliability_kind() const noexcept {
            return static_cast<ReliabilityKind>(reliability);
        }
        DurabilityKind durability_kind() const noexcept {
            return static_cast<DurabilityKind>(durability);
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

    /// @brief The record for `g` in `table`, or, when `g` is GUID_UNKNOWN, a free record; else null.
    RemoteEndpoint* find_remote(RemoteEndpoint* table, uint32_t n, const Guid& g) noexcept;

    /// @brief The full GUID of a stored remote endpoint.
    Guid guid_of(const RemoteEndpoint& r) const noexcept {
        return Guid{origins_[r.origin].prefix, r.entityId};
    }

    /// @brief A stored remote endpoint as on_match reports it; its locators
    /// live in origins_ and stay valid while the record is stored.
    MatchedEndpoint matched(const RemoteEndpoint& r) const noexcept {
        const RemoteOrigin& o = origins_[r.origin];
        return MatchedEndpoint{guid_of(r), o.locators, o.locatorCount, r.expectsInlineQos};
    }

    /**
     * @brief Finds or makes room for a remote endpoint's record and origin.
     *
     * The record is the one already stored for `guid`, else a free one; the
     * origin is the one already holding `guid`'s prefix and `locators`, else
     * a free one. When either is missing and the endpoint completes a match
     * (`completes_match`), room is made as the Capacity rule in the file
     * comment says: evict_unmatched_origin() for an origin, then
     * displace_unmatched() for a record.
     *
     * @param[in,out] table           remote_writers_ or remote_readers_.
     * @param[in]     n               Entries in `table`.
     * @param[in]     is_writer       `true` when `table` holds writers.
     * @param[in]     guid            The announced endpoint's GUID.
     * @param[in]     locators        Its kept UDP locators (LocalSubnets::select()).
     * @param[in]     locator_count   Entries in `locators`.
     * @param[in]     completes_match pairs_with_local() for the endpoint.
     * @param[out]    origin          The origin index to store, when a record is returned.
     * @return The record to store the endpoint in, or null when it is not kept.
     *
     * Verification:
     * - Correct means: a stored endpoint in a match is never cleared, and an
     *   endpoint that completes a match is refused only when every record or
     *   every origin it could take belongs to a match.
     * - Covered by: TC-SEDP-8.5.5.1-008, TC-SEDP-8.5.5.1-009, TC-SEDP-8.5.5.1-011.
     */
    RemoteEndpoint* admit_remote(RemoteEndpoint* table, uint32_t n, bool is_writer,
                                 const Guid& guid, const Locator* locators, uint8_t locator_count,
                                 bool completes_match, uint8_t& origin) noexcept;

    /// @brief The origin holding `prefix` and `locators`, else a free one, else kNoOrigin.
    uint8_t find_origin(const GuidPrefix& prefix, const Locator* locators,
                        uint8_t locator_count) const noexcept;

    /**
     * @brief Frees an origin none of whose stored endpoints is in a match.
     *
     * Every stored endpoint of that origin, writer or reader, is forgotten
     * without notification; the caller's store fires on_change for them.
     *
     * @return The freed origin's index, or kNoOrigin when every origin in use
     *   has an endpoint in a match.
     *
     * Verification:
     * - Correct means: no endpoint in a match is forgotten.
     * - Covered by: TC-SEDP-8.5.5.1-008, TC-SEDP-8.5.5.1-011.
     */
    uint8_t evict_unmatched_origin() noexcept;

    /// @brief Stores an endpoint in `slot`, naming `origin`, and releases the origin it named before.
    void store_remote(RemoteEndpoint& slot, const Guid& guid, uint8_t origin,
                      const Locator* locators, uint8_t locator_count, uint32_t topic_key,
                      uint32_t type_key, ReliabilityKind reliability, DurabilityKind durability,
                      bool expects_inline_qos) noexcept;

    /// @brief Frees a stored remote endpoint and releases its origin.
    void clear_remote(RemoteEndpoint& r) noexcept;

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
     * The record is cleared, and its origin released, without notifying
     * anyone: the caller stores the announcement that needed the slot there,
     * and the on_change that store fires covers the forgotten record too. No
     * pair is dissolved, since a record in no match has none.
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

    /**
     * @brief Unmatches a stored remote endpoint that re-announced different locators.
     *
     * The proxies of a matched endpoint that announced locators of its own
     * refer to its origin's list rather than copying it (see "Locator
     * reference" in rtps/reader_proxy.hpp). Storing the re-announcement moves
     * the endpoint to another origin and can free the old one for another
     * participant, so every pair of the endpoint is dissolved first; the
     * caller then stores the announcement and reports those pairs again, and
     * the participant matches fresh proxies at the new locators.
     *
     * @param[in] table     remote_writers_ or remote_readers_.
     * @param[in] n         Entries in `table`.
     * @param[in] is_writer `true` when `table` holds writers.
     * @param[in] guid      The announced endpoint.
     * @param[in] locators  Its kept UDP locators (LocalSubnets::select()).
     * @param[in] count     Entries in `locators`.
     *
     * Verification:
     * - Correct means: no reported pair outlives a change of its remote
     *   endpoint's origin, so no proxy refers to a released origin; an
     *   unchanged re-announcement dissolves nothing.
     * - Covered by: TC-SEDP-8.5.5.1-012.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.5.4.2 (unicastLocatorList of discovered endpoints),
     *   §8.4.7.4 and §8.4.1.4 (matched_writer_remove, matched_reader_remove)
     */
    void unmatch_if_relocated(RemoteEndpoint* table, uint32_t n, bool is_writer, const Guid& guid,
                              const Locator* locators, uint8_t count) noexcept;
    /// @brief Tells the owner the set of known endpoints changed.
    void changed() noexcept { if (on_change_) on_change_(user_data_); }

    LocalEndpoint  local_writers_[RTPS_MAX_LOCAL_ENDPOINTS]     = {};
    LocalEndpoint  local_readers_[RTPS_MAX_LOCAL_ENDPOINTS]     = {};
    RemoteEndpoint remote_writers_[RTPS_MAX_DISCOVERED_WRITERS] = {};
    RemoteEndpoint remote_readers_[RTPS_MAX_DISCOVERED_READERS] = {};
    RemoteOrigin   origins_[RTPS_MAX_REMOTE_ORIGINS]            = {};
    MatchRecord    matches_[kMaxMatches]                        = {};

    /// Ranks against no addresses until set_local_subnets() is called.
    static const LocalSubnets kNoSubnets;

    const LocalSubnets* subnets_ = &kNoSubnets;
    OnMatch    on_match_;
    OnUnmatch  on_unmatch_;
    OnChange   on_change_;
    void*      user_data_;
    GuidPrefix local_prefix_ = {};
    bool       has_local_prefix_ = false;
};

} // namespace rtps
