/**
 * @file rtps/participant.hpp
 * @brief RTPSParticipant — §8.5.1 (OMG formal/2022-04-01).
 *
 * Writers
 * -------
 * create_writer() is a function template over the writer's history depth:
 *
 *   template<uint8_t CacheDepth = 1>
 *   StatefulWriter* create_writer(const WriterConfig&);
 *
 * so each writer carries its own history depth:
 *
 *   auto* w1 = participant.create_writer<1>(cfg);  // KEEP_LAST(1)
 *   auto* w8 = participant.create_writer<8>(cfg);  // KEEP_LAST(8)
 *
 * It constructs a SharedPoolStatefulWriter<CacheDepth>; the depth changes the
 * writer's data layout only, and every writer runs the one compiled
 * StatefulWriter. The participant keeps its writers in one table of
 * StatefulWriter* and routes ACKNACK, NACK_FRAG, ticks, and matching to
 * them; the application writes through the StatefulWriter* returned.
 *
 * Storage model
 * -------------
 * Writer slots are opaque byte arrays (aligned storage) constructed via
 * placement-new, each RTPS_PARTICIPANT_MAX_WRITER_BYTES long. A
 * SharedPoolStatefulWriter<D> that does not fit a slot fails a static_assert
 * at compile time. Reader slots each hold one SharedPoolStatefulReader.
 *
 * Matched-endpoint proxies
 * ------------------------
 * The proxies of the user endpoints' matches live in two participant-wide
 * pools (proxy_pool.hpp): RTPS_READER_PROXY_POOL ReaderProxies for the user
 * writers and RTPS_WRITER_PROXY_POOL WriterProxies for the user readers. An
 * endpoint binds a proxy when it matches and returns it when it unmatches.
 * Every endpoint slot has RTPS_MIN_MATCHES_PER_ENDPOINT proxies reserved
 * from the start, created or not, so an endpoint's first matches never
 * depend on the order endpoints are created in or on what other endpoints
 * have borrowed; a static_assert proves the pools hold those floors. An
 * endpoint above its floor borrows unreserved proxies up to its cap
 * (RTPS_MAX_MATCHED_READERS / RTPS_MAX_MATCHED_WRITERS, or the config's own).
 * The SEDP builtin endpoints keep private pools of RTPS_MAX_PEERS proxies,
 * so discovery never waits for a proxy a user endpoint holds. Design:
 * docs/design/shared_proxy_pools.md.
 *
 * Spec references (all OMG formal/2022-04-01)
 * -------------------------------------------
 *   RTPSParticipant            — §8.5.1,   p. 149
 *   Participant GUID           — §8.5.1.2, p. 151
 *   SPDP builtin endpoints     — §8.5.3.3, p. 162
 *   SEDP builtin endpoints     — §8.5.4.3, p. 167
 *   Endpoint matching          — §8.5.5.1, p. 169
 */

#pragma once

#include "types.hpp"
#include "stateful_writer.hpp"
#include "stateful_reader.hpp"
#include "spdp.hpp"
#include "sedp.hpp"
#include "endpoint_match_table.hpp"
#include "message_parser.hpp"
#include <new>

namespace rtps {

// ============================================================
// GUID prefix
// ============================================================
/**
 * @brief Builds the GUID prefix of a local participant.
 *
 * Layout (§9.3.1.5 recommends the vendor ID in the first two octets):
 *   0-1   VENDORID_THIS_IMPL
 *   2-5   platform::device_unique_id() hashed to 32 bits (FNV-1a): tells
 *         devices, and processes on a host, apart
 *   6-9   platform::boot_nonce(), big-endian: tells successive boots of one
 *         device apart
 *   10    instance counter: differs for each participant this process
 *         creates
 *   11    participant_id
 *
 * A participant must never reuse its predecessor's GUID while peers still
 * hold the predecessor's lease, whether the predecessor was shut down by this
 * process (the counter) or lost to a reset (the nonce). A peer that sees a
 * known GUID announce again treats it as a lease refresh and keeps its
 * per-writer state, so the new participant's samples, numbered from 1 again,
 * are dropped as already received, and discovery and services stay broken
 * until the peer restarts or the old lease runs out.
 *
 * @param[in] participant_id Participant ID from the port mapping (§9.6.1.1).
 * @return The 12-octet prefix; participant and endpoint GUIDs share it.
 *
 * Verification:
 * - Correct means: distinct across devices, processes, boots of one device,
 *   and successive participants of one process; octets 0-1 are the vendor ID,
 *   6-9 the boot nonce, and 11 equals `participant_id`.
 * - Covered by: TC-PART-8.5.1-003 (layout); a device reset while a ROS 2
 *   host stays up, the host then rediscovering it (boots).
 *
 * References:
 * - OMG DDSI-RTPS 2.5 §8.2.4.2 (GuidPrefix_t), §9.3.1.5 (vendor ID octets)
 */
GuidPrefix make_participant_guid_prefix(uint8_t participant_id) noexcept;

// ============================================================
// Configuration
// ============================================================

#ifndef RTPS_MAX_USER_WRITERS
#  define RTPS_MAX_USER_WRITERS 4
#endif
#ifndef RTPS_MAX_USER_READERS
#  define RTPS_MAX_USER_READERS 4
#endif
#ifndef RTPS_TICK_PERIOD_MS
#  define RTPS_TICK_PERIOD_MS 10u
#endif

/// 1: SEDP rebuilds each local endpoint's announcement for every send from
/// the endpoint table, so no pool block stays allocated per endpoint; 0:
/// SEDP serializes each announcement once and keeps it (see "Announcement
/// storage" in sedp.hpp). Firmware capacity profiles select 1, where the
/// 512-octet blocks are scarce; hosts keep 0, where replaying stored bytes
/// to many peers is cheaper than serializing them again.
#ifndef RTPS_SEDP_REBUILD_ANNOUNCEMENTS
#define RTPS_SEDP_REBUILD_ANNOUNCEMENTS 0
#endif

/// Maximum byte size of one writer slot in the participant's storage table.
/// Must be >= sizeof(SharedPoolStatefulWriter<D>) for every depth D you intend to use.
/// Default covers SharedPoolStatefulWriter<8>.
#ifndef RTPS_PARTICIPANT_MAX_WRITER_BYTES
#define RTPS_PARTICIPANT_MAX_WRITER_BYTES (sizeof(SharedPoolStatefulWriter<8>))
#endif

/// ReaderProxies shared by the user writers: remote readers matched across
/// all of them at once. The default is the per-writer cap for every slot.
#ifndef RTPS_READER_PROXY_POOL
#define RTPS_READER_PROXY_POOL (RTPS_MAX_USER_WRITERS * RTPS_MAX_MATCHED_READERS)
#endif

/// WriterProxies shared by the user readers: remote writers matched across
/// all of them at once. The default is the per-reader cap for every slot.
#ifndef RTPS_WRITER_PROXY_POOL
#define RTPS_WRITER_PROXY_POOL (RTPS_MAX_USER_READERS * RTPS_MAX_MATCHED_WRITERS)
#endif

static_assert(
    RTPS_READER_PROXY_POOL >= RTPS_MAX_USER_WRITERS * RTPS_MIN_MATCHES_PER_ENDPOINT,
    "RTPS_READER_PROXY_POOL cannot hold RTPS_MIN_MATCHES_PER_ENDPOINT for every writer slot");
static_assert(
    RTPS_WRITER_PROXY_POOL >= RTPS_MAX_USER_READERS * RTPS_MIN_MATCHES_PER_ENDPOINT,
    "RTPS_WRITER_PROXY_POOL cannot hold RTPS_MIN_MATCHES_PER_ENDPOINT for every reader slot");
static_assert(RTPS_MIN_MATCHES_PER_ENDPOINT <= RTPS_MAX_MATCHED_READERS
                  && RTPS_MIN_MATCHES_PER_ENDPOINT <= RTPS_MAX_MATCHED_WRITERS,
              "an endpoint's floor cannot exceed its cap");

// ============================================================
// WriterConfig / ReaderConfig
// ============================================================

struct WriterConfig {
    EntityId        entityId    = entity_id::UNKNOWN;
    TopicName       topicName   = {};
    TopicName       typeName    = {};
    ReliabilityKind reliability = ReliabilityKind::RELIABLE;
    DurabilityKind  durability  = DurabilityKind::VOLATILE;
    TopicKind       topicKind   = TopicKind::NO_KEY;
    uint32_t        nackDelayMs = RTPS_NACK_RESPONSE_DELAY_MS;
    uint32_t        hbPeriodMs  = RTPS_HEARTBEAT_PERIOD_MS;
    /// A service reply writer: drop each reply once its requester has it
    /// (StatefulWriter::release_replies_on_delivery()).
    bool            releaseRepliesOnDelivery = false;
    /// Readers always granted a proxy (the floor); raised to
    /// RTPS_MIN_MATCHES_PER_ENDPOINT when lower. The part above that is
    /// reserved in the pool when the writer is created.
    uint8_t minMatchedReaders = RTPS_MIN_MATCHES_PER_ENDPOINT;
    /// Most readers matched at once (the cap); raised to the floor when lower.
    uint8_t maxMatchedReaders = RTPS_MAX_MATCHED_READERS;
};

struct ReaderConfig {
    EntityId        entityId    = entity_id::UNKNOWN;
    TopicName       topicName   = {};
    TopicName       typeName    = {};
    ReliabilityKind reliability = ReliabilityKind::RELIABLE;
    DurabilityKind  durability  = DurabilityKind::VOLATILE;
    TopicKind       topicKind   = TopicKind::NO_KEY;
    uint32_t        hbResponseMs= RTPS_HEARTBEAT_RESPONSE_DELAY_MS;
    /// Writers always granted a proxy (the floor), as WriterConfig::minMatchedReaders.
    uint8_t minMatchedWriters = RTPS_MIN_MATCHES_PER_ENDPOINT;
    /// Most writers matched at once (the cap); raised to the floor when lower.
    uint8_t maxMatchedWriters = RTPS_MAX_MATCHED_WRITERS;
};

// ============================================================
// RTPSParticipant
// ============================================================

class RTPSParticipant {
public:

    RTPSParticipant(uint8_t     domain_id,
                    uint8_t     participant_id,
                    ITransport& transport,
                    VendorId    vendor_id = VENDORID_THIS_IMPL) noexcept;

    RTPSParticipant(const RTPSParticipant&)            = delete;
    RTPSParticipant& operator=(const RTPSParticipant&) = delete;

    /**
     * @brief Creates a writer with a KEEP_LAST(CacheDepth) history in a free writer slot.
     *
     * Constructs a SharedPoolStatefulWriter<CacheDepth> in the
     * participant's writer storage, assigns its entity ID, and registers the
     * endpoint so SEDP announces it (§8.5.4). The writer binds its reader
     * proxies from the participant's ReaderProxy pool, with the floor and
     * cap `cfg` names; the slot's RTPS_MIN_MATCHES_PER_ENDPOINT is already
     * reserved, and any floor above it is reserved here.
     *
     * @tparam CacheDepth KEEP_LAST depth of the writer's history (§8.3.3).
     * @param[in] cfg Topic, type, QoS, timing, floor and cap of the writer.
     * @return The writer, owned by the participant until delete_writer();
     *   null when the slot table is full, a name exceeds the name limit, or
     *   the pool cannot reserve a floor above RTPS_MIN_MATCHES_PER_ENDPOINT.
     *
     * Verification:
     * - Correct means: the writer is announced through SEDP, binds proxies
     *   by ProxyPool admission with its floor reserved, and its slot and
     *   reservation are reused after delete_writer().
     * - Covered by: TC-PART-8.5.1-* (tests/spec/src/test_participant_8_5_1.cpp),
     *   TC-PART-8.5.1-011.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.5.1 (the participant creates its endpoints),
     *   §8.4.1.1 (ReaderProxy)
     *
     * Example:
     * @code{.cpp}
     * rtps::WriterConfig cfg{};
     * cfg.topicName.assign("rt/chatter");
     * cfg.typeName.assign("std_msgs::msg::dds_::String_");
     * rtps::StatefulWriter* writer = participant.create_writer<4>(cfg);
     * if (writer == nullptr) {
     *     return;  // slot table full, or a floor the pool cannot reserve
     * }
     * writer->write_async(rtps::ChangeKind::ALIVE, cdr, cdr_size);
     * participant.delete_writer(writer);
     * @endcode
     * Full program: examples/rtps_chatter/main.cpp
     */
    template <uint8_t CacheDepth = 1>
    StatefulWriter* create_writer(const WriterConfig& cfg) noexcept {
        using W = SharedPoolStatefulWriter<CacheDepth>;

        static_assert(sizeof(W) <= RTPS_PARTICIPANT_MAX_WRITER_BYTES,
                      "SharedPoolStatefulWriter<D> exceeds RTPS_PARTICIPANT_MAX_WRITER_BYTES. "
                      "Increase RTPS_PARTICIPANT_MAX_WRITER_BYTES in your config.");
        static_assert(alignof(W) <= alignof(SharedPoolStatefulWriter<8>),
                      "writer storage is aligned for SharedPoolStatefulWriter<8>");

        if (!endpoint_names_fit(cfg.topicName, cfg.typeName)) return nullptr;

        // Find a free slot.
        int slot = find_free_writer_slot();
        if (slot < 0) return nullptr;

        uint8_t floor = 0u;
        uint8_t cap = 0u;
        if (!reserve_endpoint_floor(reader_proxies_, cfg.minMatchedReaders, cfg.maxMatchedReaders,
                                    floor, cap)) {
            return nullptr;
        }

        const Guid writer_guid{ participant_guid_.prefix,
                                assign_entity_id(cfg.entityId, true, cfg.topicKind) };

        // Placement-new into the pre-allocated, correctly aligned storage.
        W* w = new (writer_storage_[slot])
            W(writer_guid, cfg.reliability, cfg.durability, transport_, reader_proxies_, floor, cap,
              cfg.nackDelayMs, cfg.hbPeriodMs);
        w->release_replies_on_delivery(cfg.releaseRepliesOnDelivery);

        writers_[slot] = w;
        writer_alive_[slot] = true;

        // Register endpoint for SEDP announcements.
        register_writer_endpoint(writer_guid, cfg);

        return w;
    }

    /**
     * @brief Creates a reader in a free reader slot.
     *
     * Every user reader is a SharedPoolStatefulReader: it binds its writer
     * proxies from the participant's WriterProxy pool, with the floor and
     * cap `cfg` names (by default RTPS_MIN_MATCHES_PER_ENDPOINT and
     * RTPS_MAX_MATCHED_WRITERS), and has no history-depth parameter.
     *
     * @param[in] cfg             Topic, type, QoS, floor and cap of the reader.
     * @param[in] on_change_avail Callback for each delivered change; null discards.
     * @param[in] user_data       Forwarded to on_change_avail.
     * @return The reader, owned by the participant until delete_reader();
     *   null when the slot table is full, a name exceeds the name limit, or
     *   the pool cannot reserve a floor above RTPS_MIN_MATCHES_PER_ENDPOINT.
     *
     * Verification:
     * - Correct means: as for create_writer(), with the WriterProxy pool.
     * - Covered by: TC-PART-8.5.1-005, TC-PART-8.5.1-011.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.5.1, §8.4.7.1 (WriterProxy)
     */
    StatefulReader* create_reader(const ReaderConfig&               cfg,
                                  StatefulReader::OnChangeAvailable on_change_avail = nullptr,
                                  void*                             user_data       = nullptr) noexcept;

    // ----------------------------------------------------------
    // delete_writer() / delete_reader()
    //
    // Destroy a local endpoint created by create_writer()/create_reader():
    // withdraw it from SEDP (peers receive a disposal and unmatch it),
    // dissolve its local matches, run its destructor, and free its slot
    // for reuse.
    //
    // §8.5.1, p. 150 (Participant owns endpoint lifecycle); §8.5.4.1
    // (endpoint removal is announced through SEDP).
    //
    // Verification: TC-PART-8.5.1-005 (slot reuse and disposal sent).
    //
    // @returns false when the pointer is not a live endpoint of this
    //          participant.
    // ----------------------------------------------------------
    bool delete_writer(StatefulWriter* writer) noexcept;
    bool delete_reader(StatefulReader* reader) noexcept;

    // ----------------------------------------------------------
    // parse_received() — §8.3.4, p. 69.
    // ----------------------------------------------------------
    ParseResult parse_received(const Octet* buf, uint32_t len) noexcept;

    /**
     * @brief Runs the participant's timed work: SPDP, lease expiry, waiting
     *   matches, and every endpoint's heartbeats and repairs.
     *
     * The driver calls this whenever ms_until_next_timer() has elapsed or a
     * datagram was handled. Nothing runs until the participant has an
     * address.
     *
     * After a user endpoint's proxy frees (an unmatch or a deleted
     * endpoint), each tick binds at most one waiting match, the oldest the
     * pools can now serve (EndpointMatchTable::match_oldest_waiting()), and
     * keeps doing so on later ticks until none can bind. A tick therefore
     * replays at most one TRANSIENT_LOCAL history for a waiting match.
     *
     * @param[in] now_ms Current platform::monotonic_ms().
     *
     * Verification:
     * - Correct means: no timer runs late by more than the driver's wake-up
     *   latency, and a waiting match binds within one tick per match ahead
     *   of it once a proxy is free for it.
     * - Covered by: TC-PART-8.5.1-006, TC-PART-8.4.1.4-001, TC-PART-8.4.1.4-002.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.5.3.2 (lease), §8.4.1.4 (matched_reader_add),
     *   §8.4.7.4 (matched_writer_add)
     */
    void tick(uint32_t now_ms) noexcept;

    // ----------------------------------------------------------
    // ms_until_next_timer()
    //
    // How long tick() can be deferred: the soonest of the SPDP
    // announcement, any peer lease expiry, and every builtin and user
    // writer's and reader's pending timer, or now while a freed proxy may
    // let a waiting match bind. A driver that sleeps between ticks waits no
    // longer than this, so no heartbeat, repair, lease expiry, or waiting
    // match is late.
    //
    // Verification: TC-PART-8.5.1-006.
    //
    // @param now_ms  Current platform::monotonic_ms().
    // @returns Milliseconds until the next timed action; 0 when one is due;
    //          UINT32_MAX when nothing is scheduled (no addresses yet).
    // ----------------------------------------------------------
    uint32_t ms_until_next_timer(uint32_t now_ms) const noexcept;

    // ----------------------------------------------------------
    // announce() — force immediate SPDP send.
    // ----------------------------------------------------------
    void announce() noexcept;

    // ----------------------------------------------------------
    // Address configuration (application-managed)
    //
    // The application is responsible for adding/removing local
    // interface addresses as they become available/unavailable.
    // ----------------------------------------------------------

    /**
     * @brief Announces an IPv4 address this participant can be reached at.
     *
     * The address becomes a metatraffic and a default unicast locator in the
     * next SPDP announcement (§8.5.3.2). With its on-link prefix, it also
     * decides which of a peer's announced locators this participant sends to:
     * the SPDP peer table and the endpoint match table rank every locator
     * list against the addresses held here (rtps/locator_ranking.hpp).
     *
     * @param[in] ipv4_be    The address as a number (192.168.1.5 is
     *   0xC0A80105).
     * @param[in] prefix_len On-link prefix length in bits (24 for a
     *   255.255.255.0 netmask), at most 32; 0 when the netmask is unknown, in
     *   which case no peer address counts as on link.
     * @return `false` when RTPS_MAX_LOCATORS_PER_PARTICIPANT addresses are
     *   already announced; `true` once the address is announced. Adding an
     *   announced address again replaces its prefix length.
     *
     * Verification:
     * - Correct means: the next SPDP announcement lists the address at the
     *   metatraffic and default unicast ports, and a peer's locator inside
     *   the prefix is preferred over its other locators.
     * - Covered by: TC-PART-8.5.1-002, TC-PART-8.5.1-010.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.5.3.2, §9.6.1.1 (port mapping); RFC 6724 §6
     *
     * Example:
     * @code{.cpp}
     * const auto& v4 = rosette::net::board_address_snapshot().v4;
     * participant.add_ipv4_address(rosette::net::board_ipv4_address(),
     *                              v4.primary().prefix_len);
     * @endcode
     * Full program: examples/rtps_chatter/main.cpp
     */
    bool add_ipv4_address(uint32_t ipv4_be, uint8_t prefix_len = 0u) noexcept;

    /**
     * @brief As add_ipv4_address(), for an IPv6 address.
     *
     * @param[in] ipv6       The 16 address octets in network order.
     * @param[in] prefix_len On-link prefix length in bits (64 for a SLAAC
     *   address), at most 128; 0 when unknown.
     * @return As add_ipv4_address().
     */
    bool add_ipv6_address(const uint8_t ipv6[16], uint8_t prefix_len = 0u) noexcept;

    bool remove_ipv4_address(uint32_t ipv4_be) noexcept;
    bool remove_ipv6_address(const uint8_t ipv6[16]) noexcept;
    void clear_addresses() noexcept;
    bool has_addresses() const noexcept;

    /// Called when the discovered graph changed; see set_on_graph_change().
    using OnGraphChange = void (*)(void* user_data) noexcept;

    /**
     * @brief Registers the one observer told when the discovered graph changes.
     *
     * The graph is what discovery knows: the participants SPDP has found
     * and the endpoints, local or remote, in the match table. The observer
     * fires when a participant is discovered or leaves, and when an
     * endpoint is recorded, changes topic or type, or is forgotten. It is
     * a hint to re-query (for rmw, to trigger the graph guard condition),
     * not a description of the change; several changes may fire it several
     * times.
     *
     * The observer runs on the thread that drives the participant, inside
     * parse_received(), tick(), or endpoint creation and deletion, with
     * whatever lock that caller holds; it must not call back into the
     * participant.
     *
     * @param[in] fn        Observer, or null to clear it.
     * @param[in] user_data Passed to `fn` unchanged.
     *
     * Verification:
     * - Correct means: the observer fires when SPDP adds or removes a peer
     *   and when SEDP adds or withdraws an endpoint, and not for a repeated
     *   identical announcement.
     * - Covered by: TC-PART-8.5.1-009.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.5.3 (SPDP), §8.5.4 (SEDP)
     * - rmw_dds_common/context.hpp (Humble), `graph_guard_condition`
     */
    void set_on_graph_change(OnGraphChange fn, void* user_data) noexcept {
        on_graph_change_ = fn;
        graph_change_user_ = user_data;
    }

    /**
     * @brief Names the local writer that every participant on the network reads.
     *
     * A ROS 2 participant's `ros_discovery_info` writer is matched by every
     * other ROS 2 participant, so a match with it says nothing about whether
     * a peer needs to stay active. The participant ignores this writer's
     * matches when it chooses a peer to park, and requires instead that the
     * peer's readers have acknowledged everything it wrote; peers parked
     * while it changes are reactivated to receive the change (see
     * participant_peer_tiers.cpp).
     *
     * @param[in] writer_guid The writer's GUID, or GUID_UNKNOWN for none.
     *
     * Verification:
     * - Correct means: a peer whose only match is with this writer is
     *   parked once settled; a peer matching any other local endpoint is not.
     * - Covered by: TC-PART-8.5.3-012.
     */
    void set_discovery_writer(const Guid& writer_guid) noexcept {
        discovery_writer_guid_ = writer_guid;
    }

    /**
     * @brief Owes every parked peer the discovery writer's new sample.
     *
     * A parked peer's readers are unmatched from the discovery writer
     * (set_discovery_writer()), so a sample written while it is parked never
     * reaches it. The writer's owner calls this after writing a sample whose
     * contents changed, which marks every parked peer owed; each is then
     * reactivated in turn, its reader of the discovery writer is matched
     * again, and the TRANSIENT_LOCAL history delivers the new sample. A
     * republish of identical contents does not call it.
     *
     * Verification:
     * - Correct means: a stock peer parked before a node is added or removed
     *   lists the device's new node set once the change is written.
     * - Covered by: TC-PART-8.5.3-013.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.7.2.2.1 (TRANSIENT_LOCAL history to new readers)
     * - rmw_dds_common/msg/ParticipantEntitiesInfo.msg (Humble)
     */
    void discovery_info_changed() noexcept { owe_parked_peers(); }

    // ----------------------------------------------------------
    // Accessors
    // ----------------------------------------------------------
    const Guid&       participant_guid() const noexcept { return participant_guid_; }
    const GuidPrefix& guid_prefix()      const noexcept { return participant_guid_.prefix; }
    uint8_t           domain_id()        const noexcept { return domain_id_; }
    SpdpReader&       spdp_reader()            noexcept { return spdp_reader_; }
    EndpointMatchTable& match_table()          noexcept { return match_table_; }

    /// @brief The ReaderProxy pool the user writers bind from.
    const ProxyPool& reader_proxy_pool() const noexcept { return reader_proxies_; }

    /// @brief The WriterProxy pool the user readers bind from.
    const ProxyPool& writer_proxy_pool() const noexcept { return writer_proxies_; }

    // ----------------------------------------------------------
    // assign_entity_id()
    //
    // Returns `requested` unchanged when the caller chose an EntityId,
    // and otherwise allocates a fresh user-defined one: a 3-octet
    // entityKey from a per-participant counter (starting at 1, never
    // reused) and the entityKind Table 9.1 prescribes for a user writer
    // (0x02 with key, 0x03 without) or reader (0x07 with key, 0x04
    // without). create_writer()/create_reader() call this, so endpoints
    // created from a default WriterConfig/ReaderConfig get distinct GUIDs.
    //
    // §8.2.4.3, p. 29: "the EntityId_t uniquely identifies the Endpoint
    //  within the Participant"; §9.3.1.2, Table 9.1 (entityKind values).
    //
    // Verification: TC-PART-8.2.4-001 (tests/spec) checks that default
    // endpoints get distinct, correctly-kinded IDs and explicit IDs are kept.
    // ----------------------------------------------------------
    EntityId assign_entity_id(const EntityId& requested,
                              bool            is_writer,
                              TopicKind       topic_kind) noexcept;

private:

    /// Next entityKey handed out by assign_entity_id().
    uint32_t next_entity_key_ = 1u;

    // ── Discovery callbacks ───────────────────────────────────
    static void on_spdp_new_peer    (const PeerEntry&, void*) noexcept;
    static void on_spdp_peer_removed(const PeerEntry&, void*) noexcept;
    /**
     * @brief Binds a proxy for every local half of a matched pair.
     *
     * The match table's OnMatch: called once when a pair is reported and
     * again for a waiting pair from match_oldest_waiting(). A half that
     * already holds its proxy is left alone, so offering a pair again binds
     * only the half that waited.
     *
     * @param[in] writer The pair's writer, with the locators it announced.
     * @param[in] reader The pair's reader, with the locators it announced.
     * @param[in] topic  The local endpoint's topic name, for the log.
     * @param[in] self   The participant.
     * @return `true` when every local half holds its proxy; `false` when a
     *   pool or an endpoint's cap refused one, and the pair waits.
     *
     * Verification:
     * - Correct means: a half is bound at most once; a refused half leaves
     *   the pool unchanged.
     * - Covered by: TC-PART-8.5.1-011, TC-PART-8.4.1.4-001, TC-PART-8.4.1.4-003.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.4.1.4 (matched_reader_add), §8.4.7.4 (matched_writer_add)
     */
    static bool on_endpoint_match(const MatchedEndpoint& writer, const MatchedEndpoint& reader,
                                  const TopicName& topic, void* self) noexcept;

    /// @brief The match table's MayMatch: whether every local half of the pair
    /// that holds no proxy yet could bind one now.
    static bool may_bind_pair(const Guid& writer_guid, const Guid& reader_guid,
                              void* self) noexcept;
    static void on_endpoint_unmatch (const Guid&, const Guid&, void*) noexcept;
    static void on_endpoints_changed(void*) noexcept;

    /**
     * @brief The LocalEndpointSource SEDP rebuilds announcements from, or an
     *   invalid one when RTPS_SEDP_REBUILD_ANNOUNCEMENTS is 0.
     *
     * Called while the participant is constructed; the source only captures
     * `this`, and its functions run after construction.
     */
    LocalEndpointSource sedp_endpoint_source() noexcept;

    /**
     * @brief LocalEndpointSource::DescribeWriter over the match table's
     *   record of a local writer plus the participant's unicast locators.
     *
     * @param[in]  user The participant.
     * @param[in]  guid A local writer's GUID.
     * @param[out] out  The writer's DiscoveredWriterData as
     *   register_writer_endpoint() announced it, with the current locators.
     * @return `false` when the writer is not (or no longer) in the table.
     *
     * Verification:
     * - Correct means: SEDP rebuilds the same PL_CDR bytes it would have
     *   stored at creation, apart from locators added or removed since.
     * - Covered by: TC-SEDP-8.5.4-017.
     */
    static bool describe_local_writer(void* user, const Guid& guid,
                                      DiscoveredWriterData& out) noexcept;

    /// @brief As describe_local_writer(), for a local reader.
    static bool describe_local_reader(void* user, const Guid& guid,
                                      DiscoveredReaderData& out) noexcept;

    // ── Peer tiers (participant_peer_tiers.cpp) ─────────────

    /**
     * @brief Makes a pending or parked peer active, parking a settled one when the tier is full.
     *
     * SPDP calls this for a new peer and again for every announcement of a
     * pending one; SEDP traffic from a parked or pending peer calls it too
     * (on_announcer_traffic()). The active tier holds at most RTPS_MAX_PEERS
     * peers, the capacity of the builtin SEDP endpoints' proxies.
     *
     * @param[in,out] peer   The peer's record in the SPDP table.
     * @param[in]     now_ms rosette::platform::monotonic_ms().
     * @return `true` when the peer is active on return.
     *
     * Verification:
     * - Correct means: a peer is admitted while fewer than RTPS_MAX_PEERS
     *   are active, or when a settled peer can be parked for it, and waits
     *   otherwise; a peer with a match that counts is never parked for it.
     * - Covered by: TC-PART-8.5.3-011, TC-PART-8.5.3-012,
     *   tests/ros2/run_rmw_peer_lockout_loopback.sh.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.5.3.2 (SPDP), §8.5.4.1 (SEDP builtin endpoints)
     */
    bool admit_peer(PeerEntry& peer, uint32_t now_ms) noexcept;

    /// @brief Matches a peer's builtin SEDP endpoints, which replays this
    /// participant's announcements to it and requests all of its own.
    void activate_peer(PeerEntry& peer, uint32_t now_ms) noexcept;

    /**
     * @brief True when an active peer can be parked without either side missing anything.
     *
     * Settled means: no SEDP DATA from the peer for RTPS_PEER_SETTLE_MS;
     * every announcement it sent has been received; its readers have
     * acknowledged everything this participant's SEDP and user writers
     * wrote; and none of its endpoints is in a match other than with the
     * discovery writer (set_discovery_writer()).
     *
     * @param[in] peer   An active peer's record.
     * @param[in] now_ms rosette::platform::monotonic_ms().
     * @return `true` when park_peer() may be applied to it.
     */
    bool peer_is_settled(const PeerEntry& peer, uint32_t now_ms) noexcept;

    /**
     * @brief Moves a settled peer from the active tier to the parked one.
     *
     * The highest sequence number received from each of its SEDP announcers
     * is kept in its record, then its builtin SEDP proxies and its remote
     * endpoints are removed, which also unmatches its readers of the
     * discovery writer. The peer itself notices nothing: this participant
     * keeps announcing itself, and the peer already holds all of it.
     *
     * @param[in,out] peer A settled peer (peer_is_settled()).
     */
    void park_peer(PeerEntry& peer) noexcept;

    /// @brief Parks the active peer that has been settled longest; `false` when none is settled.
    bool park_settled_peer(uint32_t now_ms) noexcept;

    /**
     * @brief Notes a DATA or HEARTBEAT from a peer's SEDP announcer before it is routed.
     *
     * DATA from an active peer restarts its settle time. A parked peer
     * whose announcer sends a sequence number above the one kept when it
     * was parked has a new or withdrawn endpoint, and a pending peer is
     * still waiting for a slot: both are admitted, when a slot can be made,
     * before the submessage is routed, so the new proxy processes it.
     *
     * @param[in] prefix    GUID prefix of the sending participant.
     * @param[in] writer_id The SEDP announcer that sent it.
     * @param[in] sn        DATA writerSN, or HEARTBEAT lastSN.
     * @param[in] is_data   `true` for DATA.
     * @param[in] now_ms    rosette::platform::monotonic_ms().
     */
    void on_announcer_traffic(const GuidPrefix& prefix, const EntityId& writer_id,
                              const SequenceNumber& sn, bool is_data, uint32_t now_ms) noexcept;

    /// @brief Marks every parked peer owed this participant's changed announcements.
    void owe_parked_peers() noexcept;

    /// @brief Reactivates one owed parked peer when a slot is free or can be made.
    void serve_owed_peer(uint32_t now_ms) noexcept;

    /// @brief Tells the graph observer, if any, that the graph changed.
    void notify_graph_change() noexcept {
        if (on_graph_change_) on_graph_change_(graph_change_user_);
    }

    // ── ParseOps trampolines ─────────────────────────────────
    static void dispatch_data          (const ParseContext&, const ParsedData&) noexcept;
    static void dispatch_data_frag     (const ParseContext&, const ParsedDataFrag&) noexcept;
    static void dispatch_heartbeat     (const ParseContext&, const ParsedHeartbeat&) noexcept;
    static void dispatch_heartbeat_frag(const ParseContext&, const ParsedHeartbeatFrag&) noexcept;
    static void dispatch_acknack       (const ParseContext&, const ParsedAckNack&) noexcept;
    static void dispatch_nack_frag     (const ParseContext&, const ParsedNackFrag&) noexcept;
    static void dispatch_gap           (const ParseContext&, const ParsedGap&) noexcept;

    // ── Internal helpers ─────────────────────────────────────
    int find_free_writer_slot() const noexcept;

    /**
     * @brief Settles a new endpoint's floor and cap and reserves the floor above its slot's.
     *
     * Every endpoint slot has RTPS_MIN_MATCHES_PER_ENDPOINT proxies reserved
     * from construction, so only a floor above that needs a reservation now.
     * A requested floor below it is raised to it, and a cap below the floor
     * (or 0) is raised to the floor.
     *
     * @param[in,out] pool          The pool the endpoint will bind from.
     * @param[in]     requested_min The config's floor.
     * @param[in]     requested_max The config's cap.
     * @param[out]    floor         The endpoint's floor.
     * @param[out]    cap           The endpoint's cap.
     * @return `false`, with nothing reserved, when the pool has too few
     *   unpromised proxies for the floor above the slot's.
     *
     * Verification:
     * - Correct means: after every create and delete, the pool's reserved
     *   count equals the unused floors of all slots, empty ones counting
     *   RTPS_MIN_MATCHES_PER_ENDPOINT.
     * - Covered by: TC-PART-8.5.1-011.
     */
    static bool reserve_endpoint_floor(ProxyPool& pool, uint8_t requested_min,
                                       uint8_t requested_max, uint8_t& floor,
                                       uint8_t& cap) noexcept;

    /// @brief Returns a deleted endpoint's floor above its slot's to the pool
    ///   (the reverse of reserve_endpoint_floor()).
    static void release_endpoint_floor(ProxyPool& pool, uint8_t floor) noexcept;

    /**
     * @brief True when a local endpoint's names were stored whole.
     *
     * An endpoint announced under a truncated name would be matched by
     * whatever shares the prefix, so create_writer() and create_reader()
     * refuse it instead (and log why).
     *
     * @param[in] topic The endpoint's topic name.
     * @param[in] type  The endpoint's type name.
     * @return `false` when either name did not fit a TopicName.
     *
     * Verification:
     * - Correct means: no local endpoint with a truncated name is created.
     * - Covered by: TC-PART-8.5.1-008.
     */
    static bool endpoint_names_fit(const TopicName& topic, const TopicName& type) noexcept;

    /// @brief The local writer (SEDP builtin or user) with entity ID `eid`, or null.
    StatefulWriter* find_writer(const EntityId& eid) noexcept;
    StatefulReader* find_reader(const EntityId& eid) noexcept;

    /**
     * @brief Resolves the record a matched endpoint's proxy refers to for its locators.
     *
     * Proxies keep a pointer into the list they are matched with and follow
     * it as its owner updates it (see "Locator reference" in
     * rtps/reader_proxy.hpp), so this chooses the owner. The
     * EndpointMatchTable reports an endpoint that announced no unicast
     * locators, or exactly its participant's defaultUnicastLocatorList (Fast
     * DDS and Rosette endpoints do), with none; such an endpoint is resolved
     * to that list in the peer's active slot (peer_default_locators()), or
     * in this participant's own data for a local endpoint. A peer that moves
     * to a new address re-announces SPDP, the slot takes the new address in
     * place, and every such proxy sends there next. An endpoint that
     * announced other locators keeps its own list, which lives in an
     * EndpointMatchTable locator list.
     *
     * @param[in]     remote_prefix GUID prefix of the endpoint's participant
     *   (this participant's own prefix for a local endpoint).
     * @param[in,out] locators      The reported list (the table's, or null);
     *   replaced by the participant's default list when `count` is 0. Either
     *   list outlives the match: the active slot until the peer is parked
     *   or removed, which unmatches its endpoints first; the table's list while the
     *   endpoint is in a match.
     * @param[in,out] count         Entries in `locators`; stays 0 when the
     *   participant is unknown or announced no defaults either.
     *
     * Verification:
     * - Correct means: DATA to a matched remote reader and ACKNACKs to a
     *   matched remote writer that announced no locators, or its
     *   participant's defaults, go to its participant's current default
     *   unicast locator, including after an SPDP refresh changes it.
     * - Covered by: TC-PART-8.5.1-007, TC-PART-8.5.3-001.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.5.3.2 (defaultUnicastLocatorList),
     *   §8.5.4.2 (DiscoveredWriterData / DiscoveredReaderData)
     */
    void resolve_endpoint_locators(const GuidPrefix& remote_prefix,
                                   const Locator*&   locators,
                                   uint8_t&          count) const noexcept;

    /**
     * @brief The default unicast locators the SPDP peer table keeps for an active peer.
     *
     * The EndpointMatchTable's lookup (set_participant_defaults()): the table
     * compares a remote endpoint's kept locators with these and keeps no
     * list for an endpoint announcing exactly them, and
     * resolve_endpoint_locators() reaches such an endpoint here.
     *
     * @param[in]  prefix    GUID prefix of the remote participant.
     * @param[out] locators  The defaultUnicastLocatorList of the peer's active
     *   slot; valid until the peer is parked or removed, either of which
     *   unmatches its endpoints first. Written only when the peer is found.
     * @param[out] count     Entries in `locators`; written only when found.
     * @return `false` when no active peer has `prefix`: only active peers
     *   have endpoints in the match table.
     *
     * Verification:
     * - Correct means: the result is the active slot's own list, so a
     *   refresh that rewrites it in place is seen through it.
     * - Covered by: TC-PART-8.5.1-007, TC-PART-8.5.3-001.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.5.3.2 (defaultUnicastLocatorList)
     */
    bool peer_default_locators(const GuidPrefix& prefix, const Locator*& locators,
                               uint8_t& count) const noexcept;

    /**
     * @brief The vendor a remote participant announced in SPDP.
     *
     * @param[in] remote_prefix GUID prefix of the participant.
     * @return Its PID_VENDOR_ID; VENDORID_UNKNOWN for this participant and for
     *   a prefix with no peer record.
     */
    VendorId peer_vendor_id(const GuidPrefix& remote_prefix) const noexcept;
    template <typename Deliver>
    void for_each_addressed_reader(const EntityId& writer_id, const EntityId& reader_id,
                                   Deliver&& deliver) noexcept;

    /// @brief The user reader in slot `i`; valid only while reader_alive_[i].
    SharedPoolStatefulReader* user_reader(int i) noexcept {
        return reinterpret_cast<SharedPoolStatefulReader*>(reader_storage_[i]);
    }

    const SharedPoolStatefulReader* user_reader(int i) const noexcept {
        return reinterpret_cast<const SharedPoolStatefulReader*>(reader_storage_[i]);
    }

    void register_writer_endpoint(const Guid& guid, const WriterConfig& cfg) noexcept;
    /**
     * @brief Announces one address as a metatraffic and a default unicast locator.
     *
     * @param[in] meta       The address at the metatraffic unicast port.
     * @param[in] user       The same address at the default unicast port.
     * @param[in] prefix_len Its on-link prefix length in bits; 0 when unknown.
     * @return `false`, with nothing changed, when either list or the
     *   participant's LocalSubnets is full or the address is unspecified.
     */
    bool add_address_locator_pair(const Locator& meta, const Locator& user,
                                  uint8_t prefix_len) noexcept;
    bool remove_address_locator_pair(const Locator& meta, const Locator& user) noexcept;
    void refresh_spdp_multicast_locator() noexcept;
    uint8_t append_metatraffic_multicast_locators(Locator* out, uint8_t cap) const noexcept;
    uint8_t append_metatraffic_unicast_locators(Locator* out, uint8_t cap) const noexcept;
    uint8_t append_default_unicast_locators(Locator* out, uint8_t cap) const noexcept;

    // ── Data members ─────────────────────────────────────────
    uint8_t     domain_id_;
    uint8_t     participant_id_;
    ITransport& transport_;
    Guid        participant_guid_;
    Locator     spdp_multicast_locator_;

    SPDPdiscoveredParticipantData local_participant_data_;
    /// The announced addresses with their on-link prefixes; peer locator
    /// lists are ranked against them.
    LocalSubnets local_subnets_;

    EndpointMatchTable match_table_;
    /// Local writer every participant reads; its matches keep no peer active.
    Guid discovery_writer_guid_ = GUID_UNKNOWN;
    SpdpWriter         spdp_writer_;
    SpdpReader         spdp_reader_;
    SedpWriter         sedp_writer_;
    SedpReader         sedp_reader_;

    // Proxies of the user endpoints' matches, bound and released as remote
    // endpoints match and unmatch. Declared before the endpoint storage
    // that binds from them.
    ProxyPoolStorage<ReaderProxy, RTPS_READER_PROXY_POOL> reader_proxies_;
    ProxyPoolStorage<WriterProxy, RTPS_WRITER_PROXY_POOL> writer_proxies_;

    // Writer storage: opaque byte arrays, each holding one
    // SharedPoolStatefulWriter<D> of any depth; writers_ points at the
    // StatefulWriter inside each live one.
    alignas(alignof(SharedPoolStatefulWriter<8>)) Octet
        writer_storage_[RTPS_MAX_USER_WRITERS][RTPS_PARTICIPANT_MAX_WRITER_BYTES];

    StatefulWriter* writers_[RTPS_MAX_USER_WRITERS]      = {};
    bool            writer_alive_[RTPS_MAX_USER_WRITERS] = {};

    // Reader storage: one SharedPoolStatefulReader per slot, placement-new'd
    // by create_reader().
    alignas(SharedPoolStatefulReader) Octet
        reader_storage_[RTPS_MAX_USER_READERS][sizeof(SharedPoolStatefulReader)];
    bool  reader_alive_[RTPS_MAX_USER_READERS] = {};

    ParseOps parse_ops_;

    OnGraphChange on_graph_change_   = nullptr;
    void*         graph_change_user_ = nullptr;

    /// A user endpoint's proxy freed since the last tick that bound no
    /// waiting match, so one may bind now.
    bool rebind_due_ = false;
};

} // namespace rtps
