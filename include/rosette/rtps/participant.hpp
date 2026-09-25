/**
 * @file rtps/participant.hpp
 * @brief RTPSParticipant — §8.5.1 (OMG formal/2022-04-01).
 *
 * Changes from the previous version
 * -----------------------------------
 * create_writer() is now a function template:
 *
 *   template<uint8_t CacheDepth = 1, uint8_t MaxReaders = RTPS_MAX_MATCHED_READERS>
 *   StatefulWriter<CacheDepth, MaxReaders>* create_writer(const WriterConfig&);
 *
 * This lets each writer carry its own history depth at zero runtime cost:
 *
 *   auto* w1 = participant.create_writer<1>(cfg);  // KEEP_LAST(1)
 *   auto* w8 = participant.create_writer<8>(cfg);  // KEEP_LAST(8)
 *
 * Writers of different depths are stored in the participant's internal
 * table as IStatefulWriter* (type-erased virtual base), which provides
 * on_acknack(), on_nack_frag(), tick(), guid(), matched_reader_add/remove()
 * — the only operations the participant needs without knowing the depth.
 *
 * The concrete StatefulWriter<D,R>* is what the application holds and uses
 * for new_change() / add_change() / write_async() / cache().
 *
 * Storage model
 * -------------
 * Writer slots are opaque byte arrays (aligned storage) constructed via
 * placement-new.  Each slot is large enough for the largest possible
 * instantiation of StatefulWriter (bounded by
 * RTPS_PARTICIPANT_MAX_WRITER_BYTES per slot).  If a specific
 * StatefulWriter<D,R> does not fit, create_writer() returns nullptr and
 * a static_assert fires at compile time.
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
 *   2-9   platform::device_unique_id(): unique per device, and per process
 *         on a host
 *   10    instance counter: differs for each participant this process
 *         creates, so a participant recreated after shutdown is never
 *         mistaken for its predecessor by peers still holding its lease
 *   11    participant_id
 *
 * @param[in] participant_id Participant ID from the port mapping (§9.6.1.1).
 * @return The 12-octet prefix; participant and endpoint GUIDs share it.
 *
 * Verification:
 * - Correct means: distinct across devices, processes, and successive
 *   participants of one process; octet 11 equals `participant_id`.
 * - Covered by: TC-PART-8.5.1-003.
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

/// Maximum byte size of one writer slot in the participant's storage table.
/// Must be >= sizeof(StatefulWriter<D,R>) for all D and R you intend to use.
/// Default covers StatefulWriter<8, RTPS_MAX_MATCHED_READERS>.
#ifndef RTPS_PARTICIPANT_MAX_WRITER_BYTES
#  define RTPS_PARTICIPANT_MAX_WRITER_BYTES \
       (sizeof(StatefulWriter<8, RTPS_MAX_MATCHED_READERS>))
#endif

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
};

struct ReaderConfig {
    EntityId        entityId    = entity_id::UNKNOWN;
    TopicName       topicName   = {};
    TopicName       typeName    = {};
    ReliabilityKind reliability = ReliabilityKind::RELIABLE;
    DurabilityKind  durability  = DurabilityKind::VOLATILE;
    TopicKind       topicKind   = TopicKind::NO_KEY;
    uint32_t        hbResponseMs= RTPS_HEARTBEAT_RESPONSE_DELAY_MS;
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

    // ----------------------------------------------------------
    // create_writer<CacheDepth, MaxReaders>()
    //
    // Factory template: construct a StatefulWriter<CacheDepth, MaxReaders>
    // in the participant's pre-allocated writer storage and return a
    // typed pointer to it.
    //
    // §8.5.1, p. 150:
    //   "The Participant creates Writer endpoints and associates them
    //    with the DDS layer."
    //
    // Usage:
    //   auto* w = participant.create_writer<1>(cfg);   // KEEP_LAST(1)
    //   auto* w = participant.create_writer<8,4>(cfg); // KEEP_LAST(8), 4 readers
    //
    // @returns Typed pointer, or nullptr if the slot table is full or
    //          sizeof(StatefulWriter<D,R>) > RTPS_PARTICIPANT_MAX_WRITER_BYTES.
    // ----------------------------------------------------------
    template<uint8_t CacheDepth = 1,
             uint8_t MaxReaders = RTPS_MAX_MATCHED_READERS>
    StatefulWriter<CacheDepth, MaxReaders>* create_writer(
            const WriterConfig& cfg) noexcept
    {
        using W = StatefulWriter<CacheDepth, MaxReaders>;

        static_assert(sizeof(W) <= RTPS_PARTICIPANT_MAX_WRITER_BYTES,
            "StatefulWriter<D,R> exceeds RTPS_PARTICIPANT_MAX_WRITER_BYTES. "
            "Increase RTPS_PARTICIPANT_MAX_WRITER_BYTES in your config.");

        // Find a free slot.
        int slot = find_free_writer_slot();
        if (slot < 0) return nullptr;

        const Guid writer_guid{ participant_guid_.prefix,
                                assign_entity_id(cfg.entityId, true, cfg.topicKind) };

        // Placement-new into the pre-allocated, correctly aligned storage.
        W* w = new (writer_storage_[slot]) W(
            writer_guid, cfg.reliability, cfg.durability,
            transport_, cfg.nackDelayMs, cfg.hbPeriodMs);

        // Store the IStatefulWriter* for type-erased participant operations.
        writer_iface_[slot] = static_cast<IStatefulWriter*>(w);
        writer_alive_[slot] = true;

        // Register endpoint for SEDP announcements.
        register_writer_endpoint(writer_guid, cfg);

        return w;
    }

    // ----------------------------------------------------------
    // create_reader()
    //
    // Every user reader is a DefaultStatefulReader: it matches up to
    // RTPS_MAX_MATCHED_WRITERS writers and has no history-depth parameter.
    //
    // @returns Typed pointer, or nullptr if the slot table is full.
    // ----------------------------------------------------------
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
    bool delete_writer(IStatefulWriter* writer) noexcept;
    bool delete_reader(StatefulReader* reader) noexcept;

    // ----------------------------------------------------------
    // parse_received() — §8.3.4, p. 69.
    // ----------------------------------------------------------
    ParseResult parse_received(const Octet* buf, uint32_t len) noexcept;

    // ----------------------------------------------------------
    // tick() — drives SPDP, lease expiry, writer/reader maintenance.
    // ----------------------------------------------------------
    void tick(uint32_t now_ms) noexcept;

    // ----------------------------------------------------------
    // ms_until_next_timer()
    //
    // How long tick() can be deferred: the soonest of the SPDP
    // announcement, any peer lease expiry, and every builtin and user
    // writer's and reader's pending timer. A driver that sleeps between
    // ticks waits no longer than this, so no heartbeat, repair, or lease
    // expiry is late.
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
    bool add_ipv4_address(uint32_t ipv4_be) noexcept;
    bool add_ipv6_address(const uint8_t ipv6[16]) noexcept;
    bool remove_ipv4_address(uint32_t ipv4_be) noexcept;
    bool remove_ipv6_address(const uint8_t ipv6[16]) noexcept;
    void clear_addresses() noexcept;
    bool has_addresses() const noexcept;

    // ----------------------------------------------------------
    // Accessors
    // ----------------------------------------------------------
    const Guid&       participant_guid() const noexcept { return participant_guid_; }
    const GuidPrefix& guid_prefix()      const noexcept { return participant_guid_.prefix; }
    uint8_t           domain_id()        const noexcept { return domain_id_; }
    SpdpReader&       spdp_reader()            noexcept { return spdp_reader_; }
    EndpointMatchTable& match_table()          noexcept { return match_table_; }

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
    static void on_endpoint_match   (const MatchedEndpoint&,
                                     const MatchedEndpoint&,
                                     const TopicName&,
                                     void*) noexcept;
    static void on_endpoint_unmatch (const Guid&, const Guid&, void*) noexcept;

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
    IStatefulWriter* find_writer_iface(const EntityId& eid) noexcept;
    StatefulReader*  find_reader(const EntityId& eid) noexcept;

    /**
     * @brief Resolves where a matched endpoint is reached.
     *
     * An endpoint's announcement may omit its unicast locators; it is then
     * reached at its participant's defaultUnicastLocatorList from SPDP. This
     * participant's own endpoints are always resolved this way, to its own
     * default unicast locators. Locators an endpoint announced are used as
     * they are.
     *
     * @param[in]     remote_prefix GUID prefix of the endpoint's participant
     *   (this participant's own prefix for a local endpoint).
     * @param[in,out] locators      The announced list; replaced by the
     *   participant's defaults (owned by the SPDP peer table or this
     *   participant, valid until they next change) when `count` is 0.
     * @param[in,out] count         Entries in `locators`; stays 0 when the
     *   participant is unknown or announced no defaults either.
     *
     * Verification:
     * - Correct means: DATA to a matched remote reader and ACKNACKs to a
     *   matched remote writer that announced no locators go to its
     *   participant's default unicast locators.
     * - Covered by: TC-PART-8.5.1-007.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.5.3.2 (defaultUnicastLocatorList),
     *   §8.5.4.2 (DiscoveredWriterData / DiscoveredReaderData)
     */
    void resolve_endpoint_locators(const GuidPrefix& remote_prefix,
                                   const Locator*&   locators,
                                   uint8_t&          count) const noexcept;
    template <typename Deliver>
    void for_each_addressed_reader(const EntityId& writer_id, const EntityId& reader_id,
                                   Deliver&& deliver) noexcept;

    /// @brief The user reader in slot `i`; valid only while reader_alive_[i].
    DefaultStatefulReader* user_reader(int i) noexcept {
        return reinterpret_cast<DefaultStatefulReader*>(reader_storage_[i]);
    }
    const DefaultStatefulReader* user_reader(int i) const noexcept {
        return reinterpret_cast<const DefaultStatefulReader*>(reader_storage_[i]);
    }
    void register_writer_endpoint(const Guid& guid, const WriterConfig& cfg) noexcept;
    bool add_address_locator_pair(const Locator& meta, const Locator& user) noexcept;
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

    EndpointMatchTable match_table_;
    SpdpWriter         spdp_writer_;
    SpdpReader         spdp_reader_;
    SedpWriter         sedp_writer_;
    SedpReader         sedp_reader_;

    // Writer storage: opaque byte arrays, aligned for StatefulWriter.
    // IStatefulWriter* overlay allows virtual dispatch without knowing D.
    alignas(alignof(StatefulWriter<8, RTPS_MAX_MATCHED_READERS>))
    Octet writer_storage_[RTPS_MAX_USER_WRITERS]
                         [RTPS_PARTICIPANT_MAX_WRITER_BYTES];

    IStatefulWriter* writer_iface_[RTPS_MAX_USER_WRITERS] = {};
    bool             writer_alive_[RTPS_MAX_USER_WRITERS]  = {};

    // Reader storage: one DefaultStatefulReader per slot, placement-new'd
    // by create_reader().
    alignas(DefaultStatefulReader)
    Octet reader_storage_[RTPS_MAX_USER_READERS][sizeof(DefaultStatefulReader)];
    bool  reader_alive_[RTPS_MAX_USER_READERS] = {};

    ParseOps parse_ops_;
};

} // namespace rtps
