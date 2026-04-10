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
 * for new_change() / add_change() / write_reliable() / cache().
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
// Platform hook
// ============================================================
extern "C" GuidPrefix rtps_platform_make_guid_prefix(uint8_t participant_id) noexcept;
extern "C" uint32_t rtps_platform_get_ipv4_addr(void) noexcept;

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

        const Guid writer_guid{ participant_guid_.prefix, cfg.entityId };

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
    // (Non-template: StatefulReader depth is fixed by the number of
    //  matched writers, not by history depth.)
    //
    // @returns Typed pointer, or nullptr if the slot table is full.
    // ----------------------------------------------------------
    StatefulReader* create_reader(const ReaderConfig&               cfg,
                                  StatefulReader::OnChangeAvailable on_change_avail = nullptr,
                                  void*                             user_data       = nullptr) noexcept;

    // ----------------------------------------------------------
    // parse_received() — §8.3.4, p. 69.
    // ----------------------------------------------------------
    ParseResult parse_received(const Octet* buf, uint32_t len) noexcept;

    // ----------------------------------------------------------
    // tick() — drives SPDP, lease expiry, writer/reader maintenance.
    // ----------------------------------------------------------
    void tick(uint32_t now_ms) noexcept;

    // ----------------------------------------------------------
    // announce() — force immediate SPDP send.
    // ----------------------------------------------------------
    void announce() noexcept;

    // ----------------------------------------------------------
    // Accessors
    // ----------------------------------------------------------
    const Guid&       participant_guid() const noexcept { return participant_guid_; }
    const GuidPrefix& guid_prefix()      const noexcept { return participant_guid_.prefix; }
    uint8_t           domain_id()        const noexcept { return domain_id_; }
    SpdpReader&       spdp_reader()            noexcept { return spdp_reader_; }
    EndpointMatchTable& match_table()          noexcept { return match_table_; }

private:

    // ── Discovery callbacks ───────────────────────────────────
    static void on_spdp_new_peer    (const PeerEntry&, void*) noexcept;
    static void on_spdp_peer_removed(const PeerEntry&, void*) noexcept;
    static void on_endpoint_match   (const DiscoveredWriterData&,
                                     const DiscoveredReaderData&,
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
    void register_writer_endpoint(const Guid& guid, const WriterConfig& cfg) noexcept;
    Locator metatraffic_unicast_locator() const noexcept;
    Locator default_unicast_locator()     const noexcept;

    // ── Data members ─────────────────────────────────────────
    uint8_t     domain_id_;
    uint8_t     participant_id_;
    ITransport& transport_;
    Guid        participant_guid_;

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

    // Reader storage (StatefulReader is not templated on depth).
    alignas(StatefulReader)
    Octet reader_storage_[RTPS_MAX_USER_READERS][sizeof(StatefulReader)];
    bool  reader_alive_[RTPS_MAX_USER_READERS] = {};

    ParseOps parse_ops_;
};

} // namespace rtps
