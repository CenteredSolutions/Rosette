/**
 * @file rtps/sedp.hpp
 * @brief Simple Endpoint Discovery Protocol — §8.5.4 (OMG formal/2022-04-01).
 *
 * SEDP runs on top of SPDP: once a remote participant is discovered via
 * SPDP, SEDP exchanges endpoint (publication/subscription) metadata with
 * that participant so that StatefulWriter and StatefulReader proxies can
 * be wired together.
 *
 * Builtin SEDP endpoints (§8.5.4.3, Table 8.80, p. 167)
 * -------------------------------------------------------
 *   SEDPbuiltinPublicationsWriter   EntityId 0x000003C2  — announces local writers
 *   SEDPbuiltinPublicationsReader   EntityId 0x000003C7  — receives remote writers
 *   SEDPbuiltinSubscriptionsWriter  EntityId 0x000004C2  — announces local readers
 *   SEDPbuiltinSubscriptionsReader  EntityId 0x000004C7  — receives remote readers
 *
 * Encoding (§9.6.3.2, p. 263)
 * ----------------------------
 * Each endpoint announcement is a PL_CDR_LE parameter list.
 * Mandatory PIDs for publications: PID_ENDPOINT_GUID, PID_TOPIC_NAME,
 * PID_TYPE_NAME, PID_RELIABILITY, PID_DURABILITY.
 * Optional: PID_UNICAST_LOCATOR (if different from participant default).
 *
 * Reliability (§8.5.4.1, p. 165)
 * --------------------------------
 * The SEDP builtin endpoints are RELIABLE / TRANSIENT_LOCAL, and here they
 * are the ordinary protocol engines: SedpWriter owns two StatefulWriters and
 * SedpReader two StatefulReaders. When SPDP discovers a participant, each
 * side matches the peer's builtin counterparts from its
 * availableBuiltinEndpoints. Each local endpoint is written once, when it is
 * created; TRANSIENT_LOCAL replay delivers it to peers discovered later, and
 * the readers request lost announcements until DATA or GAP resolves them.
 * SEDP itself only encodes and decodes the PL_CDR endpoint data.
 *
 * Spec references (all OMG formal/2022-04-01)
 * -------------------------------------------
 *   SEDP overview              — §8.5.4,   p. 163
 *   SEDP QoS                   — §8.5.4.1, p. 165
 *   DiscoveredWriterData       — §8.5.4.2, Table 8.78, p. 166
 *   DiscoveredReaderData       — §8.5.4.2, Table 8.79, p. 166
 *   Builtin endpoint IDs       — §8.5.4.3, Table 8.80, p. 167
 *   PL_CDR PIDs for endpoints  — §9.6.3.2, Table 9.15, p. 263
 */

#pragma once

#include "types.hpp"
#include "endpoint_match_table.hpp"
#include "message_builder.hpp"
#include "stateful_reader.hpp"
#include "spdp.hpp"   // pid namespace, plcdr_write_param
#include "stateful_writer.hpp"

namespace rtps {

struct ITransport;

// ============================================================
// PL_CDR serialisation helpers for endpoint data
// §9.6.3.2, Table 9.15, p. 263
//
// String PID values (topicName, typeName) are encoded as a
// 4-byte length prefix followed by the string bytes (including
// the null terminator), padded to a 4-byte boundary.
// — §9.4.2.11, p. 202.
// ============================================================

/// Write a string PID: { PID, padded_len, uint32 str_len, str_bytes }.
inline bool plcdr_write_string(Octet*      buf,
                               uint32_t    cap,
                               uint32_t&   off,
                               uint16_t    pid_val,
                               const char* str) noexcept
{
    uint32_t str_len = 0;
    while (str && str[str_len]) ++str_len;
    ++str_len;   // Include null terminator.

    // Total value = uint32 length prefix (4) + string bytes (str_len).
    const uint32_t val_len    = 4u + str_len;
    const uint32_t val_padded = (val_len + 3u) & ~3u;
    const uint32_t total      = 4u + val_padded;   // PID + len + value.

    if (off + total > cap) return false;

    memcpy(buf + off, &pid_val,   2u); off += 2u;
    const uint16_t padded16 = static_cast<uint16_t>(val_padded);
    memcpy(buf + off, &padded16,  2u); off += 2u;
    memcpy(buf + off, &str_len,   4u); off += 4u;   // Length prefix.
    if (str_len > 1u) memcpy(buf + off, str, str_len - 1u);
    buf[off + str_len - 1u] = '\0';
    // Zero-pad to padded boundary.
    if (val_padded > val_len) {
        memset(buf + off + str_len, 0, val_padded - val_len);
    }
    off += val_padded - 4u;   // Advance past string part (already advanced past uint32 len).
    return true;
}

// ============================================================
// SedpWriter
//
// Owns the two builtin SEDP writers and keeps one TRANSIENT_LOCAL
// instance per local endpoint in each (§8.5.4.1, §8.5.4.3):
//
//  - register_*()   writes the endpoint's DiscoveredWriterData /
//                   DiscoveredReaderData once, when the endpoint is
//                   created. Every matched peer receives it, whether it
//                   was discovered before or after the endpoint existed.
//  - unregister_*() removes that change from the history and writes a
//                   disposal (PID_KEY_HASH = endpoint GUID, PID_STATUS_INFO
//                   = DISPOSED|UNREGISTERED) so peers unmatch the endpoint.
//  - on_peer_discovered() matches the peer's builtin SEDP readers; the
//                   reliable TRANSIENT_LOCAL writer replays the current
//                   history to them. Rediscovering a peer writes nothing.
//
// History holds at most RTPS_MAX_LOCAL_ENDPOINTS live announcements plus
// one slot for disposals. When the history is full, the oldest disposal
// is dropped first, so a live announcement is never evicted; a late
// joiner asking for a dropped SN receives a GAP.
//
// §8.5.4.3, p. 167: the publications and subscriptions writers use
// the SEDPbuiltin EntityIds defined in entity_id::.
// ============================================================

#ifndef RTPS_MAX_LOCAL_ENDPOINTS
#  define RTPS_MAX_LOCAL_ENDPOINTS 8
#endif

static_assert(RTPS_MAX_LOCAL_ENDPOINTS >= 1 && RTPS_MAX_LOCAL_ENDPOINTS < 255,
              "RTPS_MAX_LOCAL_ENDPOINTS must fit the uint8_t history depth");

class SedpWriter {
public:

    /// History depth of each builtin writer: every live endpoint plus one
    /// disposal in flight.
    static constexpr uint8_t kSedpHistoryDepth =
        static_cast<uint8_t>(RTPS_MAX_LOCAL_ENDPOINTS + 1);

    using BuiltinWriter = StatefulWriterStorage<kSedpHistoryDepth, RTPS_MAX_PEERS>;

    explicit SedpWriter(const GuidPrefix& local_prefix,
                        ITransport&       transport) noexcept
        : publications_writer_(
              Guid{local_prefix, entity_id::SEDP_BUILTIN_PUBLICATIONS_ANNOUNCER},
              ReliabilityKind::RELIABLE,
              DurabilityKind::TRANSIENT_LOCAL,
              transport)
        , subscriptions_writer_(
              Guid{local_prefix, entity_id::SEDP_BUILTIN_SUBSCRIPTIONS_ANNOUNCER},
              ReliabilityKind::RELIABLE,
              DurabilityKind::TRANSIENT_LOCAL,
              transport)
    {}

    // ----------------------------------------------------------
    // register_writer() / register_reader()
    //
    // Record a local endpoint and announce it to every matched peer,
    // now and (through TRANSIENT_LOCAL replay) to peers discovered later.
    //
    // §8.5.4, p. 163: "The SEDPbuiltinPublicationsWriter sends
    //  DiscoveredWriterData for all local DataWriters"; likewise for
    //  subscriptions.
    //
    // Verification: TC-SEDP-8.5.4-001/004 (announced on first match),
    // TC-SEDP-8.5.4-007 (endpoint created after the peer was matched).
    //
    // @returns false when RTPS_MAX_LOCAL_ENDPOINTS endpoints of this kind
    //          are already registered or the payload cannot be allocated.
    // ----------------------------------------------------------
    bool register_writer(const DiscoveredWriterData& wd) noexcept;
    bool register_reader(const DiscoveredReaderData& rd) noexcept;

    // ----------------------------------------------------------
    // unregister_writer() / unregister_reader()
    //
    // Withdraw a local endpoint: drop its announcement from the history
    // and publish a disposal keyed by its GUID so peers unmatch it.
    //
    // §8.5.4.1 / §9.6.3.3: an instance is withdrawn by a change with
    // StatusInfo DISPOSED|UNREGISTERED and the instance's KeyHash.
    //
    // Verification: TC-SEDP-8.5.4-008.
    //
    // @returns false when the GUID was never registered.
    // ----------------------------------------------------------
    bool unregister_writer(const Guid& writer_guid) noexcept;
    bool unregister_reader(const Guid& reader_guid) noexcept;

    // ----------------------------------------------------------
    // announce_to_peer()
    //
    // Match one peer's builtin SEDP readers at `dest` so the current
    // history replays to them. A peer that is already matched is left
    // unchanged, so repeated calls write nothing new.
    //
    // Verification: TC-SEDP-8.5.4-005.
    //
    // @param peer_prefix    Remote participant GUID prefix.
    // @param dest           Remote metatraffic locator for builtin readers.
    // ----------------------------------------------------------
    void announce_to_peer(const GuidPrefix& peer_prefix,
                          const Locator&    dest) noexcept;

    // Match the peer's builtin SEDP readers from its SPDP data (using all
    // of its metatraffic unicast locators), which replays the history.
    void on_peer_discovered(const SPDPdiscoveredParticipantData& peer) noexcept;

    // Remove remote builtin SEDP readers when a peer expires.
    void on_peer_removed(const GuidPrefix& peer_prefix) noexcept;

    /// @brief The builtin SEDP writer with entity ID `writer_id`, or null;
    /// the participant routes ACKNACK and NACK_FRAG through it.
    StatefulWriter* find_writer(const EntityId& writer_id) noexcept;

    // Drive builtin writer periodic maintenance.
    void tick(uint32_t now_ms) noexcept;

    // Soonest timed action of the two builtin writers (heartbeat or repair).
    uint32_t ms_until_next_timer(uint32_t now_ms) const noexcept {
        const uint32_t a = publications_writer_.ms_until_next_timer(now_ms);
        const uint32_t b = subscriptions_writer_.ms_until_next_timer(now_ms);
        return a < b ? a : b;
    }

private:

    BuiltinWriter publications_writer_;
    BuiltinWriter subscriptions_writer_;

    /// A registered local endpoint: the change announcing it lives in the
    /// builtin writer's history, so only its GUID and SN are kept here.
    struct Announced {
        Guid           guid = GUID_UNKNOWN;
        SequenceNumber sn   = SEQUENCENUMBER_UNKNOWN;
    };
    Announced local_writers_[RTPS_MAX_LOCAL_ENDPOINTS] = {};
    Announced local_readers_[RTPS_MAX_LOCAL_ENDPOINTS] = {};

    /// Serialise one DiscoveredWriterData as PL_CDR_LE.
    uint32_t serialise_writer(const DiscoveredWriterData& wd,
                              Octet* buf, uint32_t cap) const noexcept;

    /// Serialise one DiscoveredReaderData as PL_CDR_LE.
    uint32_t serialise_reader(const DiscoveredReaderData& rd,
                              Octet* buf, uint32_t cap) const noexcept;
};

// ============================================================
// SedpReader
//
// Owns the two builtin SEDP readers (§8.5.4.3, Table 8.80) and turns the
// changes they deliver into EndpointMatchTable updates:
//
//  - Each builtin reader is a RELIABLE StatefulReader (§8.5.4.1) with one
//    WriterProxy per remote participant. on_peer_discovered() matches the
//    peer's SEDP announcers from its SPDP data; the readers then request,
//    repair and acknowledge announcements exactly as any reliable reader.
//  - A delivered ALIVE change is a PL_CDR DiscoveredWriterData or
//    DiscoveredReaderData (§9.6.3.2) and is added to the match table; a
//    NOT_ALIVE change withdraws the endpoint its key names.
// ============================================================

class SedpReader {
public:

    /// A builtin SEDP reader: one matched announcer per remote participant.
    using BuiltinReader = BoundedStatefulReader<RTPS_MAX_PEERS>;

    /**
     * @brief Creates the builtin publications and subscriptions readers.
     *
     * @param[in] local_prefix GUID prefix of the local participant; the
     *   readers' GUIDs are this prefix with the SEDPbuiltin detector
     *   EntityIds.
     * @param[in] transport    Transport for ACKNACKs; must outlive this.
     * @param[in] match_table  Table updated with discovered endpoints; must
     *   outlive this.
     */
    SedpReader(const GuidPrefix&   local_prefix,
               ITransport&         transport,
               EndpointMatchTable& match_table) noexcept;

    SedpReader(const SedpReader&)            = delete;
    SedpReader& operator=(const SedpReader&) = delete;

    /**
     * @brief Matches a newly discovered participant's SEDP announcers.
     *
     * Each announcer the peer lists in availableBuiltinEndpoints is matched
     * with the corresponding builtin reader at the peer's metatraffic
     * unicast locators (multicast when it lists none); an announcer it does
     * not list is unmatched. Matching an already matched announcer changes
     * nothing, so its reliability state survives SPDP refreshes.
     *
     * @param[in] peer The peer's SPDPdiscoveredParticipantData.
     *
     * Verification:
     * - Correct means: after the call, DATA and HEARTBEAT from each listed
     *   announcer are processed and ACKNACKs go to the peer's locator.
     * - Covered by: TC-SEDP-8.5.4-006, TC-SEDP-8.5.4-011.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.5.4.1, §8.5.3.2 (availableBuiltinEndpoints)
     */
    void on_peer_discovered(const SPDPdiscoveredParticipantData& peer) noexcept;

    /// @brief Unmatches a departed participant's SEDP announcers.
    void on_peer_removed(const GuidPrefix& peer_prefix) noexcept;

    /**
     * @brief The builtin reader matched with the SEDP announcer `writer_id`.
     *
     * The participant routes every DATA, DATA_FRAG, HEARTBEAT,
     * HEARTBEAT_FRAG and GAP from a SEDP announcer here. The builtin
     * EntityIds are reserved (§9.3.1.2), so the writer alone identifies the
     * destination whatever readerId the submessage names.
     *
     * @param[in] writer_id EntityId of the sending writer.
     * @return The publications or subscriptions reader, or nullptr when
     *   `writer_id` is not a SEDP announcer.
     */
    StatefulReader* reader_for_writer(const EntityId& writer_id) noexcept;

    /// @brief Sends ACKNACKs whose heartbeatResponseDelay has elapsed.
    void tick(uint32_t now_ms) noexcept {
        publications_reader_.tick(now_ms);
        subscriptions_reader_.tick(now_ms);
    }

    /// @brief Soonest pending ACKNACK of the two builtin readers, or UINT32_MAX.
    uint32_t ms_until_next_timer(uint32_t now_ms) const noexcept {
        const uint32_t a = publications_reader_.ms_until_next_timer(now_ms);
        const uint32_t b = subscriptions_reader_.ms_until_next_timer(now_ms);
        return a < b ? a : b;
    }

private:

    /**
     * @brief Applies one publication announcement delivered by the reader.
     *
     * ALIVE: decodes the DiscoveredWriterData and adds it to the match table.
     * NOT_ALIVE: removes the writer named by the change's key, provided it
     * belongs to the announcing participant (a participant may only
     * withdraw its own endpoints).
     *
     * @param[in] change Change delivered by publications_reader_.
     *
     * Verification:
     * - Correct means: a valid announcement matches the endpoint; a
     *   disposal from its owner unmatches it; anything else is ignored.
     * - Covered by: TC-SEDP-8.5.4-002, TC-SEDP-8.5.4-009,
     *   TC-SEDP-9.6.3.2-*.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.5.4.2 (DiscoveredWriterData), §9.6.3.2,
     *   §9.6.3.3 (StatusInfo_t)
     */
    void on_publication_change(const CacheChange& change) noexcept;

    /// @brief As on_publication_change(), for DiscoveredReaderData.
    void on_subscription_change(const CacheChange& change) noexcept;

    static void publication_trampoline(const CacheChange& change, void* self) noexcept {
        static_cast<SedpReader*>(self)->on_publication_change(change);
    }
    static void subscription_trampoline(const CacheChange& change, void* self) noexcept {
        static_cast<SedpReader*>(self)->on_subscription_change(change);
    }

    /**
     * @brief The endpoint a NOT_ALIVE announcement withdraws.
     *
     * @param[in]  change   A NOT_ALIVE change from a builtin reader.
     * @param[out] endpoint The PID_KEY_HASH inline QoS (the endpoint GUID,
     *   §9.6.3.8), or else the PID_ENDPOINT_GUID of the serialized key.
     * @return `false` when the change names no endpoint.
     */
    static bool withdrawn_endpoint(const CacheChange& change, Guid& endpoint) noexcept;

    /// Deserialise a PL_CDR_LE payload into a DiscoveredWriterData.
    static bool deserialise_writer(const Octet*         payload,
                                   uint32_t             len,
                                   DiscoveredWriterData& out) noexcept;

    /// Deserialise a PL_CDR_LE payload into a DiscoveredReaderData.
    static bool deserialise_reader(const Octet*         payload,
                                   uint32_t             len,
                                   DiscoveredReaderData& out) noexcept;

    /// Shared PL_CDR field reader.
    static bool read_plcdr_fields(const Octet*  payload,
                                  uint32_t      len,
                                  TopicName&    topic_out,
                                  TopicName&    type_out,
                                  Guid&         endpoint_guid_out,
                                  ReliabilityKind& reliability_out,
                                  DurabilityKind&  durability_out,
                                  Locator*         unicast_locs,
                                  uint8_t&         unicast_count) noexcept;

    EndpointMatchTable& match_table_;
    BuiltinReader       publications_reader_;
    BuiltinReader       subscriptions_reader_;
};

} // namespace rtps
