// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

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

namespace rosette::rtps {

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
// Announcement storage. A SedpWriter constructed with a LocalEndpointSource
// rebuilds each live announcement every time it is sent: the history keeps
// only the change (its SN and key hash), and the PL_CDR bytes are serialized
// into a pool block that is released when the send returns. Without a
// source, each announcement is serialized once and its block stays in the
// history for as long as the endpoint exists. Rebuilding trades one
// serialization per send (first send, NACK repair, replay to a new peer)
// for a 512-octet block per local endpoint per builtin writer; the
// participant picks one with RTPS_SEDP_REBUILD_ANNOUNCEMENTS. Disposals
// are a few dozen octets and are always stored.
//
// A rebuilt announcement carries the participant's unicast locators as
// they are when it is sent. A reader that already holds the change discards
// a duplicate by its SN, so no reader ever sees two versions of one change;
// a peer that joins after an address change receives the current locators.
//
// §8.5.4.3, p. 167: the publications and subscriptions writers use
// the SEDPbuiltin EntityIds defined in entity_id::.
// ============================================================

#ifndef RTPS_MAX_LOCAL_ENDPOINTS
#  define RTPS_MAX_LOCAL_ENDPOINTS 8
#endif

static_assert(RTPS_MAX_LOCAL_ENDPOINTS >= 1 && RTPS_MAX_LOCAL_ENDPOINTS < 255,
              "RTPS_MAX_LOCAL_ENDPOINTS must fit the uint8_t history depth");

/**
 * @brief Where a SedpWriter that rebuilds its announcements reads a local endpoint's data.
 *
 * The participant already keeps every local endpoint's topic, type, and QoS
 * (EndpointMatchTable) and its unicast locators, so a SedpWriter given this
 * source stores no copy of an announcement and rebuilds it for each send.
 * Each function fills the complete DiscoveredWriterData or
 * DiscoveredReaderData that register_writer() or register_reader() was
 * given, with the locators as they are now, and returns false for a GUID
 * the participant no longer has.
 */
struct LocalEndpointSource {
    using DescribeWriter = bool (*)(void* user, const Guid& guid,
                                    DiscoveredWriterData& out) noexcept;
    using DescribeReader = bool (*)(void* user, const Guid& guid,
                                    DiscoveredReaderData& out) noexcept;

    DescribeWriter writer = nullptr;  ///< Describes a local writer.
    DescribeReader reader = nullptr;  ///< Describes a local reader.
    void* user = nullptr;             ///< Passed back to both; outlives the SedpWriter.

    /// @brief True when both functions are set, so announcements are rebuilt.
    bool valid() const noexcept { return writer != nullptr && reader != nullptr; }
};

class SedpWriter {
public:

    /// History depth of each builtin writer: every live endpoint plus one
    /// disposal in flight.
    static constexpr uint8_t kSedpHistoryDepth =
        static_cast<uint8_t>(RTPS_MAX_LOCAL_ENDPOINTS + 1);

    using BuiltinWriter = StatefulWriterStorage<kSedpHistoryDepth, RTPS_MAX_PEERS>;

    /**
     * @brief Creates the two builtin SEDP writers with empty histories.
     *
     * @param[in] local_prefix This participant's GUID prefix.
     * @param[in] transport    Transport both writers send on; outlives them.
     * @param[in] endpoints    When valid, announcements are rebuilt from it
     *   for every send instead of stored (see "Announcement storage" above);
     *   the SedpWriter must then stay at its address, since both builtin
     *   writers call back into it.
     */
    explicit SedpWriter(const GuidPrefix& local_prefix, ITransport& transport,
                        const LocalEndpointSource& endpoints = LocalEndpointSource{}) noexcept
        : publications_writer_(Guid{local_prefix, entity_id::SEDP_BUILTIN_PUBLICATIONS_ANNOUNCER},
                               ReliabilityKind::RELIABLE, DurabilityKind::TRANSIENT_LOCAL,
                               transport),
          subscriptions_writer_(Guid{local_prefix, entity_id::SEDP_BUILTIN_SUBSCRIPTIONS_ANNOUNCER},
                                ReliabilityKind::RELIABLE, DurabilityKind::TRANSIENT_LOCAL,
                                transport),
          endpoints_(endpoints) {
        if (endpoints_.valid()) {
            publications_writer_.set_payload_builder(&SedpWriter::build_announcement, this,
                                                     &SedpWriter::announcement_size);
            subscriptions_writer_.set_payload_builder(&SedpWriter::build_announcement, this,
                                                      &SedpWriter::announcement_size);
        }
    }

    SedpWriter(const SedpWriter&) = delete;
    SedpWriter& operator=(const SedpWriter&) = delete;

    /// @brief True when announcements are rebuilt for each send rather than stored.
    bool rebuilds_announcements() const noexcept { return endpoints_.valid(); }

    /**
     * @brief Records a local writer and announces it to every matched peer.
     *
     * The participant calls this once per endpoint it creates. The
     * announcement goes to every matched peer now and, through
     * TRANSIENT_LOCAL replay, to every peer discovered later, until
     * unregister_writer() withdraws it.
     *
     * The announcement becomes one change in the publications writer's
     * history. A rebuilding SedpWriter stores only that change and first
     * confirms the endpoint source already knows the GUID, since every send
     * rebuilds from it; otherwise the PL_CDR bytes are serialized now and
     * stored with the change.
     *
     * @param[in] wd The writer's discovery data; copied, not retained.
     * @return `false` when RTPS_MAX_LOCAL_ENDPOINTS writers are already
     *   registered, the payload cannot be allocated, or (rebuilding) the
     *   endpoint source does not know the writer; `true` once the
     *   announcement is in the history.
     *
     * Verification:
     * - Correct means: every matched peer, whenever it matched, receives
     *   the endpoint's DiscoveredWriterData under one SN until it is
     *   withdrawn.
     * - Covered by: TC-SEDP-8.5.4-001/004 (announced on first match),
     *   TC-SEDP-8.5.4-007 (endpoint created after the peer was matched),
     *   TC-SEDP-8.5.4-015 (rebuilt announcements).
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.5.4 ("The SEDPbuiltinPublicationsWriter sends
     *   DiscoveredWriterData for all local DataWriters"), §8.7.2.2
     *   (TRANSIENT_LOCAL)
     */
    bool register_writer(const DiscoveredWriterData& wd) noexcept;

    /// @brief As register_writer(), for a local reader on the subscriptions writer.
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
    //                       The proxies refer to it, so it must stay valid
    //                       until on_peer_removed() for this peer.
    // ----------------------------------------------------------
    void announce_to_peer(const GuidPrefix& peer_prefix,
                          const Locator&    dest) noexcept;

    // Match the peer's builtin SEDP readers from its SPDP data (at its first,
    // most reachable, metatraffic unicast locator), which replays the
    // history. A peer with no metatraffic unicast locator is not matched
    // (§8.5.4.1). `peer` is the SPDP peer table's record: the proxies refer
    // to its locators and follow them when an SPDP refresh changes them,
    // until on_peer_removed().
    //
    // Verification: TC-SEDP-8.5.4-018, TC-PART-8.5.3-001.
    void on_peer_discovered(const DiscoveredParticipant& peer) noexcept;

    // Remove remote builtin SEDP readers when a peer expires.
    void on_peer_removed(const GuidPrefix& peer_prefix) noexcept;

    /// @brief True when the peer's builtin SEDP readers have acknowledged
    /// every announcement in both histories (StatefulWriter::acknowledged_by_participant()).
    bool acknowledged_by(const GuidPrefix& peer_prefix) const noexcept {
        return publications_writer_.acknowledged_by_participant(peer_prefix)
               && subscriptions_writer_.acknowledged_by_participant(peer_prefix);
    }

    /// @brief The builtin SEDP writer with entity ID `writer_id`, or null;
    /// the participant routes ACKNACK and NACK_FRAG through it.
    StatefulWriter* find_writer(const EntityId& writer_id) noexcept;

    // Drive builtin writer periodic maintenance, and the announcement replay
    // to newly discovered peers (StatefulWriter::drain_replays()).
    void tick(uint32_t now_ms) noexcept;

    /// @brief True while either builtin writer is still replaying its
    ///   announcements to a newly discovered peer from tick().
    bool replay_pending() const noexcept {
        return publications_writer_.replay_pending() || subscriptions_writer_.replay_pending();
    }

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

    /// Source of rebuilt announcements; invalid when they are stored.
    LocalEndpointSource endpoints_{};

    /**
     * @brief Rebuilds the announcement a builtin writer's history change stands for.
     *
     * The StatefulWriter::PayloadBuilder of both builtin writers. The
     * change's key hash is the endpoint GUID and its writer GUID names the
     * builtin writer, which says whether the endpoint is a writer or a
     * reader; the source describes it and serialise_writer() or
     * serialise_reader() encodes it with in-place framing headroom.
     *
     * @param[in] user   This SedpWriter.
     * @param[in] change A live announcement from either builtin writer.
     * @return The serializedPayload in a fresh pool block; invalid when the
     *   source no longer has the endpoint or the pool is exhausted.
     *
     * Verification:
     * - Correct means: the bytes equal what register_writer() or
     *   register_reader() would have stored for the same endpoint data.
     * - Covered by: TC-SEDP-8.5.4-015, TC-SEDP-8.5.4-016.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §9.6.3.2 (PL_CDR endpoint data), §9.4.5.3 (DATA)
     */
    static BufRef build_announcement(void* user, const CacheChange& change) noexcept;

    /**
     * @brief Sizes the announcement build_announcement() would build for a change.
     *
     * The StatefulWriter::PayloadSizer of both builtin writers, which lets a
     * replay pack several announcements per datagram without building one
     * that does not fit. It looks the endpoint up as build_announcement()
     * does and adds up the PL_CDR parameters serialise_writer() or
     * serialise_reader() write, each padded to 4 octets.
     *
     * @param[in] user   This SedpWriter.
     * @param[in] change A live announcement from either builtin writer.
     * @return The serializedPayload size in octets; 0 when the source no
     *   longer has the endpoint.
     *
     * Verification:
     * - Correct means: the size equals the size of the payload
     *   build_announcement() builds for the same change, and is never less.
     * - Covered by: TC-SEDP-8.5.4-019.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §9.6.3.2 (PL_CDR endpoint data), §9.4.2.11
     *   (ParameterList, 4-octet alignment)
     */
    static uint32_t announcement_size(void* user, const CacheChange& change) noexcept;

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
     * unicast locators; an announcer it does not list is unmatched. A peer
     * that announced no metatraffic unicast locator is left unmatched, since
     * ACKNACKs would have nowhere to go. Matching an already matched
     * announcer changes nothing, so its reliability state survives SPDP
     * refreshes.
     *
     * @param[in] peer What the SPDP peer table keeps of the participant: the
     *   table's own record, not a copy. The proxies refer to its locators and
     *   follow them when an SPDP refresh changes them in place, so it must
     *   stay valid until on_peer_removed() for this peer.
     *
     * Verification:
     * - Correct means: after the call, DATA and HEARTBEAT from each listed
     *   announcer are processed and ACKNACKs go to the peer's locator.
     * - Covered by: TC-SEDP-8.5.4-006, TC-SEDP-8.5.4-011, TC-SEDP-8.5.4-018,
     *   TC-PART-8.5.3-001.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.5.4.1, §8.5.3.2 (availableBuiltinEndpoints)
     */
    void on_peer_discovered(const DiscoveredParticipant& peer) noexcept;

    /// @brief Unmatches a departed participant's SEDP announcers.
    void on_peer_removed(const GuidPrefix& peer_prefix) noexcept;

    /**
     * @brief Reports whether every announcement a peer has sent was received.
     *
     * The participant parks only a peer whose announcements this reader
     * holds completely (RTPSParticipant::peer_is_settled()), and keeps the
     * highest sequence number of each announcer so a parked peer that
     * announces something newer can be told apart from one repeating
     * itself.
     *
     * @param[in]  peer_prefix   GUID prefix of the peer.
     * @param[out] publications  Highest sequence number below which every
     *   change of the peer's publications announcer is resolved; 0 when it
     *   is not matched.
     * @param[out] subscriptions As `publications`, for the subscriptions
     *   announcer.
     * @return `false` while either matched announcer has a change this
     *   reader knows of and has not received.
     *
     * Verification:
     * - Correct means: after a HEARTBEAT announces changes the reader has
     *   not received, the answer is false until they arrive or are GAPped.
     * - Covered by: TC-PART-8.5.3-012.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.4.10.4 (available_changes_max, missing_changes)
     */
    bool caught_up_with(const GuidPrefix& peer_prefix, SequenceNumber& publications,
                        SequenceNumber& subscriptions) noexcept;

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
     * NOT_ALIVE: removes the writer named by the change's key. Either way the
     * endpoint must belong to the announcing participant: its GUID prefix
     * equals that of the announcer, so a participant can neither announce
     * nor withdraw another's endpoints, and every endpoint recorded from a
     * participant leaves with it (EndpointMatchTable::remove_participant()).
     *
     * @param[in] change Change delivered by publications_reader_.
     *
     * Verification:
     * - Correct means: a valid announcement of the announcer's own endpoint
     *   matches it; a disposal from its owner unmatches it; anything else,
     *   including an endpoint under another participant's prefix, is ignored.
     * - Covered by: TC-SEDP-8.5.4-002, TC-SEDP-8.5.4-009,
     *   TC-SEDP-8.5.4-020, TC-SEDP-9.6.3.2-*.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.2.4.3 (GUID prefix shared by a participant's
     *   entities), §8.5.4.2 (DiscoveredWriterData), §9.6.3.2,
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
    bool withdrawn_endpoint(const CacheChange& change, Guid& endpoint) const noexcept;

    /// @brief As read_plcdr_fields(), into a DiscoveredWriterData.
    bool deserialise_writer(const Octet* payload, uint32_t len,
                            DiscoveredWriterData& out) const noexcept;

    /// @brief As read_plcdr_fields(), into a DiscoveredReaderData.
    bool deserialise_reader(const Octet* payload, uint32_t len,
                            DiscoveredReaderData& out) const noexcept;

    /**
     * @brief Decodes the fields publication and subscription data share.
     *
     * Reads the parameter list of §9.6.2 up to PID_SENTINEL, skipping unknown
     * PIDs. The unicast locator list keeps the
     * RTPS_MAX_LOCATORS_PER_PARTICIPANT most reachable locators of the whole
     * announcement, ranked as it is read against the match table's addresses
     * (LocalSubnets::offer()), so EndpointMatchTable keeps the reachable one
     * however many the endpoint announced before it.
     *
     * @param[in]  payload           Serialized payload from its
     *   representation header; `len` octets long.
     * @param[in]  len               Payload length in octets.
     * @param[out] topic_out         PID_TOPIC_NAME, when present.
     * @param[out] type_out          PID_TYPE_NAME, when present.
     * @param[out] endpoint_guid_out PID_ENDPOINT_GUID, when present.
     * @param[out] reliability_out   PID_RELIABILITY kind, when present.
     * @param[out] durability_out    PID_DURABILITY kind, when present.
     * @param[out] unicast_locs      Ranked PID_UNICAST_LOCATOR list; at least
     *   RTPS_MAX_LOCATORS_PER_PARTICIPANT long.
     * @param[out] unicast_count     Entries written to `unicast_locs`.
     * @return `true` when PID_ENDPOINT_GUID was decoded.
     *
     * Verification:
     * - Correct means: unknown PIDs are skipped, a payload without an
     *   endpoint GUID is rejected, and the most reachable announced locator
     *   is kept whatever its position.
     * - Covered by: TC-SEDP-9.6.3.2-001, TC-SEDP-9.6.3.2-002,
     *   TC-SEDP-9.6.3.2-003.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.5.4.2, §9.6.2, §9.6.3.2 (Table 9.15)
     */
    bool read_plcdr_fields(const Octet* payload, uint32_t len, TopicName& topic_out,
                           TopicName& type_out, Guid& endpoint_guid_out,
                           ReliabilityKind& reliability_out, DurabilityKind& durability_out,
                           Locator* unicast_locs, uint8_t& unicast_count) const noexcept;

    EndpointMatchTable& match_table_;
    BuiltinReader       publications_reader_;
    BuiltinReader       subscriptions_reader_;
};

}  // namespace rosette::rtps
