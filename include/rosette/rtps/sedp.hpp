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
 * Send policy (project constraint)
 * ----------------------------------
 * SEDP is sent ONCE per newly discovered peer — not re-sent on every
 * SPDP heartbeat.  This is tracked via SpdpReader::sedp_announced.
 * After sending, SpdpReader::mark_sedp_announced() is called.
 *
 * Reliability (§8.5.4.1, p. 165)
 * --------------------------------
 * The SEDP builtin endpoints use RELIABLE / TRANSIENT_LOCAL QoS.
 * On this embedded target we use a simplified approach: SEDP data is
 * sent as BEST_EFFORT (no retransmit machinery for builtin endpoints)
 * and re-sent if/when the remote participant sends a new SPDP announce.
 * This is permitted by §8.5.4.1 note that says implementations may
 * choose simpler reliability for builtins.
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
#include "message_parser.hpp"   // ParseContext, ParsedHeartbeat, ParsedGap
#include "sn_receive_window.hpp"
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

    using BuiltinWriter = StatefulWriter<kSedpHistoryDepth, RTPS_MAX_PEERS>;

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

    // Route ACKNACK/NACK_FRAG for builtin SEDP writers.
    IStatefulWriter* find_writer_iface(const EntityId& writer_id) noexcept;

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

    DiscoveredWriterData local_writers_[RTPS_MAX_LOCAL_ENDPOINTS] = {};
    DiscoveredReaderData local_readers_[RTPS_MAX_LOCAL_ENDPOINTS] = {};
    /// SN of each slot's live announcement in its builtin writer's history.
    SequenceNumber       local_writer_sn_[RTPS_MAX_LOCAL_ENDPOINTS] = {};
    SequenceNumber       local_reader_sn_[RTPS_MAX_LOCAL_ENDPOINTS] = {};

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
// Receives SEDP DATA submessages from remote participants and feeds
// the DiscoveredWriterData / DiscoveredReaderData into the
// EndpointMatchTable.
//
// Also handles incoming HEARTBEATs from remote SEDP writers and
// responds with ACKNACK so those writers deliver their endpoint data.
// — §8.5.4.1, p. 165; §8.3.7.5, p. 91.
// ============================================================

class SedpReader {
public:

    // ----------------------------------------------------------
    // Constructor
    //
    // @param local_prefix  GUID prefix of the local participant
    //                      (used as the sender prefix in ACKNACK messages).
    // @param transport     Transport for sending ACKNACK replies.
    // @param match_table   Endpoint match table to update on DATA.
    // ----------------------------------------------------------
    explicit SedpReader(const GuidPrefix&   local_prefix,
                        ITransport&         transport,
                        EndpointMatchTable& match_table) noexcept
        : local_prefix_(local_prefix)
        , transport_(transport)
        , match_table_(match_table)
    {}

    // ----------------------------------------------------------
    // on_publication_data()
    //
    // Process a SEDP publications DATA submessage payload.
    //
    // §8.5.4, p. 163:
    //   "The SEDPbuiltinPublicationsReader receives DiscoveredWriterData
    //    from remote participants and updates the local discovery state."
    //
    // @param sender_prefix  GUID prefix of the announcing participant.
    // @param payload        PL_CDR_LE payload bytes.
    // @param payload_len    Byte count.
    // @param writer_sn      Sequence number from the DATA submessage header.
    // ----------------------------------------------------------
    void on_publication_data(const GuidPrefix& sender_prefix,
                             const Octet*      payload,
                             uint32_t          payload_len,
                             SequenceNumber    writer_sn) noexcept;

    // ----------------------------------------------------------
    // on_subscription_data()
    //
    // Process a SEDP subscriptions DATA submessage payload.
    // ----------------------------------------------------------
    void on_subscription_data(const GuidPrefix& sender_prefix,
                              const Octet*      payload,
                              uint32_t          payload_len,
                              SequenceNumber    writer_sn) noexcept;

    // ----------------------------------------------------------
    // on_publication_disposed() / on_subscription_disposed()
    //
    // A remote SEDP writer withdrew an endpoint (DATA whose inline QoS
    // StatusInfo has DISPOSED or UNREGISTERED set). Removes it from the
    // match table, which unmatches it from every local endpoint.
    //
    // §8.5.4.1, §9.6.3.3. Verification: TC-SEDP-8.5.4-009.
    //
    // @param sender_prefix  GUID prefix of the announcing participant.
    // @param endpoint_guid  Withdrawn endpoint, from PID_KEY_HASH.
    // @param writer_sn      Sequence number of the disposal DATA.
    // ----------------------------------------------------------
    void on_publication_disposed(const GuidPrefix& sender_prefix,
                                 const Guid&       endpoint_guid,
                                 SequenceNumber    writer_sn) noexcept;
    void on_subscription_disposed(const GuidPrefix& sender_prefix,
                                  const Guid&       endpoint_guid,
                                  SequenceNumber    writer_sn) noexcept;

    /**
     * @brief Answers a HEARTBEAT from a remote SEDP writer with an ACKNACK.
     *
     * The builtin readers are reliable (§8.5.4.1), so the ACKNACK names
     * every endpoint announcement still missing from that writer, from the
     * per-writer SnReceiveWindow: out-of-order arrivals and lost bursts leave
     * holes, which are requested until DATA or GAP resolves them. firstSN
     * resolves everything the writer no longer holds. On a link whose
     * repairs must fit one datagram (repairs_must_fit_datagram()) only the
     * lowest missing announcement is requested, and each arriving repair
     * requests the next, so a peer that packs repairs (Fast DDS) still
     * completes discovery.
     *
     * @param[in] ctx            Parse context; senderGuidPrefix names the peer.
     * @param[in] msg            HEARTBEAT from SEDP_BUILTIN_PUBLICATIONS_ANNOUNCER
     *   or SEDP_BUILTIN_SUBSCRIPTIONS_ANNOUNCER; others are ignored.
     * @param[in] writer_locator The peer's metatraffic unicast locator; the
     *   ACKNACK and later repair requests go there.
     *
     * Verification:
     * - Correct means: every hole below lastSN is requested until resolved;
     *   a pure ACK (F=1) is sent only when nothing up to lastSN is missing.
     * - Covered by: TC-SEDP-8.5.4-010, TC-SEDP-8.5.4-011.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.3.7.5 (HEARTBEAT), §8.4.12.1, §8.5.4.1
     */
    void on_heartbeat(const ParseContext&    ctx,
                      const ParsedHeartbeat& msg,
                      const Locator&         writer_locator) noexcept;

    /**
     * @brief Resolves announcements a remote SEDP writer declared irrelevant.
     *
     * A writer sends GAP for sequence numbers it will never deliver (for
     * example an endpoint announcement superseded by its disposal). Without
     * this the reader would request them forever.
     *
     * @param[in] ctx Parse context; senderGuidPrefix names the peer.
     * @param[in] msg GAP from an SEDP announcer; others are ignored.
     *
     * Verification:
     * - Correct means: [gapStart, gapList.base) and every gapList bit are
     *   resolved; on a narrow link the next hole is requested at once.
     * - Covered by: TC-SEDP-8.5.4-011.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.3.7.4 (GAP), §8.4.12.1
     */
    void on_gap(const ParseContext& ctx, const ParsedGap& msg) noexcept;

    /// @brief Forgets the reliability state kept for a peer that left.
    void remove_peer(const GuidPrefix& prefix) noexcept;

private:

    /// Reliability state for one remote SEDP announcer (§8.4.10 WriterProxy).
    struct SedpWriterState {
        SnReceiveWindow window;
        SequenceNumber  last_sn;          ///< Highest lastSN heard in a HEARTBEAT.
        SequenceNumber  pending_repair;   ///< Sole change requested by a narrow ACKNACK.
        uint32_t        acknack_count = 0;
    };

    /// Per-peer state for the two SEDP builtin readers (§8.5.4.3, Table 8.80).
    struct SedpBuiltinAckState {
        GuidPrefix      prefix;
        SedpWriterState pub;              ///< SEDP_BUILTIN_PUBLICATIONS_ANNOUNCER.
        SedpWriterState sub;              ///< SEDP_BUILTIN_SUBSCRIPTIONS_ANNOUNCER.
        Locator         writer_locator = LOCATOR_INVALID;
        bool            alive = false;
    };

    GuidPrefix          local_prefix_;
    ITransport&         transport_;
    EndpointMatchTable& match_table_;
    MessageBuilder      builder_;
    SedpBuiltinAckState ack_states_[RTPS_MAX_PEERS] = {};

    SedpBuiltinAckState* find_or_alloc_ack_state(const GuidPrefix& prefix) noexcept {
        SedpBuiltinAckState* free_slot = nullptr;
        for (uint8_t i = 0; i < RTPS_MAX_PEERS; ++i) {
            if (ack_states_[i].alive && ack_states_[i].prefix == prefix)
                return &ack_states_[i];
            if (!ack_states_[i].alive && !free_slot)
                free_slot = &ack_states_[i];
        }
        if (free_slot) {
            *free_slot = SedpBuiltinAckState{};
            free_slot->prefix = prefix;
            free_slot->alive  = true;
        }
        return free_slot;
    }

    /**
     * @brief Records an announcement's sequence number and reports whether
     * it is new; requests the next repair when it was the one outstanding.
     *
     * @param[in] sender_prefix Announcing participant.
     * @param[in] is_pub        Publications (true) or subscriptions announcer.
     * @param[in] writer_sn     Sequence number of the DATA.
     * @return `true` to process the announcement, `false` for a duplicate.
     */
    bool accept_announcement(const GuidPrefix& sender_prefix, bool is_pub,
                             const SequenceNumber& writer_sn) noexcept;

    /// Sends the ACKNACK describing `writer`'s window to the peer in `state`.
    void send_acknack(SedpBuiltinAckState& state, bool is_pub) noexcept;

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
};

} // namespace rtps
