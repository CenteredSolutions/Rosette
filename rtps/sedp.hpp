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
#include "spdp.hpp"   // pid namespace, plcdr_write_param

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
// Sends PL_CDR_LE DATA submessages for all locally registered
// endpoints to a specific remote participant's metatraffic locator.
//
// Called by RTPSParticipant when a new peer is discovered via SPDP.
// Each call serialises all known local writers and readers.
//
// §8.5.4.3, p. 167: the publications and subscriptions writers use
// the SEDPbuiltin EntityIds defined in entity_id::.
// ============================================================

#ifndef RTPS_MAX_LOCAL_ENDPOINTS
#  define RTPS_MAX_LOCAL_ENDPOINTS 8
#endif

class SedpWriter {
public:

    explicit SedpWriter(ITransport& transport) noexcept
        : transport_(transport) {}

    // ----------------------------------------------------------
    // register_writer() / register_reader()
    //
    // Register a local endpoint for inclusion in SEDP announcements.
    // Must be called before announce_to_peer().
    // ----------------------------------------------------------
    bool register_writer(const DiscoveredWriterData& wd) noexcept {
        DiscoveredWriterData* slot = alloc_writer_slot();
        if (!slot) return false;
        *slot = wd;
        return true;
    }

    bool register_reader(const DiscoveredReaderData& rd) noexcept {
        DiscoveredReaderData* slot = alloc_reader_slot();
        if (!slot) return false;
        *slot = rd;
        return true;
    }

    // ----------------------------------------------------------
    // announce_to_peer()
    //
    // Send SEDP publications and subscriptions DATA to one remote
    // participant's metatraffic unicast locator.
    //
    // §8.5.4, p. 163:
    //   "The SEDPbuiltinPublicationsWriter sends DiscoveredWriterData
    //    for all local DataWriters to newly discovered participants."
    //   "The SEDPbuiltinSubscriptionsWriter sends DiscoveredReaderData
    //    for all local DataReaders."
    //
    // @param local_prefix   This participant's GUID prefix.
    // @param dest           Remote metatraffic unicast locator.
    // ----------------------------------------------------------
    void announce_to_peer(const GuidPrefix& local_prefix,
                          const Locator&    dest) noexcept;

private:

    ITransport& transport_;
    MessageBuilder builder_;

    DiscoveredWriterData local_writers_[RTPS_MAX_LOCAL_ENDPOINTS] = {};
    DiscoveredReaderData local_readers_[RTPS_MAX_LOCAL_ENDPOINTS] = {};

    DiscoveredWriterData* alloc_writer_slot() noexcept {
        for (uint8_t i = 0; i < RTPS_MAX_LOCAL_ENDPOINTS; ++i)
            if (!local_writers_[i].is_valid()) return &local_writers_[i];
        return nullptr;
    }
    DiscoveredReaderData* alloc_reader_slot() noexcept {
        for (uint8_t i = 0; i < RTPS_MAX_LOCAL_ENDPOINTS; ++i)
            if (!local_readers_[i].is_valid()) return &local_readers_[i];
        return nullptr;
    }

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
// ============================================================

class SedpReader {
public:

    explicit SedpReader(EndpointMatchTable& match_table) noexcept
        : match_table_(match_table) {}

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
    // ----------------------------------------------------------
    void on_publication_data(const GuidPrefix& sender_prefix,
                             const Octet*      payload,
                             uint32_t          payload_len) noexcept;

    // ----------------------------------------------------------
    // on_subscription_data()
    //
    // Process a SEDP subscriptions DATA submessage payload.
    // ----------------------------------------------------------
    void on_subscription_data(const GuidPrefix& sender_prefix,
                              const Octet*      payload,
                              uint32_t          payload_len) noexcept;

private:

    EndpointMatchTable& match_table_;

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
