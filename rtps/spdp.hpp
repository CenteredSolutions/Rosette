/**
 * @file rtps/spdp.hpp
 * @brief Simple Participant Discovery Protocol — §8.5.3 (OMG formal/2022-04-01).
 *
 * SPDP is the bootstrap discovery mechanism used to locate other RTPS
 * participants on the network before SEDP endpoint discovery can proceed.
 *
 * Spec model (§8.5.3, p. 157)
 * ----------------------------
 * Each participant runs two builtin endpoints:
 *
 *   SPDPbuiltinParticipantWriter  — §8.5.3.3, EntityId 0x000100C2
 *     A best-effort StatefulWriter.  Periodically broadcasts
 *     SPDPdiscoveredParticipantData as a PL_CDR DATA submessage to the
 *     SPDP well-known multicast or unicast locator.
 *     Resend period = resendParticipantDataPeriod (§8.5.3.3, p. 162).
 *
 *   SPDPbuiltinParticipantReader  — §8.5.3.3, EntityId 0x000100C7
 *     A best-effort StatefulReader.  Receives SPDPdiscoveredParticipantData
 *     and maintains a PeerTable (one entry per discovered participant).
 *     Each entry carries a lease expiry timestamp; entries are removed
 *     when leaseDuration elapses without a refresh (§8.5.3.2, p. 160).
 *
 * Encoding
 * --------
 * SPDPdiscoveredParticipantData is encoded as a PL_CDR (Parameter List
 * CDR) payload, per §9.6.2, p. 256.  Each field is encoded as a
 * { PID (2 bytes), length (2 bytes), value (length bytes) } tuple.
 * The list is terminated by PID_SENTINEL (0x0001, length 0).
 *
 * Well-known locators (§8.5.3.3, p. 162; §9.6.1, p. 254)
 * ---------------------------------------------------------
 * SPDP uses the discovery multicast group 239.255.0.1 on port
 *   PB + DG * domainId + d0   where PB=7400, DG=250, d0=0.
 * For domainId=0 this is 239.255.0.1:7400.
 * Unicast metatraffic port = PB + DG * domainId + d1 + 2 * participantId
 *   where d1=10.  For domainId=0, participantId=0: port 7410.
 *
 * PIDs relevant to SPDPdiscoveredParticipantData (§9.6.3.1, p. 258)
 * ------------------------------------------------------------------
 *   PID_PROTOCOL_VERSION      = 0x0015
 *   PID_VENDOR_ID             = 0x0016
 *   PID_PARTICIPANT_GUID      = 0x0050
 *   PID_METATRAFFIC_UNICAST_LOCATOR   = 0x0032
 *   PID_METATRAFFIC_MULTICAST_LOCATOR = 0x0033
 *   PID_DEFAULT_UNICAST_LOCATOR       = 0x002F
 *   PID_DEFAULT_MULTICAST_LOCATOR     = 0x0048
 *   PID_PARTICIPANT_LEASE_DURATION    = 0x0002
 *   PID_BUILTIN_ENDPOINT_SET          = 0x0058
 *   PID_SENTINEL                      = 0x0001
 *
 * SEDP trigger (implementation policy)
 * -------------------------------------
 * Per the project constraint, SEDP announcements are sent *once* on first
 * discovery of a new peer.  SpdpReader tracks a `sedp_announced` flag per
 * peer so that subsequent SPDP refreshes do not re-trigger SEDP.
 *
 * Spec references (all OMG formal/2022-04-01)
 * -------------------------------------------
 *   SPDP overview              — §8.5.3,   p. 157
 *   SPDPdiscoveredParticipantData — §8.5.3.2, Table 8.73, p. 160
 *   SPDP builtin endpoints     — §8.5.3.3, p. 162
 *   resendParticipantDataPeriod — §8.5.3.3, p. 162
 *   Lease expiry               — §8.5.3.2, p. 160
 *   Well-known locators        — §9.6.1,   Table 9.35, p. 254
 *   PL_CDR encoding            — §9.6.2,   p. 256
 *   Parameter IDs              — §9.6.3.1, Table 9.14, p. 258
 */

#pragma once

#include "types.hpp"
#include "message_builder.hpp"

namespace rtps {

struct ITransport;   // Forward — declared in stateful_writer.hpp.

// ============================================================
// Parameter IDs for PL_CDR encoding of SPDP data
// §9.6.3.1, Table 9.14, p. 258
// ============================================================

namespace pid {
    inline constexpr uint16_t SENTINEL                       = 0x0001u;
    inline constexpr uint16_t PARTICIPANT_LEASE_DURATION     = 0x0002u;
    inline constexpr uint16_t DEFAULT_UNICAST_LOCATOR        = 0x002Fu;
    inline constexpr uint16_t METATRAFFIC_UNICAST_LOCATOR    = 0x0032u;
    inline constexpr uint16_t METATRAFFIC_MULTICAST_LOCATOR  = 0x0033u;
    inline constexpr uint16_t DEFAULT_MULTICAST_LOCATOR      = 0x0048u;
    inline constexpr uint16_t PARTICIPANT_GUID               = 0x0050u;
    inline constexpr uint16_t BUILTIN_ENDPOINT_SET           = 0x0058u;
    inline constexpr uint16_t PROTOCOL_VERSION               = 0x0015u;
    inline constexpr uint16_t VENDOR_ID                      = 0x0016u;
    // SEDP endpoint data PIDs — §9.6.3.2, Table 9.15, p. 263
    inline constexpr uint16_t TOPIC_NAME                     = 0x0005u;
    inline constexpr uint16_t TYPE_NAME                      = 0x0007u;
    inline constexpr uint16_t RELIABILITY                    = 0x001Au;
    inline constexpr uint16_t DURABILITY                     = 0x001Du;
    inline constexpr uint16_t ENDPOINT_GUID                  = 0x005Au;
    inline constexpr uint16_t UNICAST_LOCATOR                = 0x002Fu;   // Reused for endpoints.
}

// ============================================================
// PL_CDR serialiser helpers
//
// §9.6.2, p. 256 — parameter list format:
//   RepresentationHeader (4 bytes): PL_CDR_LE (0x0003) + options (0x0000)
//   Repeated: { uint16 PID, uint16 length, uint8 value[length] }
//   Terminated by: { PID_SENTINEL (0x0001), 0x0000 }
//
// All lengths are padded to a 4-byte boundary.
// ============================================================

/// Write a PID parameter into a flat byte buffer.
/// Returns the number of bytes written (header + padded value), or 0
/// on overflow.
///
/// @param buf      Destination buffer.
/// @param cap      Total capacity of buf in bytes.
/// @param off      Current write offset; advanced on success.
/// @param pid_val  Parameter ID (§9.6.3.1).
/// @param value    Pointer to parameter value bytes.
/// @param val_len  Unpadded value length in bytes.
inline bool plcdr_write_param(Octet*   buf,
                              uint32_t cap,
                              uint32_t& off,
                              uint16_t  pid_val,
                              const void* value,
                              uint16_t  val_len) noexcept
{
    // Pad length to 4-byte boundary — §9.6.2, p. 256.
    const uint16_t padded = static_cast<uint16_t>((val_len + 3u) & ~3u);
    const uint32_t total  = 4u + padded;   // PID(2) + len(2) + value(padded)
    if (off + total > cap) return false;

    memcpy(buf + off, &pid_val, 2u); off += 2u;
    memcpy(buf + off, &padded,  2u); off += 2u;
    if (val_len > 0 && value) {
        memcpy(buf + off, value, val_len);
    }
    // Zero-fill the padding bytes.
    if (padded > val_len) {
        memset(buf + off + val_len, 0, padded - val_len);
    }
    off += padded;
    return true;
}

/// Write PID_SENTINEL to terminate a parameter list.
inline bool plcdr_write_sentinel(Octet* buf, uint32_t cap, uint32_t& off) noexcept {
    if (off + 4u > cap) return false;
    const uint16_t pid = pid::SENTINEL;
    const uint16_t len = 0u;
    memcpy(buf + off, &pid, 2u); off += 2u;
    memcpy(buf + off, &len, 2u); off += 2u;
    return true;
}

// ============================================================
// PeerEntry — one discovered remote participant
//
// §8.5.3.2, Table 8.73, p. 160.
// Maintained in SpdpReader::peer_table_.
// ============================================================

struct PeerEntry {
    SPDPdiscoveredParticipantData data;         ///< Received participant data.
    uint32_t                      leaseExpiryMs = 0; ///< RTPS_GET_MS() at expiry.
    bool                          alive         = false;
    bool                          sedp_announced = false; ///< SEDP sent for this peer.
};

// ============================================================
// Well-known port formula
// §9.6.1, Table 9.35, p. 254
//
// PB  = 7400
// DG  = 250
// d0  = 0   (SPDP multicast)
// d1  = 10  (SPDP metatraffic unicast)
// d2  = 1   (user unicast)
// d3  = 11  (user multicast — not used on this target)
//
// Discovery multicast port = PB + DG * domainId + d0
// Metatraffic unicast port = PB + DG * domainId + d1 + 2 * participantId
// ============================================================

inline constexpr uint32_t spdp_multicast_port(uint8_t domain_id) noexcept {
    return 7400u + 250u * domain_id + 0u;
}
inline constexpr uint32_t spdp_unicast_port(uint8_t domain_id,
                                            uint8_t participant_id) noexcept {
    return 7400u + 250u * domain_id + 10u + 2u * participant_id;
}

/// SPDP well-known IPv4 multicast address — §9.6.1, p. 254: 239.255.0.1
inline Locator spdp_multicast_locator_v4(uint8_t domain_id) noexcept {
    return make_udpv4_locator(0xEFFF0001u, static_cast<uint16_t>(
                              spdp_multicast_port(domain_id)));
}

/// SPDP well-known IPv6 multicast address — FF02::FFFF:239.255.0.1
inline Locator spdp_multicast_locator_v6(uint8_t domain_id) noexcept {
    const uint8_t ipv6[16] = {
        0xFFu, 0x02u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
        0x00u, 0x00u, 0xFFu, 0xFFu, 0xEFu, 0xFFu, 0x00u, 0x01u,
    };
    return make_udpv6_locator(ipv6, static_cast<uint16_t>(
                                        spdp_multicast_port(domain_id)));
}

inline bool spdp_ipv6_discovery_available() noexcept
{
    uint8_t ipv6[16] = {};
    return rtps_platform_get_ipv6_addr(ipv6);
}

inline Locator spdp_multicast_locator(uint8_t domain_id) noexcept {
    return spdp_ipv6_discovery_available()
        ? spdp_multicast_locator_v6(domain_id)
        : spdp_multicast_locator_v4(domain_id);
}

// ============================================================
// SpdpWriter
//
// §8.5.3.3, p. 162 — SPDPbuiltinParticipantWriter.
//
// Serialises this participant's SPDPdiscoveredParticipantData as a
// PL_CDR DATA submessage and sends it to the SPDP multicast locator
// (and optionally configured unicast locators) at
// resendParticipantDataPeriod intervals.
//
// resendParticipantDataPeriod is not given a fixed default in the spec;
// implementations typically use leaseDuration / 3.  Here it is
// configurable via RTPS_SPDP_RESEND_PERIOD_MS with a default of
// RTPS_SPDP_LEASE_DURATION_MS / 3.
//
// The writer is BEST_EFFORT (§8.5.3.3) — no HEARTBEAT/ACKNACK cycle.
// ============================================================

#ifndef RTPS_SPDP_LEASE_DURATION_MS
#  define RTPS_SPDP_LEASE_DURATION_MS  20000u   ///< 20 s default — §8.5.3.2
#endif
#ifndef RTPS_SPDP_RESEND_PERIOD_MS
#  define RTPS_SPDP_RESEND_PERIOD_MS   (RTPS_SPDP_LEASE_DURATION_MS / 3u)
#endif

class SpdpWriter {
public:

    // ----------------------------------------------------------
    // Constructor
    //
    // @param local_data   This participant's data to announce.
    //                     The caller populates the struct before
    //                     constructing SpdpWriter and must keep it
    //                     live for the lifetime of the writer.
    // @param transport    Transport for sending.
    // @param dest         Destination locator (multicast or unicast).
    // @param resend_ms    resendParticipantDataPeriod in ms.
    // ----------------------------------------------------------
    SpdpWriter(const SPDPdiscoveredParticipantData& local_data,
               ITransport&                          transport,
               const Locator&                       dest,
               uint32_t                             resend_ms =
                   RTPS_SPDP_RESEND_PERIOD_MS) noexcept
        : local_data_(local_data)
        , transport_(transport)
        , dest_(dest)
        , resend_ms_(resend_ms)
    {}

    // ----------------------------------------------------------
    // announce()
    //
    // Build and immediately send one SPDP announcement.
    // Called at startup and whenever forced (e.g. after a new peer
    // is added to the unicast dest list).
    //
    // §8.5.3.3, p. 162: "The SPDPbuiltinParticipantWriter sends a
    //  DATA Submessage containing the SPDPdiscoveredParticipantData
    //  of the local participant."
    //
    // Encoding: PL_CDR_LE (RepresentationIdentifier = 0x0003) per §9.6.2.
    // ----------------------------------------------------------
    void announce() noexcept;

    // ----------------------------------------------------------
    // tick()
    //
    // Call periodically.  Sends the announcement if
    // resendParticipantDataPeriod has elapsed since the last send.
    //
    // @param now_ms  Current RTPS_GET_MS().
    // ----------------------------------------------------------
    void tick(uint32_t now_ms) noexcept {
        if ((now_ms - last_sent_ms_) >= resend_ms_) {
            announce();
            last_sent_ms_ = now_ms;
        }
    }

    void set_dest(const Locator& l) noexcept { dest_ = l; }

private:

    /// Serialise local_data_ into a PL_CDR payload buffer.
    /// Returns the number of bytes written.
    uint32_t serialise(Octet* buf, uint32_t cap) const noexcept;

    const SPDPdiscoveredParticipantData& local_data_;
    ITransport&  transport_;
    Locator      dest_;
    uint32_t     resend_ms_;
    uint32_t     last_sent_ms_ = 0;
    MessageBuilder builder_;
};

// ============================================================
// SpdpReader
//
// §8.5.3.3, p. 162 — SPDPbuiltinParticipantReader.
//
// Receives DATA submessages on the SPDP well-known locator, deserialises
// the PL_CDR payload, and maintains a PeerTable.
//
// Lease management (§8.5.3.2, p. 160):
//   "If the Writer does not refresh its data within leaseDuration
//    the participant is considered inactive and its state is removed."
//   — §8.5.3.2, Table 8.73.
//   Enforced in tick() by comparing RTPS_GET_MS() against leaseExpiryMs.
//
// on_new_peer callback: fired once per newly discovered participant.
// on_peer_removed callback: fired when a participant's lease expires.
// Both callbacks carry a const PeerEntry& valid for the call duration.
// ============================================================

#ifndef RTPS_MAX_PEERS
#  define RTPS_MAX_PEERS 8
#endif

class SpdpReader {
public:

    using OnNewPeer     = void (*)(const PeerEntry&, void*) noexcept;
    using OnPeerRemoved = void (*)(const PeerEntry&, void*) noexcept;

    SpdpReader(OnNewPeer     on_new_peer     = nullptr,
               OnPeerRemoved on_peer_removed = nullptr,
               void*         user_data       = nullptr) noexcept
        : on_new_peer_(on_new_peer)
        , on_peer_removed_(on_peer_removed)
        , user_data_(user_data)
    {}

    // ----------------------------------------------------------
    // on_data()
    //
    // Process a raw serialized-payload buffer received on the SPDP
    // well-known locator (already stripped of the RTPS message header
    // and DATA submessage headers by the parser).
    //
    // "The SPDPbuiltinParticipantReader processes DATA Submessages
    //  containing SPDPdiscoveredParticipantData and updates the
    //  participant discovery information."
    //  — §8.5.3.3, p. 162.
    //
    // If the GuidPrefix is new: insert into peer table, fire on_new_peer.
    // If already present: refresh leaseExpiryMs.
    // In both cases sedp_announced remains false on first sight.
    //
    // @param sender_prefix  GUID prefix from the RTPS message header.
    // @param payload        Pointer to the PL_CDR serialized payload.
    // @param payload_len    Byte count.
    // @param now_ms         Current RTPS_GET_MS().
    // @returns true if the payload was successfully parsed.
    // ----------------------------------------------------------
    bool on_data(const GuidPrefix& sender_prefix,
                 const Octet*      payload,
                 uint32_t          payload_len,
                 uint32_t          now_ms) noexcept;

    // ----------------------------------------------------------
    // tick()
    //
    // Check for expired leases and evict stale peers.
    //
    // §8.5.3.2, p. 160:
    //   "If a participant does not receive a refresh within
    //    leaseDuration, it removes the remote participant entry."
    //
    // @param now_ms  Current RTPS_GET_MS().
    // ----------------------------------------------------------
    void tick(uint32_t now_ms) noexcept;

    // ----------------------------------------------------------
    // mark_sedp_announced()
    //
    // Called by the participant after SEDP has been sent for a peer
    // so that subsequent SPDP refreshes do not re-trigger SEDP.
    // ----------------------------------------------------------
    void mark_sedp_announced(const GuidPrefix& prefix) noexcept {
        PeerEntry* e = find_peer(prefix);
        if (e) e->sedp_announced = true;
    }

    // ----------------------------------------------------------
    // Iterate over live peers.
    // fn : void(const PeerEntry&)
    // ----------------------------------------------------------
    template<typename Fn>
    void for_each_peer(Fn&& fn) const noexcept {
        for (uint8_t i = 0; i < RTPS_MAX_PEERS; ++i) {
            if (peers_[i].alive) fn(peers_[i]);
        }
    }

    uint8_t peer_count() const noexcept {
        uint8_t n = 0;
        for (uint8_t i = 0; i < RTPS_MAX_PEERS; ++i) {
            if (peers_[i].alive) ++n;
        }
        return n;
    }

private:

    PeerEntry  peers_[RTPS_MAX_PEERS] = {};
    OnNewPeer  on_new_peer_;
    OnPeerRemoved on_peer_removed_;
    void*      user_data_;

    PeerEntry* find_peer(const GuidPrefix& prefix) noexcept {
        for (uint8_t i = 0; i < RTPS_MAX_PEERS; ++i) {
            if (peers_[i].alive &&
                peers_[i].data.guidPrefix == prefix)
                return &peers_[i];
        }
        return nullptr;
    }

    PeerEntry* find_free() noexcept {
        for (uint8_t i = 0; i < RTPS_MAX_PEERS; ++i) {
            if (!peers_[i].alive) return &peers_[i];
        }
        return nullptr;
    }

    /// Deserialise a PL_CDR payload into out.
    /// Returns true if at least the guidPrefix PID was found.
    static bool deserialise(const Octet*              payload,
                            uint32_t                  len,
                            SPDPdiscoveredParticipantData& out) noexcept;
};

} // namespace rtps
