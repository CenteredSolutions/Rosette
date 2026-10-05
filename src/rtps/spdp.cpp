/**
 * @file rtps/spdp.cpp
 * @brief SPDP implementation — §8.5.3 (OMG formal/2022-04-01).
 */

#include "spdp.hpp"
#include "stateful_writer.hpp"   // ITransport
#include "log.hpp"
#include <string.h>

namespace rtps {

namespace {

inline bool decode_plcdr_rep_id(const uint8_t* payload,
                                uint32_t       len,
                                bool&          little_endian) noexcept
{
    if ((payload == nullptr) || (len < 2u)) {
        return false;
    }

    const uint16_t rep_id_be = (static_cast<uint16_t>(payload[0]) << 8u) |
                               static_cast<uint16_t>(payload[1]);
    switch (rep_id_be) {
    case 0x0003u: little_endian = true;  return true;   // PL_CDR_LE (canonical)
    case 0x0002u: little_endian = false; return true;   // PL_CDR_BE (canonical)
    case 0x0300u: little_endian = true;  return true;   // legacy host-order tolerant
    case 0x0200u: little_endian = false; return true;   // legacy host-order tolerant
    default: return false;
    }
}

inline uint16_t read_u16_wire(const uint8_t* p, bool little_endian) noexcept
{
    if (little_endian) {
        return static_cast<uint16_t>(p[0]) |
               static_cast<uint16_t>(p[1] << 8u);
    }
    return static_cast<uint16_t>(p[0] << 8u) |
           static_cast<uint16_t>(p[1]);
}

} // namespace

// ============================================================
// SpdpWriter — serialisation and transmission
// ============================================================

// ----------------------------------------------------------
// SpdpWriter::serialise()
//
// Produces a PL_CDR_LE parameter-list payload for the local
// participant data.
//
// §9.6.2, p. 256 — PL_CDR format:
//   RepresentationHeader (PL_CDR_LE = 0x0003, options = 0x0000)
//   { PID, length (padded), value } repeated
//   PID_SENTINEL (0x0001, length 0)
//
// §9.6.3.1, Table 9.14, p. 258 — PIDs used:
//   PID_PARTICIPANT_GUID              = 0x0050
//   PID_PROTOCOL_VERSION              = 0x0015
//   PID_VENDOR_ID                     = 0x0016
//   PID_METATRAFFIC_UNICAST_LOCATOR   = 0x0032
//   PID_METATRAFFIC_MULTICAST_LOCATOR = 0x0033
//   PID_DEFAULT_UNICAST_LOCATOR       = 0x0031
//   PID_DEFAULT_MULTICAST_LOCATOR     = 0x0048
//   PID_PARTICIPANT_LEASE_DURATION    = 0x0002
//   PID_BUILTIN_ENDPOINT_SET          = 0x0058
//   PID_SENTINEL                      = 0x0001
//
// Locator wire size: kind(4) + port(4) + address(16) = 24 bytes.
// Duration wire size: seconds(4) + fraction(4) = 8 bytes.
// ----------------------------------------------------------

uint32_t SpdpWriter::serialise(Octet* buf, uint32_t cap) const noexcept {
    uint32_t off = 0;

    // RepresentationHeader — PL_CDR_LE = 0x0003, options = 0x0000.
    // §9.6.2, p. 256 and §10.2, p. 288.
    if (off + 4u > cap) return 0;
    // CDR encapsulation identifier/options are fixed-octet order.
    // PL_CDR_LE = 0x0003 => bytes {0x00, 0x03}.
    buf[off++] = 0x00u;
    buf[off++] = 0x03u;
    buf[off++] = 0x00u;
    buf[off++] = 0x00u;

    const auto& d = local_data_;

    // PID_PARTICIPANT_GUID (0x0050) — 16 bytes: GuidPrefix(12) + EntityId(4).
    // §9.6.3.1, Table 9.14.
    {
        Octet guid_bytes[16] = {};
        memcpy(guid_bytes,      d.guidPrefix.value, 12u);
        // EntityId for the participant = {0x00,0x00,0x01,0xC1} — §8.2.4.
        guid_bytes[12] = 0x00u; guid_bytes[13] = 0x00u;
        guid_bytes[14] = 0x01u; guid_bytes[15] = 0xC1u;
        if (!plcdr_write_param(buf, cap, off, pid::PARTICIPANT_GUID,
                               guid_bytes, 16u)) return 0;
    }

    // PID_PROTOCOL_VERSION (0x0015) — 2 bytes: {major, minor}.
    {
        Octet pv[2] = { d.protocolVersion.major, d.protocolVersion.minor };
        if (!plcdr_write_param(buf, cap, off, pid::PROTOCOL_VERSION, pv, 2u))
            return 0;
    }

    // PID_VENDOR_ID (0x0016) — 2 bytes.
    if (!plcdr_write_param(buf, cap, off, pid::VENDOR_ID,
                           d.vendorId.vendorId, 2u)) return 0;

    // PID_METATRAFFIC_UNICAST_LOCATOR (0x0032) — 24 bytes each.
    for (uint8_t i = 0; i < d.metatrafficUnicastLocatorCount; ++i) {
        if (!plcdr_write_param(buf, cap, off,
                               pid::METATRAFFIC_UNICAST_LOCATOR,
                               &d.metatrafficUnicastLocatorList[i],
                               24u)) return 0;
    }

    // PID_METATRAFFIC_MULTICAST_LOCATOR (0x0033).
    for (uint8_t i = 0; i < d.metatrafficMulticastLocatorCount; ++i) {
        if (!plcdr_write_param(buf, cap, off,
                               pid::METATRAFFIC_MULTICAST_LOCATOR,
                               &d.metatrafficMulticastLocatorList[i],
                               24u)) return 0;
    }

    // PID_DEFAULT_UNICAST_LOCATOR (0x0031).
    for (uint8_t i = 0; i < d.defaultUnicastLocatorCount; ++i) {
        if (!plcdr_write_param(buf, cap, off,
                               pid::DEFAULT_UNICAST_LOCATOR,
                               &d.defaultUnicastLocatorList[i],
                               24u)) return 0;
    }

    // PID_DEFAULT_MULTICAST_LOCATOR (0x0048).
    for (uint8_t i = 0; i < d.defaultMulticastLocatorCount; ++i) {
        if (!plcdr_write_param(buf, cap, off,
                               pid::DEFAULT_MULTICAST_LOCATOR,
                               &d.defaultMulticastLocatorList[i],
                               24u)) return 0;
    }

    // PID_PARTICIPANT_LEASE_DURATION (0x0002) — 8 bytes: {seconds(4), nanoseconds(4)}.
    {
        uint32_t lease[2] = {
            static_cast<uint32_t>(d.leaseDuration.seconds),
            d.leaseDuration.nanoseconds
        };
        if (!plcdr_write_param(buf, cap, off,
                               pid::PARTICIPANT_LEASE_DURATION,
                               lease, 8u)) return 0;
    }

    // PID_BUILTIN_ENDPOINT_SET (0x0058) — 4 bytes.
    if (!plcdr_write_param(buf, cap, off,
                           pid::BUILTIN_ENDPOINT_SET,
                           &d.availableBuiltinEndpoints, 4u)) return 0;

    // PID_SENTINEL (0x0001).
    if (!plcdr_write_sentinel(buf, cap, off)) return 0;

    return off;
}

// ----------------------------------------------------------
// SpdpWriter::announce()
//
// §8.5.3.3, p. 162:
//   "The SPDPbuiltinParticipantWriter sends a DATA Submessage
//    containing the local SPDPdiscoveredParticipantData."
//
// Message structure:
//   RTPS Header
//   DATA submessage (writerId = SPDP_BUILTIN_PARTICIPANT_WRITER,
//                    readerId = SPDP_BUILTIN_PARTICIPANT_READER)
//     SerializedPayload: PL_CDR_LE parameter list
// ----------------------------------------------------------

void SpdpWriter::do_announce(const Locator& dest, bool use_unicast) noexcept {
    BufRef payload_buf = buf_alloc_with_headroom(256u, kRtpsDataHeadroom);
    if (!payload_buf.valid()) {
        RTPS_LOG_E("spdp", "buf_alloc failed for participant announce");
        return;
    }

    const uint32_t serialised_len =
        serialise(payload_buf.data, payload_buf.capacity());

    if (serialised_len == 0) {
        RTPS_LOG_E("spdp", "participant data serialisation failed (capacity=%lu)",
                   (unsigned long)payload_buf.capacity());
        payload_buf.reset();
        return;
    }
    payload_buf.size = serialised_len;

    // The DATA submessage for SPDP does NOT prepend an extra CDR payload
    // encapsulation header. The PL_CDR RepresentationHeader written by
    // serialise() is already at payload[0..3] per §9.6.2.
    //
    // Build DATA framing in-place over the headroom-backed payload.
    bool ok = builder_.begin_with_payload(static_cast<BufRef&&>(payload_buf))
              && builder_.prepend_data_message(
                  local_data_.guidPrefix,
                  entity_id::SPDP_BUILTIN_PARTICIPANT_READER,
                  entity_id::SPDP_BUILTIN_PARTICIPANT_WRITER,
                  SequenceNumber::from_int64(next_sn_));

    if (ok) {
        // Each announcement is a new change: best-effort SPDP readers
        // (Fast DDS, Cyclone DDS) discard a sample whose SN is not greater
        // than the last one seen from this writer, which would hide any
        // later change to this participant's data.
        ++next_sn_;
        RTPS_LOG_D("spdp", "%s participant (%lu bytes)",
                   use_unicast ? "unicasting" : "announcing",
                   (unsigned long)serialised_len);
        BufRef msg = builder_.finish();
        const bool sent = use_unicast ? transport_.send_unicast(msg, dest)
                                      : transport_.send_multicast(msg, dest);
        if (!sent) {
            RTPS_LOG_W("spdp", "announce send failed: kind=%ld port=%lu",
                       (long)dest.kind,
                       (unsigned long)dest.port);
        }
        msg.reset();
    } else {
        builder_.abort();
    }
}

void SpdpWriter::announce() noexcept {
    do_announce(dest_, false);
}

void SpdpWriter::announce_unicast(const Locator& dest) noexcept {
    do_announce(dest, true);
}

// ============================================================
// DiscoveredParticipant — what the peer table keeps of a peer
// ============================================================

DiscoveredParticipant DiscoveredParticipant::from(const SPDPdiscoveredParticipantData& announced,
                                                  const LocalSubnets& subnets) noexcept {
    DiscoveredParticipant p{};
    p.guidPrefix                = announced.guidPrefix;
    p.vendorId = announced.vendorId;
    p.availableBuiltinEndpoints = announced.availableBuiltinEndpoints;
    p.metatrafficUnicastLocatorCount = subnets.select(
        p.metatrafficUnicastLocatorList, RTPS_MAX_PEER_LOCATORS,
        announced.metatrafficUnicastLocatorList, announced.metatrafficUnicastLocatorCount);
    p.defaultUnicastLocatorCount =
        subnets.select(p.defaultUnicastLocatorList, RTPS_MAX_PEER_LOCATORS,
                       announced.defaultUnicastLocatorList, announced.defaultUnicastLocatorCount);
    return p;
}

// ============================================================
// SpdpReader — deserialisation and peer table management
// ============================================================

const LocalSubnets SpdpReader::kNoSubnets{};

// ----------------------------------------------------------
// SpdpReader::deserialise()
//
// Parse a PL_CDR_LE parameter list into an
// SPDPdiscoveredParticipantData struct.
//
// §9.6.2, p. 256 — reader algorithm:
//   1. Read RepresentationHeader (4 bytes); verify PL_CDR_LE or PL_CDR_BE.
//   2. Loop: read PID(2) + length(2); dispatch on PID.
//   3. Stop at PID_SENTINEL.
//   4. Unknown PIDs are skipped by advancing `length` bytes.
//
// Returns true if the mandatory PID_PARTICIPANT_GUID was found.
// ----------------------------------------------------------

bool SpdpReader::deserialise(const Octet* payload, uint32_t len, const LocalSubnets& subnets,
                             SPDPdiscoveredParticipantData& out) noexcept {
    if (len < 4u) return false;

    // RepresentationHeader — §9.6.2, p. 256.
    bool little_endian = true;
    if (!decode_plcdr_rep_id(payload, len, little_endian)) {
        return false;
    }

    uint32_t off     = 4u;    // Past the 4-byte RepresentationHeader.
    bool     got_guid = false;

    while (off + 4u <= len) {
        const uint16_t pid_val = read_u16_wire(payload + off, little_endian);
        const uint16_t param_len = read_u16_wire(payload + off + 2u, little_endian);
        off += 4u;

        // Pad length to 4-byte boundary for next-PID alignment.
        const uint16_t padded = static_cast<uint16_t>((param_len + 3u) & ~3u);

        if (off + padded > len && pid_val != pid::SENTINEL) break;

        const Octet* val = payload + off;

        switch (pid_val) {
            // PID_PARTICIPANT_GUID — 16 bytes.
            case pid::PARTICIPANT_GUID:
                if (param_len >= 12u) {
                    memcpy(out.guidPrefix.value, val, 12u);
                    got_guid = true;
                }
                break;

            // PID_PROTOCOL_VERSION — 2 bytes.
            case pid::PROTOCOL_VERSION:
                if (param_len >= 2u) {
                    out.protocolVersion.major = val[0];
                    out.protocolVersion.minor = val[1];
                }
                break;

            // PID_VENDOR_ID — 2 bytes.
            case pid::VENDOR_ID:
                if (param_len >= 2u) {
                    out.vendorId.vendorId[0] = val[0];
                    out.vendorId.vendorId[1] = val[1];
                }
                break;

            // PID_METATRAFFIC_UNICAST_LOCATOR — 24 bytes per locator.
            // Ranked as read, so a reachable locator announced after the
            // list's capacity is still kept (LocalSubnets::offer()).
            case pid::METATRAFFIC_UNICAST_LOCATOR: {
                Locator locator{};
                if (param_len >= 24u && Locator::read_from(locator, val, 24u, little_endian)) {
                    subnets.offer(out.metatrafficUnicastLocatorList,
                                  out.metatrafficUnicastLocatorCount,
                                  RTPS_MAX_LOCATORS_PER_PARTICIPANT, locator);
                }

                break;
            }

            // PID_METATRAFFIC_MULTICAST_LOCATOR.
            case pid::METATRAFFIC_MULTICAST_LOCATOR:
                if (param_len >= 24u &&
                    out.metatrafficMulticastLocatorCount <
                        RTPS_MAX_LOCATORS_PER_PARTICIPANT) {
                    if (Locator::read_from(out.metatrafficMulticastLocatorList[
                                       out.metatrafficMulticastLocatorCount],
                                       val, 24u, little_endian)) {
                        out.metatrafficMulticastLocatorCount++;
                    }
                }
                break;

            // PID_DEFAULT_UNICAST_LOCATOR.
            // Accept legacy PID_UNICAST_LOCATOR (0x002F) in SPDP payloads
            // for interoperability with older stacks/captures.
            case pid::DEFAULT_UNICAST_LOCATOR:
            case pid::UNICAST_LOCATOR: {
                Locator locator{};
                if (param_len >= 24u && Locator::read_from(locator, val, 24u, little_endian)) {
                    subnets.offer(out.defaultUnicastLocatorList, out.defaultUnicastLocatorCount,
                                  RTPS_MAX_LOCATORS_PER_PARTICIPANT, locator);
                }

                break;
            }

            // PID_DEFAULT_MULTICAST_LOCATOR.
            case pid::DEFAULT_MULTICAST_LOCATOR:
                if (param_len >= 24u &&
                    out.defaultMulticastLocatorCount <
                        RTPS_MAX_LOCATORS_PER_PARTICIPANT
                ) {
                    if (Locator::read_from(out.defaultMulticastLocatorList[
                                       out.defaultMulticastLocatorCount],
                                       val, 24u, little_endian)) {
                        out.defaultMulticastLocatorCount++;
                    }
                }
                break;

            // PID_PARTICIPANT_LEASE_DURATION — 8 bytes: seconds(4) + nanoseconds(4).
            case pid::PARTICIPANT_LEASE_DURATION:
                Duration::read_from(out.leaseDuration, val, param_len, little_endian);
                break;

            // PID_BUILTIN_ENDPOINT_SET — 4 bytes.
            case pid::BUILTIN_ENDPOINT_SET:
                if (param_len >= 4u) {
                    memcpy(&out.availableBuiltinEndpoints, val, 4u);
                    fixup_u32(out.availableBuiltinEndpoints, !little_endian);
                }
                break;

            // PID_SENTINEL — end of list.
            case pid::SENTINEL:
                return got_guid;

            default:
                break;   // Unknown PID — skip.
        }

        off += padded;
    }
    return got_guid;
}

// ----------------------------------------------------------
// SpdpReader::on_data()
//
// §8.5.3.3, p. 162:
//   "The SPDPbuiltinParticipantReader processes the received DATA
//    and updates the Participant's view of remote participants."
// ----------------------------------------------------------

bool SpdpReader::on_data(const GuidPrefix& sender_prefix,
                          const Octet*      payload,
                          uint32_t          payload_len,
                          uint32_t          now_ms) noexcept
{
    SPDPdiscoveredParticipantData parsed{};
    if (!deserialise(payload, payload_len, *subnets_, parsed)) {
        RTPS_LOG_W("spdp", "failed to deserialise participant data (%lu bytes)",
                   (unsigned long)payload_len);
        return false;
    }

    // Use the sender_prefix from the RTPS header if the payload's GUID
    // prefix is still zeroed after parsing (robustness).
    if (parsed.guidPrefix == GUIDPREFIX_UNKNOWN) {
        parsed.guidPrefix = sender_prefix;
    }

    // Compute lease expiry in ms.
    // §8.5.3.2: "If a participant does not receive a refresh within
    //  leaseDuration, the remote participant is considered gone."
    const uint32_t lease_ms =
        static_cast<uint32_t>(parsed.leaseDuration.seconds) * 1000u +
        parsed.leaseDuration.nanoseconds / 1'000'000u;

    PeerEntry* existing = find_peer(parsed.guidPrefix);
    if (existing) {
        // Refresh existing participant data and lease.
        // If SEDP was previously deferred due to missing locator, trigger it
        // once locator information arrives.
        //
        // The record is overwritten in place, never moved to another slot:
        // matched proxies refer to its locator lists (see "Locator reference"
        // in rtps/reader_proxy.hpp), so a peer that re-announces from a new
        // address is reached there from the next send on.
        existing->data = DiscoveredParticipant::from(parsed, *subnets_);
        existing->leaseExpiryMs = now_ms + lease_ms;

        if (!existing->sedp_announced &&
            existing->data.metatrafficUnicastLocatorCount > 0u &&
            on_new_peer_) {
            on_new_peer_(*existing, user_data_);
        }

        RTPS_LOG_D("spdp", "lease refreshed for %02x%02x%02x%02x (expires in %lums)",
                   parsed.guidPrefix.value[0], parsed.guidPrefix.value[1],
                   parsed.guidPrefix.value[2], parsed.guidPrefix.value[3],
                   (unsigned long)lease_ms);
        return true;
    }

    // New participant — insert into peer table.
    PeerEntry* slot = find_free();
    if (!slot) {
        RTPS_LOG_W("spdp", "peer table full, dropping %02x%02x%02x%02x",
                   parsed.guidPrefix.value[0], parsed.guidPrefix.value[1],
                   parsed.guidPrefix.value[2], parsed.guidPrefix.value[3]);
        return false;   // Table full; drop.
    }

    slot->data = DiscoveredParticipant::from(parsed, *subnets_);
    slot->alive          = true;
    slot->sedp_announced = false;
    slot->leaseExpiryMs  = now_ms + lease_ms;

    RTPS_LOG_I("spdp", "new participant %02x%02x%02x%02x%02x%02x"
               " lease=%lums",
               parsed.guidPrefix.value[0], parsed.guidPrefix.value[1],
               parsed.guidPrefix.value[2], parsed.guidPrefix.value[3],
               parsed.guidPrefix.value[4], parsed.guidPrefix.value[5],
               (unsigned long)lease_ms);

    if (on_new_peer_) on_new_peer_(*slot, user_data_);
    return true;
}

// ----------------------------------------------------------
// SpdpReader::tick()
//
// §8.5.3.2, p. 160 — lease expiry enforcement.
// ----------------------------------------------------------

uint32_t SpdpReader::ms_until_next_expiry(uint32_t now_ms) const noexcept {
    uint32_t next = UINT32_MAX;
    for (uint8_t i = 0; i < RTPS_MAX_PEERS; ++i) {
        if (!peers_[i].alive) continue;
        // Same wrap-safe "already past" test tick() applies.
        const uint32_t left = peers_[i].leaseExpiryMs - now_ms;
        const uint32_t wait = (left > 0x80000000u) ? 0u : left;
        if (wait < next) next = wait;
    }
    return next;
}

bool SpdpReader::remove_peer(const GuidPrefix& prefix) noexcept {
    for (uint8_t i = 0; i < RTPS_MAX_PEERS; ++i) {
        if (!peers_[i].alive || peers_[i].data.guidPrefix != prefix) continue;
        RTPS_LOG_I("spdp", "participant disposed: %02x%02x%02x%02x%02x%02x",
                   prefix.value[0], prefix.value[1], prefix.value[2],
                   prefix.value[3], prefix.value[4], prefix.value[5]);
        // Unmatch before freeing the slot: proxies refer to this record's
        // locators until on_peer_removed_ has removed them.
        if (on_peer_removed_) on_peer_removed_(peers_[i], user_data_);
        peers_[i].alive = false;
        return true;
    }
    return false;
}

void SpdpReader::tick(uint32_t now_ms) noexcept {
    for (uint8_t i = 0; i < RTPS_MAX_PEERS; ++i) {
        if (!peers_[i].alive) continue;

        // Wrap-safe: if leaseExpiryMs is in the past, the subtraction
        // overflows to a large positive — check the sign via comparison.
        const bool expired =
            (now_ms >= peers_[i].leaseExpiryMs) ||
            // Also handle the uint32 wrap case: if expiry is very far
            // from now, the peer was refreshed after a wrap.
            ((peers_[i].leaseExpiryMs - now_ms) > 0x80000000u);

        if (expired) {
            RTPS_LOG_I("spdp", "participant lease expired: %02x%02x%02x%02x%02x%02x",
                       peers_[i].data.guidPrefix.value[0],
                       peers_[i].data.guidPrefix.value[1],
                       peers_[i].data.guidPrefix.value[2],
                       peers_[i].data.guidPrefix.value[3],
                       peers_[i].data.guidPrefix.value[4],
                       peers_[i].data.guidPrefix.value[5]);
            // As in remove_peer(): unmatch before the slot is freed.
            if (on_peer_removed_) on_peer_removed_(peers_[i], user_data_);
            peers_[i].alive = false;
        }
    }
}

} // namespace rtps
