/**
 * @file rtps/sedp.cpp
 * @brief SEDP implementation — §8.5.4 (OMG formal/2022-04-01).
 */

#include "sedp.hpp"
#include "log.hpp"
#include "stateful_writer.hpp"   // ITransport
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

inline uint32_t read_u32_wire(const uint8_t* p, bool little_endian) noexcept
{
    if (little_endian) {
        return static_cast<uint32_t>(p[0]) |
               (static_cast<uint32_t>(p[1]) << 8u) |
               (static_cast<uint32_t>(p[2]) << 16u) |
               (static_cast<uint32_t>(p[3]) << 24u);
    }
    return (static_cast<uint32_t>(p[0]) << 24u) |
           (static_cast<uint32_t>(p[1]) << 16u) |
           (static_cast<uint32_t>(p[2]) << 8u) |
           static_cast<uint32_t>(p[3]);
}

} // namespace

// ============================================================
// Reliability / Durability wire values
//
// §9.6.3.2 Table 9.15 uses the DDS QoS policy encoding.
// Reliability: BEST_EFFORT=1, RELIABLE=2 (as per ReliabilityKind).
// Durability:  VOLATILE=0, TRANSIENT_LOCAL=1 (as per DurabilityKind).
// Each is preceded by a 4-byte kind field (from DDS IDL union
// discriminant) per the PL_CDR encoding in §9.6.3.2, p. 265.
// ============================================================

static constexpr uint16_t RELIABILITY_POLICY_LEN = 4u;
static constexpr uint16_t DURABILITY_POLICY_LEN  = 4u;

static uint32_t plcdr_param_size_estimate(uint32_t value_len) noexcept
{
    return 4u + ((value_len + 3u) & ~3u);
}

static uint32_t plcdr_string_size_estimate(const char* str) noexcept
{
    uint32_t str_len = 0;
    while (str && str[str_len]) ++str_len;
    ++str_len;   // Include null terminator.
    return plcdr_param_size_estimate(4u + str_len);
}

static uint32_t writer_payload_size_estimate(const DiscoveredWriterData& wd) noexcept
{
    return 4u +                                   // RepresentationHeader
           plcdr_param_size_estimate(16u) +       // PID_ENDPOINT_GUID
           plcdr_string_size_estimate(wd.topicName.str) +
           plcdr_string_size_estimate(wd.typeName.str) +
           plcdr_param_size_estimate(RELIABILITY_POLICY_LEN) +
           plcdr_param_size_estimate(DURABILITY_POLICY_LEN) +
           static_cast<uint32_t>(wd.unicastLocatorCount) *
               plcdr_param_size_estimate(24u) +
           4u;                                    // PID_SENTINEL
}

static uint32_t reader_payload_size_estimate(const DiscoveredReaderData& rd) noexcept
{
    return 4u +                                   // RepresentationHeader
           plcdr_param_size_estimate(16u) +       // PID_ENDPOINT_GUID
           plcdr_string_size_estimate(rd.topicName.str) +
           plcdr_string_size_estimate(rd.typeName.str) +
           plcdr_param_size_estimate(RELIABILITY_POLICY_LEN) +
           plcdr_param_size_estimate(DURABILITY_POLICY_LEN) +
           static_cast<uint32_t>(rd.unicastLocatorCount) *
               plcdr_param_size_estimate(24u) +
           4u;                                    // PID_SENTINEL
}

template<typename EndpointData, typename EstimateFn, typename SerialiseFn>
static void announce_endpoints(StatefulWriter<RTPS_MAX_LOCAL_ENDPOINTS, RTPS_MAX_PEERS>& writer,
                               const EndpointData* endpoints,
                               EstimateFn          estimate,
                               SerialiseFn         serialise) noexcept
{
    for (uint8_t i = 0; i < RTPS_MAX_LOCAL_ENDPOINTS; ++i) {
        if (!endpoints[i].is_valid()) continue;

        BufRef payload = buf_alloc(estimate(endpoints[i]));
        if (!payload.valid()) continue;

        const uint32_t plen = serialise(endpoints[i], payload.data, payload.capacity());
        if (plen == 0) {
            payload.reset();
            continue;
        }
        payload.size = plen;
        writer.write_preframed_async(ChangeKind::ALIVE, payload.data, payload.size);
        payload.reset();
    }
}

// ============================================================
// SedpWriter::serialise_writer()
//
// Produces PL_CDR_LE for one DiscoveredWriterData.
//
// §9.6.3.2, Table 9.15, p. 263 — mandatory PIDs for publications:
//   PID_ENDPOINT_GUID    (0x005A) — 16 bytes
//   PID_TOPIC_NAME       (0x0005) — string
//   PID_TYPE_NAME        (0x0007) — string
//   PID_RELIABILITY      (0x001A) — 4 bytes (kind enum)
//   PID_DURABILITY       (0x001D) — 4 bytes (kind enum)
//   PID_UNICAST_LOCATOR  (0x002F) — 24 bytes, repeated
//   PID_SENTINEL         (0x0001)
// ============================================================

uint32_t SedpWriter::serialise_writer(const DiscoveredWriterData& wd,
                                      Octet* buf, uint32_t cap) const noexcept
{
    uint32_t off = 0;

    // RepresentationHeader — PL_CDR_LE.
    if (off + 4u > cap) return 0;
    buf[off++] = 0x00u;
    buf[off++] = 0x03u;
    buf[off++] = 0x00u;
    buf[off++] = 0x00u;

    // PID_ENDPOINT_GUID — GuidPrefix(12) + EntityId(4).
    {
        Octet g[16] = {};
        memcpy(g,      wd.writerGuid.prefix.value, 12u);
        memcpy(g + 12, &wd.writerGuid.entityId,     4u);
        if (!plcdr_write_param(buf, cap, off, pid::ENDPOINT_GUID, g, 16u))
            return 0;
    }

    // PID_TOPIC_NAME — string.
    if (!plcdr_write_string(buf, cap, off, pid::TOPIC_NAME,
                            wd.topicName.str)) return 0;

    // PID_TYPE_NAME — string.
    if (!plcdr_write_string(buf, cap, off, pid::TYPE_NAME,
                            wd.typeName.str)) return 0;

    // PID_RELIABILITY — 4 bytes (uint32 kind).
    {
        uint32_t v = static_cast<uint32_t>(wd.reliabilityKind);
        if (!plcdr_write_param(buf, cap, off, pid::RELIABILITY,
                               &v, RELIABILITY_POLICY_LEN)) return 0;
    }

    // PID_DURABILITY — 4 bytes.
    {
        uint32_t v = static_cast<uint32_t>(wd.durabilityKind);
        if (!plcdr_write_param(buf, cap, off, pid::DURABILITY,
                               &v, DURABILITY_POLICY_LEN)) return 0;
    }

    // PID_UNICAST_LOCATOR — 24 bytes each.
    for (uint8_t i = 0; i < wd.unicastLocatorCount; ++i) {
        if (!plcdr_write_param(buf, cap, off, pid::UNICAST_LOCATOR,
                               &wd.unicastLocatorList[i], 24u)) return 0;
    }

    if (!plcdr_write_sentinel(buf, cap, off)) return 0;
    return off;
}

// ============================================================
// SedpWriter::serialise_reader()
//
// §9.6.3.2, Table 9.15, p. 263 — subscriptions PIDs:
//   PID_ENDPOINT_GUID, PID_TOPIC_NAME, PID_TYPE_NAME,
//   PID_RELIABILITY, PID_DURABILITY, PID_UNICAST_LOCATOR,
//   PID_SENTINEL.
// ============================================================

uint32_t SedpWriter::serialise_reader(const DiscoveredReaderData& rd,
                                      Octet* buf, uint32_t cap) const noexcept
{
    uint32_t off = 0;

    if (off + 4u > cap) return 0;
    buf[off++] = 0x00u;
    buf[off++] = 0x03u;
    buf[off++] = 0x00u;
    buf[off++] = 0x00u;

    {
        Octet g[16] = {};
        memcpy(g,      rd.readerGuid.prefix.value, 12u);
        memcpy(g + 12, &rd.readerGuid.entityId,     4u);
        if (!plcdr_write_param(buf, cap, off, pid::ENDPOINT_GUID, g, 16u))
            return 0;
    }

    if (!plcdr_write_string(buf, cap, off, pid::TOPIC_NAME, rd.topicName.str))
        return 0;
    if (!plcdr_write_string(buf, cap, off, pid::TYPE_NAME,  rd.typeName.str))
        return 0;

    {
        uint32_t v = static_cast<uint32_t>(rd.reliabilityKind);
        if (!plcdr_write_param(buf, cap, off, pid::RELIABILITY, &v,
                               RELIABILITY_POLICY_LEN)) return 0;
    }
    {
        uint32_t v = static_cast<uint32_t>(rd.durabilityKind);
        if (!plcdr_write_param(buf, cap, off, pid::DURABILITY, &v,
                               DURABILITY_POLICY_LEN)) return 0;
    }
    for (uint8_t i = 0; i < rd.unicastLocatorCount; ++i) {
        if (!plcdr_write_param(buf, cap, off, pid::UNICAST_LOCATOR,
                               &rd.unicastLocatorList[i], 24u)) return 0;
    }

    if (!plcdr_write_sentinel(buf, cap, off)) return 0;
    return off;
}

// ============================================================
// SedpWriter::announce_to_peer()
//
// §8.5.4, p. 163 — send all local writer and reader announcements
// to the specified destination (remote participant metatraffic locator).
// ============================================================

void SedpWriter::announce_to_peer(const GuidPrefix& peer_prefix,
                                  const Locator&    dest) noexcept
{
    const Guid pub_reader{peer_prefix, entity_id::SEDP_BUILTIN_PUBLICATIONS_DETECTOR};
    const Guid sub_reader{peer_prefix, entity_id::SEDP_BUILTIN_SUBSCRIPTIONS_DETECTOR};
    if (!publications_writer_.matched_reader_lookup(pub_reader)) {
        publications_writer_.matched_reader_add(pub_reader, &dest, 1u, false);
    }
    if (!subscriptions_writer_.matched_reader_lookup(sub_reader)) {
        subscriptions_writer_.matched_reader_add(sub_reader, &dest, 1u, false);
    }

    announce_endpoints(
        publications_writer_,
        local_writers_,
        writer_payload_size_estimate,
        [this](const DiscoveredWriterData& wd,
               Octet* buf,
               uint32_t cap) noexcept {
            return serialise_writer(wd, buf, cap);
        });

    announce_endpoints(
        subscriptions_writer_,
        local_readers_,
        reader_payload_size_estimate,
        [this](const DiscoveredReaderData& rd,
               Octet* buf,
               uint32_t cap) noexcept {
            return serialise_reader(rd, buf, cap);
        });
}

void SedpWriter::on_peer_discovered(const SPDPdiscoveredParticipantData& peer) noexcept
{
    const Locator* locs = peer.metatrafficUnicastLocatorList;
    uint8_t count = peer.metatrafficUnicastLocatorCount;
    if (count == 0u) {
        locs = peer.metatrafficMulticastLocatorList;
        count = peer.metatrafficMulticastLocatorCount;
    }
    if (count == 0u) {
        return;
    }

    const Guid pub_reader{peer.guidPrefix, entity_id::SEDP_BUILTIN_PUBLICATIONS_DETECTOR};
    const Guid sub_reader{peer.guidPrefix, entity_id::SEDP_BUILTIN_SUBSCRIPTIONS_DETECTOR};

    const bool has_pub_detector =
        (peer.availableBuiltinEndpoints &
         builtin_endpoint::DISC_BUILTIN_ENDPOINT_PUBLICATIONS_DETECTOR) != 0u;
    const bool has_sub_detector =
        (peer.availableBuiltinEndpoints &
         builtin_endpoint::DISC_BUILTIN_ENDPOINT_SUBSCRIPTIONS_DETECTOR) != 0u;

    if (has_pub_detector) {
        if (!publications_writer_.matched_reader_lookup(pub_reader)) {
            publications_writer_.matched_reader_add(pub_reader, locs, count, false);
        }
    } else {
        publications_writer_.matched_reader_remove(pub_reader);
    }

    if (has_sub_detector) {
        if (!subscriptions_writer_.matched_reader_lookup(sub_reader)) {
            subscriptions_writer_.matched_reader_add(sub_reader, locs, count, false);
        }
    } else {
        subscriptions_writer_.matched_reader_remove(sub_reader);
    }
}

void SedpWriter::on_peer_removed(const GuidPrefix& peer_prefix) noexcept
{
    publications_writer_.matched_reader_remove(
        Guid{peer_prefix, entity_id::SEDP_BUILTIN_PUBLICATIONS_DETECTOR});
    subscriptions_writer_.matched_reader_remove(
        Guid{peer_prefix, entity_id::SEDP_BUILTIN_SUBSCRIPTIONS_DETECTOR});
}

IStatefulWriter* SedpWriter::find_writer_iface(const EntityId& writer_id) noexcept
{
    if (writer_id == entity_id::SEDP_BUILTIN_PUBLICATIONS_ANNOUNCER) {
        return &publications_writer_;
    }
    if (writer_id == entity_id::SEDP_BUILTIN_SUBSCRIPTIONS_ANNOUNCER) {
        return &subscriptions_writer_;
    }
    return nullptr;
}

void SedpWriter::tick(uint32_t now_ms) noexcept
{
    publications_writer_.tick(now_ms);
    subscriptions_writer_.tick(now_ms);
}

// ============================================================
// SedpReader — shared PL_CDR field reader
//
// Reads the fields common to both publications and subscriptions.
// §9.6.3.2, Table 9.15, p. 263.
// ============================================================

bool SedpReader::read_plcdr_fields(const Octet*  payload,
                                   uint32_t      len,
                                   TopicName&    topic_out,
                                   TopicName&    type_out,
                                   Guid&         endpoint_guid_out,
                                   ReliabilityKind& reliability_out,
                                   DurabilityKind&  durability_out,
                                   Locator*      unicast_locs,
                                   uint8_t&      unicast_count) noexcept
{
    if (len < 4u) return false;

    bool little_endian = true;
    if (!decode_plcdr_rep_id(payload, len, little_endian)) {
        return false;
    }

    uint32_t off      = 4u;
    bool     got_guid = false;
    unicast_count     = 0;

    while (off + 4u <= len) {
        const uint16_t pid_val = read_u16_wire(payload + off, little_endian);
        const uint16_t param_len = read_u16_wire(payload + off + 2u, little_endian);
        off += 4u;

        const uint16_t padded = static_cast<uint16_t>((param_len + 3u) & ~3u);
        if (pid_val == pid::SENTINEL) break;
        if (off + padded > len) break;

        const Octet* val = payload + off;

        switch (pid_val) {
            case pid::ENDPOINT_GUID:
                if (param_len >= 16u) {
                    memcpy(endpoint_guid_out.prefix.value, val, 12u);
                    memcpy(&endpoint_guid_out.entityId, val + 12, 4u);
                    got_guid = true;
                }
                break;

            case pid::TOPIC_NAME:
                if (param_len >= 5u) {
                    // String: uint32 len prefix + chars.
                    const uint32_t slen = read_u32_wire(val, little_endian);
                    if (slen > 0 && slen <= param_len - 4u) {
                        uint32_t copy = slen < RTPS_MAX_TOPIC_NAME_LEN
                                        ? slen : RTPS_MAX_TOPIC_NAME_LEN - 1u;
                        memcpy(topic_out.str, val + 4, copy);
                        topic_out.str[copy] = '\0';
                    }
                }
                break;

            case pid::TYPE_NAME:
                if (param_len >= 5u) {
                    const uint32_t slen = read_u32_wire(val, little_endian);
                    if (slen > 0 && slen <= param_len - 4u) {
                        uint32_t copy = slen < RTPS_MAX_TOPIC_NAME_LEN
                                        ? slen : RTPS_MAX_TOPIC_NAME_LEN - 1u;
                        memcpy(type_out.str, val + 4, copy);
                        type_out.str[copy] = '\0';
                    }
                }
                break;

            case pid::RELIABILITY:
                if (param_len >= 4u) {
                    const uint32_t v = read_u32_wire(val, little_endian);
                    reliability_out = static_cast<ReliabilityKind>(v);
                }
                break;

            case pid::DURABILITY:
                if (param_len >= 4u) {
                    const uint32_t v = read_u32_wire(val, little_endian);
                    durability_out = static_cast<DurabilityKind>(v);
                }
                break;

            case pid::UNICAST_LOCATOR:
                if (param_len >= 24u &&
                    unicast_count < RTPS_MAX_LOCATORS_PER_PARTICIPANT) {
                    if (Locator::read_from(unicast_locs[unicast_count],
                                           val,
                                           24u,
                                           little_endian)) {
                        ++unicast_count;
                    }
                }
                break;

            default:
                break;
        }
        off += padded;
    }
    return got_guid;
}

// ============================================================
// SedpReader::on_publication_data()
//
// §8.5.4, p. 163:
//   "The SEDPbuiltinPublicationsReader processes DiscoveredWriterData."
// ============================================================

void SedpReader::on_publication_data(const GuidPrefix& sender_prefix,
                                     const Octet*      payload,
                                     uint32_t          payload_len,
                                     SequenceNumber    writer_sn) noexcept
{
    SedpBuiltinAckState* state = find_or_alloc_ack_state(sender_prefix);
    if (state && writer_sn > state->pub_highest) {
        state->pub_highest = writer_sn;
    }
    DiscoveredWriterData wd{};
    if (!deserialise_writer(payload, payload_len, wd)) return;
    if (!wd.is_valid()) return;
    match_table_.add_writer(wd);
}

bool SedpReader::deserialise_writer(const Octet*         payload,
                                    uint32_t             len,
                                    DiscoveredWriterData& out) noexcept
{
    Locator locs[RTPS_MAX_LOCATORS_PER_PARTICIPANT];
    uint8_t loc_count = 0;
    bool ok = read_plcdr_fields(payload, len,
                                out.topicName, out.typeName,
                                out.writerGuid,
                                out.reliabilityKind, out.durabilityKind,
                                locs, loc_count);
    if (!ok) return false;
    for (uint8_t i = 0; i < loc_count; ++i)
        out.unicastLocatorList[i] = locs[i];
    out.unicastLocatorCount = loc_count;
    return true;
}

// ============================================================
// SedpReader::on_subscription_data()
//
// §8.5.4, p. 163:
//   "The SEDPbuiltinSubscriptionsReader processes DiscoveredReaderData."
// ============================================================

void SedpReader::on_subscription_data(const GuidPrefix& sender_prefix,
                                      const Octet*      payload,
                                      uint32_t          payload_len,
                                      SequenceNumber    writer_sn) noexcept
{
    SedpBuiltinAckState* state = find_or_alloc_ack_state(sender_prefix);
    if (state && writer_sn > state->sub_highest) {
        state->sub_highest = writer_sn;
    }
    DiscoveredReaderData rd{};
    if (!deserialise_reader(payload, payload_len, rd)) return;
    if (!rd.is_valid()) return;
    match_table_.add_reader(rd);
}

// ============================================================
// SedpReader::on_heartbeat()
//
// Respond to a HEARTBEAT from a remote SEDP writer.
//
// §8.3.7.5, p. 91: F=0 → MUST send ACKNACK.
// §8.5.4.1, p. 165: SEDP uses RELIABLE / TRANSIENT_LOCAL.
//
// The ACKNACK bitmapBase is set to (highest_received + 1).
// Any SNs in [highest+1 .. lastSN] are marked missing so the
// remote writer retransmits them.  Once all have been received
// the next HEARTBEAT will get a pure ACK (numBits = 0).
// ============================================================

void SedpReader::on_heartbeat(const ParseContext&    ctx,
                               const ParsedHeartbeat& msg,
                               const Locator&         writer_locator) noexcept
{
    if (writer_locator == LOCATOR_INVALID) return;

    EntityId     reader_id;
    bool         is_pub;
    if (msg.writerId == entity_id::SEDP_BUILTIN_PUBLICATIONS_ANNOUNCER) {
        reader_id = entity_id::SEDP_BUILTIN_PUBLICATIONS_DETECTOR;
        is_pub    = true;
    } else if (msg.writerId == entity_id::SEDP_BUILTIN_SUBSCRIPTIONS_ANNOUNCER) {
        reader_id = entity_id::SEDP_BUILTIN_SUBSCRIPTIONS_DETECTOR;
        is_pub    = false;
    } else {
        return;
    }

    SedpBuiltinAckState* state = find_or_alloc_ack_state(ctx.senderGuidPrefix);
    if (!state) return;

    SequenceNumber& highest = is_pub ? state->pub_highest : state->sub_highest;
    uint32_t&       count   = is_pub ? state->pub_count   : state->sub_count;

    SequenceNumberSet sn_state{};
    const SequenceNumber next_want = highest + 1;

    if (next_want <= msg.lastSN) {
        // NACK all SNs from next_want through lastSN — §9.4.2.6, p. 193.
        sn_state.bitmapBase = next_want;
        const int64_t gap = msg.lastSN - next_want + 1;
        const uint32_t bits = (gap > 256) ? 256u : static_cast<uint32_t>(gap);
        for (uint32_t i = 0; i < bits; ++i) sn_state.set(i);
    } else {
        // Pure ACK: reader has everything the writer has advertised.
        sn_state.bitmapBase = highest + 1;
        sn_state.numBits    = 0;
    }

    ++count;

    RTPS_LOG_D("sedp", "HEARTBEAT→ACKNACK base=%d:%lu bits=%lu final=%d",
               sn_state.bitmapBase.high, (unsigned long)sn_state.bitmapBase.low,
               (unsigned long)sn_state.numBits, (int)(sn_state.numBits == 0));

    if (!builder_.begin(local_prefix_)) return;
    builder_.append_info_dst(ctx.senderGuidPrefix);
    const bool ok = builder_.append_acknack(
        reader_id, msg.writerId, sn_state,
        static_cast<Long>(count), /*final=*/true);
    if (ok) {
        BufRef buf = builder_.finish();
        transport_.send_unicast(buf, writer_locator);
        buf.reset();
    } else {
        RTPS_LOG_W("sedp", "SEDP ACKNACK builder failed (pool exhausted?)");
        builder_.abort();
    }
}

bool SedpReader::deserialise_reader(const Octet*         payload,
                                    uint32_t             len,
                                    DiscoveredReaderData& out) noexcept
{
    Locator locs[RTPS_MAX_LOCATORS_PER_PARTICIPANT];
    uint8_t loc_count = 0;
    bool ok = read_plcdr_fields(payload, len,
                                out.topicName, out.typeName,
                                out.readerGuid,
                                out.reliabilityKind, out.durabilityKind,
                                locs, loc_count);
    if (!ok) return false;
    for (uint8_t i = 0; i < loc_count; ++i)
        out.unicastLocatorList[i] = locs[i];
    out.unicastLocatorCount = loc_count;
    return true;
}

} // namespace rtps
