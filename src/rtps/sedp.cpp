/**
 * @file rtps/sedp.cpp
 * @brief SEDP implementation — §8.5.4 (OMG formal/2022-04-01).
 */

#include "sedp.hpp"

#include "gap_encoding.hpp"
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

// ReliabilityQosPolicy on the wire is {kind, max_blocking_time}: a uint32
// kind followed by a Duration_t (int32 seconds, uint32 fraction), 12 bytes
// in total (DDSI-RTPS 2.5 §9.6.3.2, Table 9.13; DDS 1.4 §2.2.3.14). Stock
// implementations reject any other length and discard the whole endpoint
// announcement with it.
static constexpr uint16_t RELIABILITY_POLICY_LEN = 12u;
static constexpr uint16_t DURABILITY_POLICY_LEN  = 4u;

/// ReliabilityQosPolicy as it appears in a little-endian parameter list.
struct ReliabilityWire {
    uint32_t kind;
    int32_t  max_blocking_seconds;
    uint32_t max_blocking_fraction;
};
static_assert(sizeof(ReliabilityWire) == RELIABILITY_POLICY_LEN,
              "ReliabilityWire must match the PID_RELIABILITY wire length");

/// Encodes a reliability kind with the DDS default max_blocking_time of
/// 100 ms (fraction = 0.1 * 2^32), which is what stock implementations
/// announce when the application does not override it.
static ReliabilityWire reliability_wire(ReliabilityKind kind) noexcept {
    return ReliabilityWire{static_cast<uint32_t>(kind), 0, 0x1999999Au};
}

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

// Removes the oldest disposal from a builtin writer's history when it is
// full, so the writer's own KEEP_LAST eviction never drops a live
// announcement. Live announcements never exceed kSedpHistoryDepth - 1, so a
// full history always holds at least one disposal.
static void make_room_for_change(SedpWriter::BuiltinWriter& writer) noexcept
{
    if (writer.cache().size() < SedpWriter::kSedpHistoryDepth) return;
    SequenceNumber oldest_disposal = SEQUENCENUMBER_UNKNOWN;
    writer.cache().for_each_change([&](const CacheChange& c) noexcept {
        if (c.kind == ChangeKind::ALIVE) return;
        if (oldest_disposal == SEQUENCENUMBER_UNKNOWN || c.sequenceNumber < oldest_disposal)
            oldest_disposal = c.sequenceNumber;
    });
    if (oldest_disposal != SEQUENCENUMBER_UNKNOWN) writer.remove_change(oldest_disposal);
}

// Serialises one endpoint and writes it as a live announcement keyed by
// the endpoint GUID. Returns the change's SN, or SEQUENCENUMBER_UNKNOWN.
template<typename EndpointData, typename SerialiseFn>
static SequenceNumber write_announcement(SedpWriter::BuiltinWriter& writer,
                                         const EndpointData&        endpoint,
                                         const Guid&                endpoint_guid,
                                         uint32_t                   size_estimate,
                                         SerialiseFn                serialise) noexcept
{
    BufRef payload = buf_alloc(size_estimate);
    if (!payload.valid()) return SEQUENCENUMBER_UNKNOWN;
    const uint32_t plen = serialise(endpoint, payload.data, payload.capacity());
    if (plen == 0) {
        payload.reset();
        return SEQUENCENUMBER_UNKNOWN;
    }

    InlineQos key{};
    key.set_key_hash(endpoint_guid);
    make_room_for_change(writer);
    SequenceNumber sn = SEQUENCENUMBER_UNKNOWN;
    const bool ok = writer.write_preframed_async(ChangeKind::ALIVE, payload.data, plen,
                                                 &sn, key);
    payload.reset();
    return ok ? sn : SEQUENCENUMBER_UNKNOWN;
}

// Withdraws an endpoint: drops its live announcement and writes a disposal
// whose serialized key is PID_ENDPOINT_GUID (§9.6.3.2) and whose inline QoS
// carries PID_KEY_HASH and PID_STATUS_INFO = DISPOSED|UNREGISTERED.
static bool write_disposal(SedpWriter::BuiltinWriter& writer,
                           const Guid&                endpoint_guid,
                           const SequenceNumber&      alive_sn) noexcept
{
    if (alive_sn != SEQUENCENUMBER_UNKNOWN) writer.remove_change(alive_sn);

    Octet key_payload[4u + 4u + 16u + 4u] = {0x00u, 0x03u, 0x00u, 0x00u};  // PL_CDR_LE
    uint32_t off = 4u;
    Octet g[16] = {};
    memcpy(g,      endpoint_guid.prefix.value, 12u);
    memcpy(g + 12, &endpoint_guid.entityId,     4u);
    if (!plcdr_write_param(key_payload, sizeof(key_payload), off, pid::ENDPOINT_GUID, g, 16u))
        return false;
    if (!plcdr_write_sentinel(key_payload, sizeof(key_payload), off)) return false;

    InlineQos qos{};
    qos.set_key_hash(endpoint_guid);
    qos.hasStatusInfo = true;
    qos.statusInfo    = status_info::DISPOSED | status_info::UNREGISTERED;
    make_room_for_change(writer);
    return writer.write_preframed_async(ChangeKind::NOT_ALIVE_DISPOSED, key_payload, off,
                                        nullptr, qos);
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
//   PID_RELIABILITY      (0x001A) — 12 bytes (kind + max_blocking_time)
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

    // PID_RELIABILITY — kind + max_blocking_time (12 bytes).
    {
        const ReliabilityWire v = reliability_wire(wd.reliabilityKind);
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
        const ReliabilityWire v = reliability_wire(rd.reliabilityKind);
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
// SedpWriter::register_*() / unregister_*()
// ============================================================

bool SedpWriter::register_writer(const DiscoveredWriterData& wd) noexcept
{
    for (uint8_t i = 0; i < RTPS_MAX_LOCAL_ENDPOINTS; ++i) {
        if (local_writers_[i].is_valid()) continue;
        local_writers_[i] = wd;
        local_writer_sn_[i] = write_announcement(
            publications_writer_, wd, wd.writerGuid, writer_payload_size_estimate(wd),
            [this](const DiscoveredWriterData& d, Octet* buf, uint32_t cap) noexcept {
                return serialise_writer(d, buf, cap);
            });
        if (local_writer_sn_[i] == SEQUENCENUMBER_UNKNOWN) {
            local_writers_[i] = DiscoveredWriterData{};
            return false;
        }
        return true;
    }
    RTPS_LOG_E("sedp", "local writer table full (RTPS_MAX_LOCAL_ENDPOINTS=%d)",
               RTPS_MAX_LOCAL_ENDPOINTS);
    return false;
}

bool SedpWriter::register_reader(const DiscoveredReaderData& rd) noexcept
{
    for (uint8_t i = 0; i < RTPS_MAX_LOCAL_ENDPOINTS; ++i) {
        if (local_readers_[i].is_valid()) continue;
        local_readers_[i] = rd;
        local_reader_sn_[i] = write_announcement(
            subscriptions_writer_, rd, rd.readerGuid, reader_payload_size_estimate(rd),
            [this](const DiscoveredReaderData& d, Octet* buf, uint32_t cap) noexcept {
                return serialise_reader(d, buf, cap);
            });
        if (local_reader_sn_[i] == SEQUENCENUMBER_UNKNOWN) {
            local_readers_[i] = DiscoveredReaderData{};
            return false;
        }
        return true;
    }
    RTPS_LOG_E("sedp", "local reader table full (RTPS_MAX_LOCAL_ENDPOINTS=%d)",
               RTPS_MAX_LOCAL_ENDPOINTS);
    return false;
}

bool SedpWriter::unregister_writer(const Guid& writer_guid) noexcept
{
    for (uint8_t i = 0; i < RTPS_MAX_LOCAL_ENDPOINTS; ++i) {
        if (!local_writers_[i].is_valid() || local_writers_[i].writerGuid != writer_guid)
            continue;
        const bool ok = write_disposal(publications_writer_, writer_guid, local_writer_sn_[i]);
        local_writers_[i] = DiscoveredWriterData{};
        local_writer_sn_[i] = SEQUENCENUMBER_UNKNOWN;
        return ok;
    }
    return false;
}

bool SedpWriter::unregister_reader(const Guid& reader_guid) noexcept
{
    for (uint8_t i = 0; i < RTPS_MAX_LOCAL_ENDPOINTS; ++i) {
        if (!local_readers_[i].is_valid() || local_readers_[i].readerGuid != reader_guid)
            continue;
        const bool ok = write_disposal(subscriptions_writer_, reader_guid, local_reader_sn_[i]);
        local_readers_[i] = DiscoveredReaderData{};
        local_reader_sn_[i] = SEQUENCENUMBER_UNKNOWN;
        return ok;
    }
    return false;
}

// ============================================================
// SedpWriter::announce_to_peer()
//
// §8.5.4, p. 163 — match the peer's builtin readers; TRANSIENT_LOCAL
// replay (§8.7.2.2) delivers every live announcement to them.
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
                    Locator locator{};
                    if (Locator::read_from(locator,
                                           val,
                                           24u,
                                           little_endian) &&
                        (locator.kind == locator_kind::UDPv4 ||
                         locator.kind == locator_kind::UDPv6)) {
                        unicast_locs[unicast_count] = locator;
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
    if (!accept_announcement(sender_prefix, /*is_pub=*/true, writer_sn)) return;
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
    if (!accept_announcement(sender_prefix, /*is_pub=*/false, writer_sn)) return;
    DiscoveredReaderData rd{};
    if (!deserialise_reader(payload, payload_len, rd)) return;
    if (!rd.is_valid()) return;
    match_table_.add_reader(rd);
}

// ============================================================
// SedpReader::on_publication_disposed() / on_subscription_disposed()
//
// §8.5.4.1 / §9.6.3.3: a DATA with StatusInfo DISPOSED or UNREGISTERED
// withdraws the endpoint named by its key. Removing it from the match
// table fires on_unmatch for every local endpoint it was matched with.
// ============================================================

void SedpReader::on_publication_disposed(const GuidPrefix& sender_prefix,
                                         const Guid&       endpoint_guid,
                                         SequenceNumber    writer_sn) noexcept
{
    if (!accept_announcement(sender_prefix, /*is_pub=*/true, writer_sn)) return;
    // Only the owning participant may withdraw its endpoints.
    if (endpoint_guid.prefix != sender_prefix) return;
    match_table_.remove_writer(endpoint_guid);
}

void SedpReader::on_subscription_disposed(const GuidPrefix& sender_prefix,
                                          const Guid&       endpoint_guid,
                                          SequenceNumber    writer_sn) noexcept
{
    if (!accept_announcement(sender_prefix, /*is_pub=*/false, writer_sn)) return;
    if (endpoint_guid.prefix != sender_prefix) return;
    match_table_.remove_reader(endpoint_guid);
}

// ============================================================
// SedpReader reliability: HEARTBEAT, GAP, repairs.
//
// §8.4.12.1: the reader answers a HEARTBEAT with the set of changes it is
// missing; §8.3.7.4: a GAP resolves changes the writer will never send.
// ============================================================

bool SedpReader::accept_announcement(const GuidPrefix&     sender_prefix,
                                     bool                  is_pub,
                                     const SequenceNumber& writer_sn) noexcept
{
    SedpBuiltinAckState* state = find_or_alloc_ack_state(sender_prefix);
    if (!state) return true;
    SedpWriterState& writer = is_pub ? state->pub : state->sub;
    // An announcement beyond the window cannot be recorded; it is processed
    // anyway (discovery data is idempotent) and requested again later.
    const bool is_new = writer.window.covers(writer_sn) ? writer.window.resolve(writer_sn)
                                                        : !writer.window.is_resolved(writer_sn);
    if (!is_new) return false;
    if (writer_sn > writer.last_sn) writer.last_sn = writer_sn;
    if (writer_sn == writer.pending_repair && repairs_must_fit_datagram(transport_)) {
        send_acknack(*state, is_pub);
    }
    return true;
}

void SedpReader::send_acknack(SedpBuiltinAckState& state, bool is_pub) noexcept
{
    if (state.writer_locator == LOCATOR_INVALID) return;
    SedpWriterState& writer = is_pub ? state.pub : state.sub;

    const bool narrow = repairs_must_fit_datagram(transport_);
    SequenceNumberSet sn_state{};
    writer.window.unresolved(writer.last_sn, narrow ? 1u : 256u, sn_state);
    // bitmapBase is the window's lowest unresolved number, so a narrow
    // request names exactly that change.
    writer.pending_repair = (narrow && !sn_state.empty()) ? sn_state.bitmapBase
                                                          : SequenceNumber{};
    const bool final_flag = sn_state.empty();
    ++writer.acknack_count;

    const EntityId reader_id = is_pub ? entity_id::SEDP_BUILTIN_PUBLICATIONS_DETECTOR
                                      : entity_id::SEDP_BUILTIN_SUBSCRIPTIONS_DETECTOR;
    const EntityId writer_id = is_pub ? entity_id::SEDP_BUILTIN_PUBLICATIONS_ANNOUNCER
                                      : entity_id::SEDP_BUILTIN_SUBSCRIPTIONS_ANNOUNCER;

    RTPS_LOG_D("sedp", "ACKNACK base=%d:%lu bits=%lu final=%d",
               sn_state.bitmapBase.high, (unsigned long)sn_state.bitmapBase.low,
               (unsigned long)sn_state.numBits, (int)final_flag);

    if (!builder_.begin(local_prefix_)) return;
    builder_.append_info_dst(state.prefix);
    const bool ok = builder_.append_acknack(
        reader_id, writer_id, sn_state,
        static_cast<Long>(writer.acknack_count), final_flag);
    if (ok) {
        BufRef buf = builder_.finish();
        transport_.send_unicast(buf, state.writer_locator);
        buf.reset();
    } else {
        RTPS_LOG_W("sedp", "SEDP ACKNACK builder failed (pool exhausted?)");
        builder_.abort();
    }
}

void SedpReader::on_heartbeat(const ParseContext&    ctx,
                              const ParsedHeartbeat& msg,
                              const Locator&         writer_locator) noexcept
{
    if (writer_locator == LOCATOR_INVALID) return;

    bool is_pub;
    if (msg.writerId == entity_id::SEDP_BUILTIN_PUBLICATIONS_ANNOUNCER) {
        is_pub = true;
    } else if (msg.writerId == entity_id::SEDP_BUILTIN_SUBSCRIPTIONS_ANNOUNCER) {
        is_pub = false;
    } else {
        return;
    }

    SedpBuiltinAckState* state = find_or_alloc_ack_state(ctx.senderGuidPrefix);
    if (!state) return;
    state->writer_locator = writer_locator;
    SedpWriterState& writer = is_pub ? state->pub : state->sub;
    if (msg.lastSN > writer.last_sn) writer.last_sn = msg.lastSN;
    writer.window.resolve_below(msg.firstSN);
    send_acknack(*state, is_pub);
}

void SedpReader::on_gap(const ParseContext& ctx, const ParsedGap& msg) noexcept
{
    bool is_pub;
    if (msg.writerId == entity_id::SEDP_BUILTIN_PUBLICATIONS_ANNOUNCER) {
        is_pub = true;
    } else if (msg.writerId == entity_id::SEDP_BUILTIN_SUBSCRIPTIONS_ANNOUNCER) {
        is_pub = false;
    } else {
        return;
    }
    SedpBuiltinAckState* state = find_or_alloc_ack_state(ctx.senderGuidPrefix);
    if (!state) return;
    SedpWriterState& writer = is_pub ? state->pub : state->sub;

    writer.window.resolve_range(msg.gapStart, gap_run_end(msg.gapStart, msg.gapList));
    msg.gapList.for_each([&](const SequenceNumber& sn) noexcept {
        writer.window.resolve(sn);
    });
    if (repairs_must_fit_datagram(transport_) && writer.pending_repair != SequenceNumber{} &&
        writer.window.is_resolved(writer.pending_repair)) {
        send_acknack(*state, is_pub);
    }
}

void SedpReader::remove_peer(const GuidPrefix& prefix) noexcept
{
    for (uint8_t i = 0; i < RTPS_MAX_PEERS; ++i) {
        if (ack_states_[i].alive && ack_states_[i].prefix == prefix) {
            ack_states_[i] = SedpBuiltinAckState{};
        }
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
