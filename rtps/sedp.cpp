/**
 * @file rtps/sedp.cpp
 * @brief SEDP implementation — §8.5.4 (OMG formal/2022-04-01).
 */

#include "sedp.hpp"
#include "stateful_writer.hpp"   // ITransport
#include <string.h>

namespace rtps {

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
static void announce_endpoints(MessageBuilder& builder,
                               ITransport&     transport,
                               const GuidPrefix& local_prefix,
                               const Locator&  dest,
                               const EndpointData* endpoints,
                               const EntityId& reader_id,
                               const EntityId& writer_id,
                               EstimateFn      estimate,
                               SerialiseFn     serialise) noexcept
{
    for (uint8_t i = 0; i < RTPS_MAX_LOCAL_ENDPOINTS; ++i) {
        if (!endpoints[i].is_valid()) continue;

        BufRef msg = buf_alloc_with_headroom(
            estimate(endpoints[i]), kRtpsDataHeadroom);
        if (!msg.valid()) continue;

        const uint32_t plen = serialise(endpoints[i], msg.data, msg.capacity());
        if (plen == 0) {
            msg.reset();
            continue;
        }
        msg.size = plen;

        const bool ok = builder.begin_with_payload(static_cast<BufRef&&>(msg)) &&
                        builder.prepend_data_message(
                            local_prefix,
                            reader_id,
                            writer_id,
                            SequenceNumber::from_int64(1),
                            /*key_flag=*/false);

        if (ok) {
            BufRef out = builder.finish();
            transport.send_unicast(out, dest);
            out.reset();
        } else {
            builder.abort();
        }
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
    const uint16_t rep_id = 0x0003u, rep_opt = 0x0000u;
    if (off + 4u > cap) return 0;
    memcpy(buf + off, &rep_id,  2u); off += 2u;
    memcpy(buf + off, &rep_opt, 2u); off += 2u;

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

    const uint16_t rep_id = 0x0003u, rep_opt = 0x0000u;
    if (off + 4u > cap) return 0;
    memcpy(buf + off, &rep_id,  2u); off += 2u;
    memcpy(buf + off, &rep_opt, 2u); off += 2u;

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

void SedpWriter::announce_to_peer(const GuidPrefix& local_prefix,
                                  const Locator&    dest) noexcept
{
    announce_endpoints(
        builder_,
        transport_,
        local_prefix,
        dest,
        local_writers_,
        entity_id::SEDP_BUILTIN_PUBLICATIONS_DETECTOR,
        entity_id::SEDP_BUILTIN_PUBLICATIONS_ANNOUNCER,
        writer_payload_size_estimate,
        [this](const DiscoveredWriterData& wd,
               Octet* buf,
               uint32_t cap) noexcept {
            return serialise_writer(wd, buf, cap);
        });

    announce_endpoints(
        builder_,
        transport_,
        local_prefix,
        dest,
        local_readers_,
        entity_id::SEDP_BUILTIN_SUBSCRIPTIONS_DETECTOR,
        entity_id::SEDP_BUILTIN_SUBSCRIPTIONS_ANNOUNCER,
        reader_payload_size_estimate,
        [this](const DiscoveredReaderData& rd,
               Octet* buf,
               uint32_t cap) noexcept {
            return serialise_reader(rd, buf, cap);
        });
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

    uint16_t rep_id;
    memcpy(&rep_id, payload, 2u);
    if (rep_id != 0x0003u && rep_id != 0x0002u) return false;

    uint32_t off      = 4u;
    bool     got_guid = false;
    unicast_count     = 0;

    while (off + 4u <= len) {
        uint16_t pid_val, param_len;
        memcpy(&pid_val,   payload + off,     2u);
        memcpy(&param_len, payload + off + 2u, 2u);
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
                    uint32_t slen;
                    memcpy(&slen, val, 4u);
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
                    uint32_t slen;
                    memcpy(&slen, val, 4u);
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
                    uint32_t v; memcpy(&v, val, 4u);
                    reliability_out = static_cast<ReliabilityKind>(v);
                }
                break;

            case pid::DURABILITY:
                if (param_len >= 4u) {
                    uint32_t v; memcpy(&v, val, 4u);
                    durability_out = static_cast<DurabilityKind>(v);
                }
                break;

            case pid::UNICAST_LOCATOR:
                if (param_len >= 24u &&
                    unicast_count < RTPS_MAX_LOCATORS_PER_PARTICIPANT) {
                    memcpy(&unicast_locs[unicast_count++], val, 24u);
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

void SedpReader::on_publication_data(const GuidPrefix& /*sender_prefix*/,
                                     const Octet*      payload,
                                     uint32_t          payload_len) noexcept
{
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

void SedpReader::on_subscription_data(const GuidPrefix& /*sender_prefix*/,
                                      const Octet*      payload,
                                      uint32_t          payload_len) noexcept
{
    DiscoveredReaderData rd{};
    if (!deserialise_reader(payload, payload_len, rd)) return;
    if (!rd.is_valid()) return;
    match_table_.add_reader(rd);
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
