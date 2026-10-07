/**
 * @file participant_test_support.hpp
 * @brief What the participant spec tests share: a capturing transport and
 *        builders for the SPDP, SEDP and RTPS messages a remote peer sends.
 *
 * The participant tests are compiled into several executables, each with its
 * own capacity macros (the peer-tier tests need a small active tier), so the
 * helpers are inline and every test file includes them.
 */

#pragma once

#include "spec_test_common.hpp"

#include "rosette/rtps/participant.hpp"
#include "rosette/rtps/spdp.hpp"
#include "rosette/rtps/transport.hpp"

#include <string.h>

#include <vector>

namespace spec_test {
namespace participant_support {

struct CaptureTransport : ITransport {
    struct Packet {
        std::vector<Octet> bytes{};
        Locator dest = LOCATOR_INVALID;
        bool multicast = false;
    };

    std::vector<Packet> packets{};

    bool send_unicast(const BufRef& buf, const Locator& dest) noexcept override {
        Packet p{};
        p.bytes.assign(buf.data, buf.data + buf.size);
        p.dest = dest;
        p.multicast = false;
        packets.push_back(static_cast<Packet&&>(p));
        return true;
    }

    bool send_multicast(const BufRef& buf, const Locator& dest) noexcept override {
        Packet p{};
        p.bytes.assign(buf.data, buf.data + buf.size);
        p.dest = dest;
        p.multicast = true;
        packets.push_back(static_cast<Packet&&>(p));
        return true;
    }
};

struct ReaderChangeObs {
    int calls = 0;
};

inline void on_reader_change(const CacheChange&, void* user_data) noexcept {
    auto* obs = static_cast<ReaderChangeObs*>(user_data);
    obs->calls++;
}

inline Guid make_guid(Octet p0, Octet p1, Octet p2, const EntityId& eid) {
    Guid g{};
    g.prefix.value[0] = p0;
    g.prefix.value[1] = p1;
    g.prefix.value[2] = p2;
    g.entityId = eid;
    return g;
}

inline GuidPrefix make_prefix(Octet p0, Octet p1, Octet p2) {
    GuidPrefix gp{};
    gp.value[0] = p0;
    gp.value[1] = p1;
    gp.value[2] = p2;
    return gp;
}

inline bool build_data_message(const GuidPrefix& sender_prefix, const EntityId& reader_id,
                               const EntityId& writer_id, const std::vector<Octet>& payload,
                               std::vector<Octet>& out,
                               const SequenceNumber& writer_sn = SequenceNumber::from_int64(1)) {
    BufRef payload_buf =
        buf_alloc_with_headroom(static_cast<uint32_t>(payload.size()), kRtpsDataHeadroom);
    if (!payload_buf.valid()) {
        return false;
    }

    if (!payload.empty()) {
        memcpy(payload_buf.data, payload.data(), payload.size());
    }

    payload_buf.size = static_cast<uint32_t>(payload.size());

    MessageBuilder builder{};
    const bool ok = builder.begin_with_payload(static_cast<BufRef&&>(payload_buf))
                    && builder.prepend_data_message(sender_prefix, reader_id, writer_id, writer_sn);
    if (!ok) {
        builder.abort();
        return false;
    }

    BufRef msg = builder.finish();
    out.assign(msg.data, msg.data + msg.size);
    msg.reset();
    return true;
}

inline bool build_acknack_message(const GuidPrefix& sender_prefix, const EntityId& reader_id,
                                  const EntityId& writer_id, const SequenceNumberSet& sn_state,
                                  int32_t count, bool final_flag, std::vector<Octet>& out) {
    MessageBuilder builder{};
    if (!builder.begin(sender_prefix)) {
        return false;
    }

    if (!builder.append_acknack(reader_id, writer_id, sn_state, count, final_flag)) {
        builder.abort();
        return false;
    }

    BufRef msg = builder.finish();
    out.assign(msg.data, msg.data + msg.size);
    msg.reset();
    return true;
}

inline bool build_nack_frag_message(const GuidPrefix& sender_prefix, const EntityId& reader_id,
                                    const EntityId& writer_id, const SequenceNumber& writer_sn,
                                    const FragmentNumberSet& missing_frags, int32_t count,
                                    std::vector<Octet>& out) {
    MessageBuilder builder{};
    if (!builder.begin(sender_prefix)) {
        return false;
    }

    if (!builder.append_nack_frag(reader_id, writer_id, writer_sn, missing_frags, count)) {
        builder.abort();
        return false;
    }

    BufRef msg = builder.finish();
    out.assign(msg.data, msg.data + msg.size);
    msg.reset();
    return true;
}

inline bool build_spdp_payload(const GuidPrefix& guid_prefix, uint32_t lease_seconds,
                               const Locator& metatraffic_unicast, std::vector<Octet>& out,
                               const Locator& default_unicast = LOCATOR_INVALID) {
    Octet buf[512] = {};
    uint32_t off = 0;

    const uint16_t rep_id = 0x0003u;
    const uint16_t rep_opt = 0x0000u;
    memcpy(buf + off, &rep_id, 2u);
    off += 2u;
    memcpy(buf + off, &rep_opt, 2u);
    off += 2u;

    Octet guid[16] = {};
    memcpy(guid, guid_prefix.value, 12u);
    guid[12] = 0x00u;
    guid[13] = 0x00u;
    guid[14] = 0x01u;
    guid[15] = 0xC1u;
    if (!plcdr_write_param(buf, sizeof(buf), off, pid::PARTICIPANT_GUID, guid, 16u)) {
        return false;
    }

    const Octet pv[2] = {PROTOCOLVERSION_2_5.major, PROTOCOLVERSION_2_5.minor};
    if (!plcdr_write_param(buf, sizeof(buf), off, pid::PROTOCOL_VERSION, pv, 2u)) {
        return false;
    }

    if (!plcdr_write_param(buf, sizeof(buf), off, pid::VENDOR_ID, VENDORID_THIS_IMPL.vendorId,
                           2u)) {
        return false;
    }

    const uint32_t lease[2] = {lease_seconds, 0u};
    if (!plcdr_write_param(buf, sizeof(buf), off, pid::PARTICIPANT_LEASE_DURATION, lease, 8u)) {
        return false;
    }

    if (!plcdr_write_param(buf, sizeof(buf), off, pid::METATRAFFIC_UNICAST_LOCATOR,
                           &metatraffic_unicast, 24u)) {
        return false;
    }

    if (default_unicast != LOCATOR_INVALID
        && !plcdr_write_param(buf, sizeof(buf), off, pid::DEFAULT_UNICAST_LOCATOR, &default_unicast,
                              24u)) {
        return false;
    }

    // availableBuiltinEndpoints is a required SPDP field (§8.5.3.2,
    // Table 8.73); SEDP matches only the builtin readers a peer declares.
    const uint32_t builtin_endpoints =
        builtin_endpoint::DISC_BUILTIN_ENDPOINT_PARTICIPANT_ANNOUNCER
        | builtin_endpoint::DISC_BUILTIN_ENDPOINT_PARTICIPANT_DETECTOR
        | builtin_endpoint::DISC_BUILTIN_ENDPOINT_PUBLICATIONS_ANNOUNCER
        | builtin_endpoint::DISC_BUILTIN_ENDPOINT_PUBLICATIONS_DETECTOR
        | builtin_endpoint::DISC_BUILTIN_ENDPOINT_SUBSCRIPTIONS_ANNOUNCER
        | builtin_endpoint::DISC_BUILTIN_ENDPOINT_SUBSCRIPTIONS_DETECTOR;
    if (!plcdr_write_param(buf, sizeof(buf), off, pid::BUILTIN_ENDPOINT_SET, &builtin_endpoints,
                           4u)) {
        return false;
    }

    if (!plcdr_write_sentinel(buf, sizeof(buf), off)) {
        return false;
    }

    out.assign(buf, buf + off);
    return true;
}

inline bool build_publication_payload(const DiscoveredWriterData& wd, std::vector<Octet>& out) {
    Octet buf[512] = {};
    uint32_t off = 0;

    const uint16_t rep_id = 0x0003u;
    const uint16_t rep_opt = 0x0000u;
    memcpy(buf + off, &rep_id, 2u);
    off += 2u;
    memcpy(buf + off, &rep_opt, 2u);
    off += 2u;

    Octet g[16] = {};
    memcpy(g, wd.writerGuid.prefix.value, 12u);
    memcpy(g + 12u, &wd.writerGuid.entityId, 4u);
    if (!plcdr_write_param(buf, sizeof(buf), off, pid::ENDPOINT_GUID, g, 16u)) {
        return false;
    }

    if (!plcdr_write_string(buf, sizeof(buf), off, pid::TOPIC_NAME, wd.topicName.str)) {
        return false;
    }

    if (!plcdr_write_string(buf, sizeof(buf), off, pid::TYPE_NAME, wd.typeName.str)) {
        return false;
    }

    {
        const uint32_t rel = static_cast<uint32_t>(wd.reliabilityKind);
        if (!plcdr_write_param(buf, sizeof(buf), off, pid::RELIABILITY, &rel, 4u)) {
            return false;
        }
    }

    {
        const uint32_t dur = static_cast<uint32_t>(wd.durabilityKind);
        if (!plcdr_write_param(buf, sizeof(buf), off, pid::DURABILITY, &dur, 4u)) {
            return false;
        }
    }

    for (uint8_t i = 0; i < wd.unicastLocatorCount; ++i) {
        if (!plcdr_write_param(buf, sizeof(buf), off, pid::UNICAST_LOCATOR,
                               &wd.unicastLocatorList[i], 24u)) {
            return false;
        }
    }

    if (!plcdr_write_sentinel(buf, sizeof(buf), off)) {
        return false;
    }

    out.assign(buf, buf + off);
    return true;
}

inline bool build_subscription_payload(const DiscoveredReaderData& rd, std::vector<Octet>& out) {
    Octet buf[512] = {};
    uint32_t off = 0;

    const uint16_t rep_id = 0x0003u;
    const uint16_t rep_opt = 0x0000u;
    memcpy(buf + off, &rep_id, 2u);
    off += 2u;
    memcpy(buf + off, &rep_opt, 2u);
    off += 2u;

    Octet g[16] = {};
    memcpy(g, rd.readerGuid.prefix.value, 12u);
    memcpy(g + 12u, &rd.readerGuid.entityId, 4u);
    if (!plcdr_write_param(buf, sizeof(buf), off, pid::ENDPOINT_GUID, g, 16u)) {
        return false;
    }

    if (!plcdr_write_string(buf, sizeof(buf), off, pid::TOPIC_NAME, rd.topicName.str)) {
        return false;
    }

    if (!plcdr_write_string(buf, sizeof(buf), off, pid::TYPE_NAME, rd.typeName.str)) {
        return false;
    }

    {
        const uint32_t rel = static_cast<uint32_t>(rd.reliabilityKind);
        if (!plcdr_write_param(buf, sizeof(buf), off, pid::RELIABILITY, &rel, 4u)) {
            return false;
        }
    }

    {
        const uint32_t dur = static_cast<uint32_t>(rd.durabilityKind);
        if (!plcdr_write_param(buf, sizeof(buf), off, pid::DURABILITY, &dur, 4u)) {
            return false;
        }
    }

    for (uint8_t i = 0; i < rd.unicastLocatorCount; ++i) {
        if (!plcdr_write_param(buf, sizeof(buf), off, pid::UNICAST_LOCATOR,
                               &rd.unicastLocatorList[i], 24u)) {
            return false;
        }
    }

    if (!plcdr_write_sentinel(buf, sizeof(buf), off)) {
        return false;
    }

    out.assign(buf, buf + off);
    return true;
}

inline EntityId packet_writer_id(const CaptureTransport::Packet& packet) {
    EntityId writer = entity_id::UNKNOWN;
    ParseOps ops{};
    ops.on_data = [](const ParseContext& ctx, const ParsedData& d) noexcept {
        *static_cast<EntityId*>(ctx.user_data) = d.writerId;
    };
    (void)parse_message(packet.bytes.data(), static_cast<uint32_t>(packet.bytes.size()),
                        GuidPrefix{}, ops, &writer);
    return writer;
}

inline size_t count_sedp_data_announcements(const CaptureTransport& tx) {
    struct CountCap {
        size_t announces = 0u;
    } cap{};

    ParseOps ops{};
    ops.on_data = [](const ParseContext& ctx, const ParsedData& d) noexcept {
        auto* c = static_cast<CountCap*>(ctx.user_data);
        const bool pub = d.writerId == entity_id::SEDP_BUILTIN_PUBLICATIONS_ANNOUNCER
                         && d.readerId == entity_id::SEDP_BUILTIN_PUBLICATIONS_DETECTOR;
        const bool sub = d.writerId == entity_id::SEDP_BUILTIN_SUBSCRIPTIONS_ANNOUNCER
                         && d.readerId == entity_id::SEDP_BUILTIN_SUBSCRIPTIONS_DETECTOR;
        if (pub || sub) {
            c->announces++;
        }
    };

    for (const auto& packet : tx.packets) {
        (void)parse_message(packet.bytes.data(), static_cast<uint32_t>(packet.bytes.size()),
                            GuidPrefix{}, ops, &cap);
    }

    return cap.announces;
}

}  // namespace participant_support
}  // namespace spec_test
