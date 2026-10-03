/**
 * @file sedp_peer_driver.hpp
 * @brief Plays a remote participant's SEDP announcers against a SedpReader.
 *
 * SedpReader receives announcements through its two builtin StatefulReaders,
 * exactly as the participant routes them (SedpReader::reader_for_writer()).
 * SedpPeer builds the DATA, HEARTBEAT and GAP submessages a remote
 * SEDPbuiltinPublicationsWriter or SEDPbuiltinSubscriptionsWriter would
 * send and hands them to the matching builtin reader, so SEDP tests exercise
 * the same path as received traffic without encoding and parsing datagrams.
 */

#pragma once

#include "rosette/rtps/sedp.hpp"

#include <vector>

namespace spec_test {

using namespace rtps;

/// A remote participant whose SEDP announcers talk to one SedpReader.
struct SedpPeer {
    GuidPrefix prefix{};
    Locator    meta = LOCATOR_INVALID;   ///< Metatraffic unicast locator.

    /// Stands in for the peer's SPDP peer-table slot. Matched proxies refer
    /// to its locators rather than copying them, so, like the slot, it lives
    /// as long as the peer does.
    mutable DiscoveredParticipant record{};

    /// @brief What SPDP keeps of the peer, listing both SEDP announcers and detectors.
    DiscoveredParticipant spdp() const {
        DiscoveredParticipant d{};
        d.guidPrefix = prefix;
        d.metatrafficUnicastLocatorList[0] = meta;
        d.metatrafficUnicastLocatorCount = 1u;
        d.availableBuiltinEndpoints =
            builtin_endpoint::DISC_BUILTIN_ENDPOINT_PUBLICATIONS_ANNOUNCER |
            builtin_endpoint::DISC_BUILTIN_ENDPOINT_PUBLICATIONS_DETECTOR |
            builtin_endpoint::DISC_BUILTIN_ENDPOINT_SUBSCRIPTIONS_ANNOUNCER |
            builtin_endpoint::DISC_BUILTIN_ENDPOINT_SUBSCRIPTIONS_DETECTOR;
        return d;
    }

    /// @brief SPDP discovered this peer: its announcers become matched writers.
    void discover(SedpReader& reader) const {
        record = spdp();
        reader.on_peer_discovered(record);
    }

    /// @brief DATA carrying a PL_CDR endpoint announcement.
    void announce(SedpReader& reader, const EntityId& writer, int64_t sn,
                  const std::vector<Octet>& payload) const {
        ParsedData d = data(writer, sn);
        d.dataFlag = true;
        d.serializedPayload = payload.data();
        d.serializedPayloadSize = static_cast<uint32_t>(payload.size());
        deliver(reader, writer, d);
    }

    /// @brief The same announcement split into DATA_FRAGs of `frag_size` octets.
    void announce_fragmented(SedpReader& reader, const EntityId& writer, int64_t sn,
                             const std::vector<Octet>& payload, uint16_t frag_size) const {
        const uint32_t total = static_cast<uint32_t>(payload.size());
        for (uint32_t off = 0, num = 1; off < total; off += frag_size, ++num) {
            ParsedDataFrag f{};
            f.writerId = writer;
            f.readerId = detector_for(writer);
            f.writerSN = SequenceNumber::from_int64(sn);
            f.fragmentStartingNum = num;
            f.fragmentsInSubmessage = 1u;
            f.fragmentSize = frag_size;
            f.dataSize = total;
            f.fragmentPayload = payload.data() + off;
            f.fragmentPayloadSize = (total - off < frag_size) ? total - off : frag_size;
            reader.reader_for_writer(writer)->on_data_frag(context(), f);
        }
    }

    /// @brief DATA with neither payload nor key: only its sequence number counts.
    void account(SedpReader& reader, const EntityId& writer, int64_t sn) const {
        deliver(reader, writer, data(writer, sn));
    }

    /// @brief Inline-QoS-only DATA withdrawing `endpoint` (DISPOSED|UNREGISTERED).
    void dispose(SedpReader& reader, const EntityId& writer, int64_t sn,
                 const Guid& endpoint) const {
        ParsedData d = data(writer, sn);
        d.inlineQosPresent = true;
        d.inlineQos.set_key_hash(endpoint);
        d.inlineQos.hasStatusInfo = true;
        d.inlineQos.statusInfo = status_info::DISPOSED | status_info::UNREGISTERED;
        deliver(reader, writer, d);
    }

    /// @brief HEARTBEAT announcing changes [first, last].
    void heartbeat(SedpReader& reader, const EntityId& writer, int64_t first, int64_t last,
                   int32_t count, bool final_flag = false) const {
        ParsedHeartbeat hb{};
        hb.writerId = writer;
        hb.readerId = detector_for(writer);
        hb.firstSN = SequenceNumber::from_int64(first);
        hb.lastSN = SequenceNumber::from_int64(last);
        hb.count = count;
        hb.finalFlag = final_flag;
        reader.reader_for_writer(writer)->on_heartbeat(context(), hb);
    }

    /// @brief GAP declaring [start, base) irrelevant (empty bitmap).
    void gap(SedpReader& reader, const EntityId& writer, int64_t start, int64_t base) const {
        ParsedGap g{};
        g.writerId = writer;
        g.readerId = detector_for(writer);
        g.gapStart = SequenceNumber::from_int64(start);
        g.gapList.bitmapBase = SequenceNumber::from_int64(base);
        reader.reader_for_writer(writer)->on_gap(context(), g);
    }

    ParseContext context() const {
        ParseContext ctx{};
        ctx.senderGuidPrefix = prefix;
        return ctx;
    }

    static EntityId detector_for(const EntityId& writer) {
        return writer == entity_id::SEDP_BUILTIN_PUBLICATIONS_ANNOUNCER
                   ? entity_id::SEDP_BUILTIN_PUBLICATIONS_DETECTOR
                   : entity_id::SEDP_BUILTIN_SUBSCRIPTIONS_DETECTOR;
    }

private:
    ParsedData data(const EntityId& writer, int64_t sn) const {
        ParsedData d{};
        d.writerId = writer;
        d.readerId = detector_for(writer);
        d.writerSN = SequenceNumber::from_int64(sn);
        return d;
    }

    void deliver(SedpReader& reader, const EntityId& writer, const ParsedData& d) const {
        reader.reader_for_writer(writer)->on_data(context(), d);
    }
};

}  // namespace spec_test
