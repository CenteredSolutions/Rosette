/**
 * @file test_sedp_8_5_4_rebuild.cpp
 * @brief SEDP announcements rebuilt for each send instead of stored.
 *
 * A SedpWriter given a LocalEndpointSource keeps each live announcement as a
 * history change without a payload and serializes it again whenever it is
 * sent (sedp.hpp, "Announcement storage"). These tests pin down that the
 * choice is invisible on the wire: a rebuilt announcement is byte for byte
 * the stored one, at first send, on replay to a late peer, and on repair,
 * while no pool block stays allocated per endpoint. They also cover the
 * endpoint table lookups the participant's source is built on.
 */

#include "sedp_recorded_endpoints.hpp"
#include "spec_test_common.hpp"

#include "rosette/rtps/buf_pool.hpp"
#include "rosette/rtps/sedp.hpp"
#include "rosette/rtps/transport.hpp"

namespace spec_test {

extern uint32_t g_stateful_writer_test_now_ms;

#define CHECK(COND, MSG)  \
    do {                  \
        if (!(COND)) {    \
            err = (MSG);  \
            return false; \
        }                 \
    } while (0)

namespace {

struct RecordingTransport : ITransport {
    std::vector<std::vector<Octet>> sent{};

    bool send_unicast(const BufRef& buf, const Locator&) noexcept override {
        sent.emplace_back(buf.data, buf.data + buf.size);
        return true;
    }

    bool send_multicast(const BufRef&, const Locator&) noexcept override { return false; }
};

/// Pool blocks currently allocated, across every size class.
uint32_t live_blocks() {
    uint32_t live = 0u;
    for (uint8_t c = 0; c < g_buf_pool.class_count(); ++c) {
        const MultiBufPool::ClassInfo ci = g_buf_pool.class_info(c);
        live += static_cast<uint32_t>(ci.total - ci.available);
    }

    return live;
}

Guid make_guid(Octet p0, const EntityId& eid) {
    Guid g{};
    g.prefix.value[0] = p0;
    g.prefix.value[1] = 0xAEu;
    g.entityId = eid;
    return g;
}

DiscoveredWriterData make_writer(const Guid& g) {
    DiscoveredWriterData wd{};
    wd.writerGuid = g;
    wd.topicName.assign("rt/machine/arm_0a1b2c3d/follow_joint_trajectory/_action/feedback");
    wd.typeName.assign("control_msgs::action::dds_::FollowJointTrajectory_FeedbackMessage_");
    wd.reliabilityKind = ReliabilityKind::RELIABLE;
    wd.durabilityKind = DurabilityKind::VOLATILE;
    wd.unicastLocatorCount = 1u;
    wd.unicastLocatorList[0] = make_udpv4_locator(0xC0A80110u, 7411u);
    return wd;
}

DiscoveredReaderData make_reader(const Guid& g) {
    DiscoveredReaderData rd{};
    rd.readerGuid = g;
    rd.topicName.assign("rq/machine/arm_0a1b2c3d/follow_joint_trajectory/_action/send_goalRequest");
    rd.typeName.assign("control_msgs::action::dds_::FollowJointTrajectory_SendGoal_Request_");
    rd.reliabilityKind = ReliabilityKind::RELIABLE;
    rd.durabilityKind = DurabilityKind::VOLATILE;
    rd.unicastLocatorCount = 1u;
    rd.unicastLocatorList[0] = make_udpv4_locator(0xC0A80110u, 7411u);
    return rd;
}

/// The serializedPayloads of every DATA in `sent`, in order.
std::vector<std::vector<Octet>> data_payloads(const std::vector<std::vector<Octet>>& sent) {
    std::vector<std::vector<Octet>> out{};
    ParseOps ops{};
    ops.on_data = [](const ParseContext& ctx, const ParsedData& d) noexcept {
        auto* payloads = static_cast<std::vector<std::vector<Octet>>*>(ctx.user_data);
        payloads->emplace_back(d.serializedPayload, d.serializedPayload + d.serializedPayloadSize);
    };
    for (const std::vector<Octet>& msg : sent) {
        (void)parse_message(msg.data(), static_cast<uint32_t>(msg.size()), GuidPrefix{}, ops, &out);
    }

    return out;
}

/// An ACKNACK from `peer`'s publications detector that NACKs SN 1 alone.
void nack_first_publication(SedpWriter& w, const GuidPrefix& peer, int32_t count) {
    ParsedAckNack ack{};
    ack.readerId = entity_id::SEDP_BUILTIN_PUBLICATIONS_DETECTOR;
    ack.writerId = entity_id::SEDP_BUILTIN_PUBLICATIONS_ANNOUNCER;
    (void)ack.readerSNState.add(SequenceNumber::from_int64(1));
    ack.count = count;
    ParseContext ctx{};
    ctx.senderGuidPrefix = peer;
    w.find_writer(entity_id::SEDP_BUILTIN_PUBLICATIONS_ANNOUNCER)->on_acknack(ctx, ack);
}

}  // namespace

// A rebuilt announcement is byte for byte the stored one, and the writer
// holds no pool block for it between sends.
static bool tc_sedp_8_5_4_015(std::string& err) {
    const GuidPrefix local = make_guid(0x01u, entity_id::PARTICIPANT).prefix;
    const GuidPrefix peer = make_guid(0x02u, entity_id::PARTICIPANT).prefix;
    const Locator dest = make_udpv4_locator(0xC0A80102u, 7410u);
    const DiscoveredWriterData wd = make_writer(make_guid(0x01u, make_entity(1u, 2u, 3u, 0x03u)));
    const DiscoveredReaderData rd = make_reader(make_guid(0x01u, make_entity(4u, 5u, 6u, 0x04u)));
    RecordedEndpoints endpoints{};
    endpoints.writers.push_back(wd);
    endpoints.readers.push_back(rd);

    const uint32_t live_before = live_blocks();
    RecordingTransport stored_tx{};
    SedpWriter stored(local, stored_tx);
    CHECK(!stored.rebuilds_announcements(), "a SedpWriter without a source stores announcements");
    CHECK(stored.register_writer(wd) && stored.register_reader(rd), "stored registration");
    const uint32_t stored_held = live_blocks() - live_before;
    CHECK(stored_held == 2u, "a stored announcement holds one block per endpoint");

    RecordingTransport rebuilt_tx{};
    SedpWriter rebuilt(local, rebuilt_tx, endpoints.source());
    CHECK(rebuilt.rebuilds_announcements(), "a SedpWriter with a source rebuilds announcements");
    CHECK(rebuilt.register_writer(wd) && rebuilt.register_reader(rd), "rebuilt registration");
    CHECK(live_blocks() - live_before == stored_held, "a rebuilt announcement holds no block");

    stored.announce_to_peer(peer, dest);
    rebuilt.announce_to_peer(peer, dest);
    CHECK(!stored_tx.sent.empty(), "the stored writer announces to the peer");
    CHECK(rebuilt_tx.sent == stored_tx.sent, "rebuilt datagrams equal stored ones byte for byte");
    CHECK(live_blocks() - live_before == stored_held,
          "a rebuilt send returns its block to the pool");

    // A peer matched later is replayed the same bytes.
    const GuidPrefix late = make_guid(0x03u, entity_id::PARTICIPANT).prefix;
    stored_tx.sent.clear();
    rebuilt_tx.sent.clear();
    stored.announce_to_peer(late, dest);
    rebuilt.announce_to_peer(late, dest);
    CHECK(!rebuilt_tx.sent.empty() && rebuilt_tx.sent == stored_tx.sent,
          "TRANSIENT_LOCAL replay to a late peer is identical");
    return true;
}

// A rebuild that fails (the endpoint source no longer knows the endpoint)
// is a lost datagram: nothing is sent, no block leaks, and the next repair
// request is answered once the source can describe the endpoint again.
static bool tc_sedp_8_5_4_016(std::string& err) {
    const GuidPrefix local = make_guid(0x11u, entity_id::PARTICIPANT).prefix;
    const GuidPrefix peer = make_guid(0x12u, entity_id::PARTICIPANT).prefix;
    const Locator dest = make_udpv4_locator(0xC0A80112u, 7410u);
    const DiscoveredWriterData wd = make_writer(make_guid(0x11u, make_entity(1u, 1u, 1u, 0x03u)));
    RecordedEndpoints endpoints{};
    endpoints.writers.push_back(wd);
    RecordingTransport tx{};
    SedpWriter w(local, tx, endpoints.source());
    CHECK(w.register_writer(wd), "registration");
    w.announce_to_peer(peer, dest);
    const std::vector<std::vector<Octet>> first = data_payloads(tx.sent);
    CHECK(first.size() == 1u, "the peer is sent the announcement once");

    const uint32_t live_before = live_blocks();
    endpoints.writers.clear();
    tx.sent.clear();
    g_stateful_writer_test_now_ms = 1000u;
    nack_first_publication(w, peer, 1);
    g_stateful_writer_test_now_ms += RTPS_NACK_RESPONSE_DELAY_MS + 1u;
    w.tick(g_stateful_writer_test_now_ms);
    CHECK(data_payloads(tx.sent).empty(), "an announcement that cannot be rebuilt is not sent");
    CHECK(live_blocks() == live_before, "a failed rebuild leaks no block");

    endpoints.writers.push_back(wd);
    tx.sent.clear();
    nack_first_publication(w, peer, 2);
    g_stateful_writer_test_now_ms += RTPS_NACK_RESPONSE_DELAY_MS + 1u;
    w.tick(g_stateful_writer_test_now_ms);
    CHECK(data_payloads(tx.sent) == first, "the next NACK is repaired with the same announcement");
    CHECK(live_blocks() == live_before, "the repair returns its block to the pool");
    return true;
}

// The endpoint table serves back what it recorded for a local endpoint and
// nothing for a remote, unknown, or removed one.
static bool tc_sedp_8_5_4_017(std::string& err) {
    EndpointMatchTable table(nullptr, nullptr, nullptr);
    const GuidPrefix local = make_guid(0x21u, entity_id::PARTICIPANT).prefix;
    table.set_local_prefix(local);
    DiscoveredWriterData wd = make_writer(make_guid(0x21u, make_entity(1u, 0u, 1u, 0x03u)));
    DiscoveredReaderData rd = make_reader(make_guid(0x21u, make_entity(2u, 0u, 2u, 0x04u)));
    rd.expectsInlineQos = true;
    table.add_writer(wd);
    table.add_reader(rd);

    DiscoveredWriterData w_out{};
    CHECK(table.local_writer_data(wd.writerGuid, w_out), "a local writer is found");
    CHECK(w_out.writerGuid == wd.writerGuid && w_out.topicName == wd.topicName
              && w_out.typeName == wd.typeName && w_out.reliabilityKind == wd.reliabilityKind
              && w_out.durabilityKind == wd.durabilityKind,
          "the writer's GUID, names, and QoS are served back");
    CHECK(w_out.unicastLocatorCount == 0u, "the table adds no locators");

    DiscoveredReaderData r_out{};
    CHECK(table.local_reader_data(rd.readerGuid, r_out), "a local reader is found");
    CHECK(r_out.readerGuid == rd.readerGuid && r_out.topicName == rd.topicName
              && r_out.typeName == rd.typeName && r_out.expectsInlineQos,
          "the reader's GUID, names, and expectsInlineQos are served back");

    const DiscoveredWriterData remote =
        make_writer(make_guid(0x22u, make_entity(1u, 0u, 1u, 0x03u)));
    table.add_writer(remote);
    CHECK(!table.local_writer_data(remote.writerGuid, w_out), "a remote writer is not local");
    CHECK(!table.local_reader_data(wd.writerGuid, r_out), "a writer GUID is not a reader");
    table.remove_writer(wd.writerGuid);
    CHECK(!table.local_writer_data(wd.writerGuid, w_out), "a removed writer is gone");
    return true;
}

void append_sedp_tests_8_5_4_rebuild(std::vector<TestCase>& tests) {
    tests.push_back(
        {"TC-SEDP-8.5.4-015",
         "§8.5.4/§9.6.3.2: an announcement rebuilt for each send is byte for byte the stored one, "
         "at first send and on TRANSIENT_LOCAL replay, and holds no pool block between sends.",
         tc_sedp_8_5_4_015});
    tests.push_back({"TC-SEDP-8.5.4-016",
                     "§8.4.9.2: an announcement that cannot be rebuilt is a lost datagram, leaking "
                     "nothing; the next NACK is repaired with the same bytes.",
                     tc_sedp_8_5_4_016});
    tests.push_back({"TC-SEDP-8.5.4-017",
                     "§8.5.5.1: the endpoint table serves back a local endpoint's recorded "
                     "discovery data, and nothing for remote, unknown, or removed endpoints.",
                     tc_sedp_8_5_4_017});
}

}  // namespace spec_test
