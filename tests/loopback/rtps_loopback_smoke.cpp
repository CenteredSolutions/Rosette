/**
 * @file rtps_loopback_smoke.cpp
 * @brief Two in-process RTPSParticipants, wired directly to each other's
 *        parse_received() instead of real sockets — the smallest possible
 *        end-to-end proof that the restored RTPS core still works: SPDP
 *        discovery, SEDP endpoint matching, and a reliable DATA delivery.
 *
 * No ROS 2, no rosidl, no micro-ROS codegen, no rmw — this only exercises
 * rosette::rtps directly, which is deliberately the layer rmw_rosette will
 * sit on top of. If this doesn't work, nothing built on top of it will.
 */

#include "rosette/rtps/participant.hpp"
#include "rosette/rtps/spdp.hpp"
#include "rosette/rtps/stateful_reader.hpp"
#include "rosette/rtps/stateful_writer.hpp"
#include "rosette/rtps/transport.hpp"

#include <cstdio>
#include <cstring>
#include <vector>

namespace {

uint32_t g_now_ms = 0;

// A transport that captures everything sent instead of touching a real
// socket. The test driver drains one participant's outbox directly into
// the other's parse_received() — a synthetic "wire" with zero loss and
// zero latency, matching how tests/spec's CaptureTransport-style fakes
// already work in this codebase (see test_participant_8_5_1.cpp).
struct BridgeTransport : rtps::ITransport {
    std::vector<std::vector<rtps::Octet>> outbox;

    bool send_unicast(const rtps::BufRef& buf, const rtps::Locator&) noexcept override {
        capture(buf);
        return true;
    }
    bool send_multicast(const rtps::BufRef& buf, const rtps::Locator&) noexcept override {
        capture(buf);
        return true;
    }

private:
    void capture(const rtps::BufRef& buf) {
        outbox.emplace_back(buf.data, buf.data + buf.size);
    }
};

struct ReceivedMessage {
    std::vector<rtps::Octet> bytes;
};

std::vector<ReceivedMessage> g_received;

void on_change_available(const rtps::CacheChange& change, void* /*user_data*/) noexcept {
    ReceivedMessage msg;
    msg.bytes.assign(change.payload.data, change.payload.data + change.payload.size);
    g_received.push_back(static_cast<ReceivedMessage&&>(msg));
}

/// Drains `from`'s outbox into `to`'s parse_received(), one packet at a time.
void deliver(BridgeTransport& from, rtps::RTPSParticipant& to) {
    for (auto& pkt : from.outbox) {
        to.parse_received(pkt.data(), static_cast<uint32_t>(pkt.size()));
    }
    from.outbox.clear();
}

}  // namespace

// Platform hooks rosette::rtps requires the application to provide.
extern "C" uint32_t rtps_platform_get_ms(void) {
    return g_now_ms;
}

extern "C" rtps::GuidPrefix rtps_platform_make_guid_prefix(uint8_t participant_id) noexcept {
    rtps::GuidPrefix gp{};
    gp.value[0] = 0x52;  // 'R' for Rosette, arbitrary.
    gp.value[11] = participant_id;
    return gp;
}

int main() {
    BridgeTransport transport_a;
    BridgeTransport transport_b;

    rtps::RTPSParticipant participant_a(/*domain_id=*/0, /*participant_id=*/1, transport_a);
    rtps::RTPSParticipant participant_b(/*domain_id=*/0, /*participant_id=*/2, transport_b);

    // Locators are never actually dereferenced by BridgeTransport, but
    // RTPSParticipant::tick()/announce() both no-op without at least one
    // address configured (see participant.cpp's has_addresses() gate).
    participant_a.add_ipv4_address(0x7F000001u /* 127.0.0.1 */);
    participant_b.add_ipv4_address(0x7F000001u);

    rtps::WriterConfig writer_cfg;
    writer_cfg.entityId.entityKey[0] = 0x01;
    writer_cfg.entityId.entityKind   = 0x03;  // user-defined writer, no key
    writer_cfg.topicName.assign("loopback_smoke");
    writer_cfg.typeName.assign("std_msgs::msg::dds_::String_");
    writer_cfg.reliability = rtps::ReliabilityKind::RELIABLE;

    rtps::ReaderConfig reader_cfg;
    reader_cfg.entityId.entityKey[0] = 0x01;
    reader_cfg.entityId.entityKind   = 0x04;  // user-defined reader, no key
    reader_cfg.topicName.assign("loopback_smoke");
    reader_cfg.typeName.assign("std_msgs::msg::dds_::String_");
    reader_cfg.reliability = rtps::ReliabilityKind::RELIABLE;

    auto* writer_concrete = participant_a.create_writer<1>(writer_cfg);
    auto* reader = participant_b.create_reader(reader_cfg, on_change_available);

    if (!writer_concrete || !reader) {
        std::fprintf(stderr, "FAIL: create_writer/create_reader returned nullptr\n");
        return 1;
    }

    // Exercise the depth-erased IStatefulWriter path deliberately, not the
    // concrete StatefulWriter<1,...>* — this is what an rmw_create_publisher()
    // that picked CacheDepth at runtime (matching rmw_qos_profile_t.depth
    // from a small closed set of instantiations) would actually hold.
    rtps::IStatefulWriter* writer = writer_concrete;

    participant_a.announce();
    participant_b.announce();

    // Drive SPDP -> SEDP -> endpoint matching to completion. 500 steps of
    // 10ms = 5s of simulated time, comfortably past the default 2s
    // heartbeat period and 200ms nack-response delay.
    constexpr uint32_t kStepMs = 10;
    constexpr int kDiscoverySteps = 500;
    for (int i = 0; i < kDiscoverySteps; ++i) {
        g_now_ms += kStepMs;
        participant_a.tick(g_now_ms);
        participant_b.tick(g_now_ms);
        deliver(transport_a, participant_b);
        deliver(transport_b, participant_a);
    }

    const char payload_text[] = "hello from rosette rtps";
    const bool sent = writer->write_reliable(
        rtps::ChangeKind::ALIVE,
        reinterpret_cast<const rtps::Octet*>(payload_text),
        static_cast<uint32_t>(sizeof(payload_text)));

    if (!sent) {
        std::fprintf(stderr, "FAIL: write_reliable() returned false\n");
        return 1;
    }

    // Drain the DATA + any HEARTBEAT/ACKNACK follow-up.
    constexpr int kDeliverySteps = 200;
    for (int i = 0; i < kDeliverySteps; ++i) {
        g_now_ms += kStepMs;
        participant_a.tick(g_now_ms);
        participant_b.tick(g_now_ms);
        deliver(transport_a, participant_b);
        deliver(transport_b, participant_a);
        if (!g_received.empty()) break;
    }

    if (g_received.empty()) {
        std::fprintf(stderr, "FAIL: reader never received a change "
                              "(endpoints likely never matched — check SPDP/SEDP)\n");
        return 1;
    }

    // MessageBuilder::append_data() always prepends a 4-byte CDR_LE
    // encapsulation header (representation identifier octets {0x00, 0x01},
    // two zero option octets; DDSI-RTPS 2.5 §10.2 — see message_builder.hpp
    // and tests/spec's TC-BLD-10.2-001) ahead of
    // whatever payload bytes the writer was given, and the reader reports
    // that header as part of the delivered CacheChange — this is the same
    // framing convention rosidl's CDR serializers expect to produce, not
    // an artifact of this test.
    const rtps::Octet expected_header[4] = {0x00, 0x01, 0x00, 0x00};
    const auto& got = g_received.front().bytes;
    const bool ok = got.size() == sizeof(expected_header) + sizeof(payload_text) &&
        std::memcmp(got.data(), expected_header, sizeof(expected_header)) == 0 &&
        std::memcmp(got.data() + sizeof(expected_header), payload_text, sizeof(payload_text)) == 0;
    if (!ok) {
        std::fprintf(stderr, "FAIL: payload mismatch (got %zu bytes, expected %zu)\n",
                     got.size(), sizeof(expected_header) + sizeof(payload_text));
        return 1;
    }

    std::printf("PASS: received %zu bytes (4-byte encapsulation header + \"%s\")\n", got.size(),
                reinterpret_cast<const char*>(got.data() + sizeof(expected_header)));
    return 0;
}
