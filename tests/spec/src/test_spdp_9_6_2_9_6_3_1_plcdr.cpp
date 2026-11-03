// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

#include "spec_test_common.hpp"

#include "rosette/rtps/spdp.hpp"

namespace spec_test {

#define CHECK(COND, MSG) do { if (!(COND)) { err = (MSG); return false; } } while (0)

namespace {

struct ReaderObs {
    int new_calls = 0;
    int removed_calls = 0;
    GuidPrefix last_new = {};
};

static void on_new_peer(const PeerEntry& peer, void* user) noexcept {
    auto* obs = static_cast<ReaderObs*>(user);
    obs->new_calls++;
    obs->last_new = peer.guidPrefix;
}

static void on_removed(const PeerEntry&, void* user) noexcept {
    auto* obs = static_cast<ReaderObs*>(user);
    obs->removed_calls++;
}

static GuidPrefix make_prefix(Octet b0, Octet b1, Octet b2) {
    GuidPrefix gp{};
    gp.value[0] = b0;
    gp.value[1] = b1;
    gp.value[2] = b2;
    return gp;
}

static bool build_payload(const GuidPrefix& guid_prefix,
                          bool include_guid,
                          bool guid_all_zeros,
                          bool include_unknown_pid,
                          uint16_t default_unicast_pid,
                          std::vector<Octet>& out) {
    Octet buf[512] = {};
    uint32_t off = 0;

    const uint16_t rep_id = 0x0003u;
    const uint16_t rep_opt = 0x0000u;
    memcpy(buf + off, &rep_id, 2u); off += 2u;
    memcpy(buf + off, &rep_opt, 2u); off += 2u;

    if (include_unknown_pid) {
        const Octet junk[3] = {0xAAu, 0xBBu, 0xCCu};
        if (!plcdr_write_param(buf, sizeof(buf), off, 0x7777u, junk, 3u)) return false;
    }

    if (include_guid) {
        Octet guid[16] = {};
        if (!guid_all_zeros) memcpy(guid, guid_prefix.value, 12u);
        guid[12] = 0x00u;
        guid[13] = 0x00u;
        guid[14] = 0x01u;
        guid[15] = 0xC1u;
        if (!plcdr_write_param(buf, sizeof(buf), off, pid::PARTICIPANT_GUID, guid, 16u)) return false;
    }

    const Octet pv[2] = {PROTOCOLVERSION_2_5.major, PROTOCOLVERSION_2_5.minor};
    if (!plcdr_write_param(buf, sizeof(buf), off, pid::PROTOCOL_VERSION, pv, 2u)) return false;

    const Octet vid[2] = {VENDORID_THIS_IMPL.vendorId[0], VENDORID_THIS_IMPL.vendorId[1]};
    if (!plcdr_write_param(buf, sizeof(buf), off, pid::VENDOR_ID, vid, 2u)) return false;

    const uint32_t lease[2] = {1u, 0u};
    if (!plcdr_write_param(buf, sizeof(buf), off, pid::PARTICIPANT_LEASE_DURATION, lease, 8u)) return false;

    if (default_unicast_pid != 0u) {
        const Locator loc = make_udpv4_locator(0xC0A80144u, 7411u);
        if (!plcdr_write_param(buf, sizeof(buf), off, default_unicast_pid, &loc, 24u)) return false;
    }

    if (!plcdr_write_sentinel(buf, sizeof(buf), off)) return false;

    out.assign(buf, buf + off);
    return true;
}

} // namespace

static bool tc_spdp_9_6_2_001(std::string& err) {
    ReaderObs obs{};
    SpdpReader r(on_new_peer, on_removed, &obs);
    std::vector<Octet> payload{};
    CHECK(build_payload(make_prefix(0x10u, 0x11u, 0x12u), true, false, true, 0u, payload),
          "payload construction failed");
    CHECK(r.on_data(GuidPrefix{}, payload.data(), static_cast<uint32_t>(payload.size()), 100u),
          "SPDP PL_CDR payload with sentinel should deserialize");
    CHECK(r.peer_count() == 1u, "expected one discovered peer");
    CHECK(obs.new_calls == 1, "expected new peer callback once");
    return true;
}

static bool tc_spdp_9_6_2_002(std::string& err) {
    ReaderObs obs{};
    SpdpReader r(on_new_peer, on_removed, &obs);
    std::vector<Octet> payload{};
    CHECK(build_payload(make_prefix(0x20u, 0x21u, 0x22u), false, false, false, 0u, payload),
          "payload construction failed");
    CHECK(!r.on_data(GuidPrefix{}, payload.data(), static_cast<uint32_t>(payload.size()), 200u),
          "SPDP payload without PID_PARTICIPANT_GUID should be rejected");
    CHECK(r.peer_count() == 0u, "missing mandatory GUID should not add peer");
    return true;
}

/// A parameter whose declared length (0xFFFD) pads to 0x10000 must be treated
/// as running past the end of the payload, not as a zero-length parameter.
/// The payload is allocated at its exact size so a sanitizer build reports any
/// read beyond it.
static bool tc_spdp_9_6_2_003(std::string& err) {
    ReaderObs  obs{};
    SpdpReader r(on_new_peer, on_removed, &obs);

    const std::vector<Octet> payload = {
        0x00,
        0x03,
        0x00,
        0x00,  // PL_CDR_LE representation header
        static_cast<Octet>(pid::PARTICIPANT_GUID & 0xFFu),
        static_cast<Octet>(pid::PARTICIPANT_GUID >> 8u),
        0xFD,
        0xFF,  // parameter_length 0xFFFD
        0x01,
        0x02,
        0x03,
        0x04,  // 4 octets of value
    };
    CHECK(!r.on_data(GuidPrefix{}, payload.data(), static_cast<uint32_t>(payload.size()), 100u),
          "a parameter longer than the payload must be rejected");
    CHECK(r.peer_count() == 0u, "an oversized parameter must not add a peer");
    return true;
}

static bool tc_spdp_9_6_3_1_001(std::string& err) {
    ReaderObs obs{};
    SpdpReader r(on_new_peer, on_removed, &obs);
    std::vector<Octet> payload{};
    CHECK(build_payload(GuidPrefix{}, true, true, false, 0u, payload),
          "payload construction failed");

    GuidPrefix sender = make_prefix(0xA1u, 0xB2u, 0xC3u);
    CHECK(r.on_data(sender, payload.data(), static_cast<uint32_t>(payload.size()), 300u),
          "payload with zero GUID should still parse");
    CHECK(obs.new_calls == 1, "expected new peer callback");
    CHECK(obs.last_new == sender,
          "reader should fall back to senderGuidPrefix when participant GUID is zero");
    return true;
}

static bool tc_spdp_9_6_3_1_002(std::string& err) {
    ReaderObs obs{};
    SpdpReader r(on_new_peer, on_removed, &obs);
    std::vector<Octet> payload{};
    const GuidPrefix gp = make_prefix(0x77u, 0x78u, 0x79u);
    CHECK(build_payload(gp, true, false, false, pid::DEFAULT_UNICAST_LOCATOR, payload),
          "payload construction failed");

    CHECK(r.on_data(gp, payload.data(), static_cast<uint32_t>(payload.size()), 500u),
          "SPDP payload with PID_DEFAULT_UNICAST_LOCATOR should be accepted");
    CHECK(r.peer_count() == 1u, "expected one discovered peer");
    CHECK(obs.new_calls == 1, "expected new peer callback once");
    return true;
}

void append_spdp_tests_9_6_2_9_6_3_1(std::vector<TestCase>& tests) {
    tests.push_back({"TC-SPDP-9.6.2-001",
                     "§9.6.2: SPDP reader accepts PL_CDR parameter lists terminated by PID_SENTINEL and skips unknown PID entries.",
                     tc_spdp_9_6_2_001});
    tests.push_back({"TC-SPDP-9.6.2-002",
                     "§9.6.2/§9.6.3.1: PID_PARTICIPANT_GUID is mandatory for SPDP participant identity; missing GUID causes parse rejection.",
                     tc_spdp_9_6_2_002});
    tests.push_back({"TC-SPDP-9.6.2-003",
                     "§9.6.2: a parameter whose padded length reaches 0x10000 is bounded by the "
                     "payload and rejected; the reader never reads past the datagram.",
                     tc_spdp_9_6_2_003});
    tests.push_back({"TC-SPDP-9.6.3.1-001",
                     "§9.6.3.1 robustness: if PID_PARTICIPANT_GUID carries unknown prefix, SPDP reader falls back to RTPS header senderGuidPrefix.",
                     tc_spdp_9_6_3_1_001});
    tests.push_back({"TC-SPDP-9.6.3.1-002",
                     "§9.6.3.1: SPDP reader accepts PID_DEFAULT_UNICAST_LOCATOR (0x0031) for participant default unicast locator data.",
                     tc_spdp_9_6_3_1_002});
}

} // namespace spec_test
