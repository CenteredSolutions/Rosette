/**
 * @file session_supervisor_test.cpp
 * @brief `SessionSupervisor`: when the application's session is created, kept, and rebuilt.
 *
 * The network, the application, and the gate are fakes whose state each test
 * sets directly: `g_net` is the address snapshot and readiness the backend
 * reports, `g_app` counts the application's create and destroy calls, and
 * `FakeGate` stands in for the micro-ROS agent's discovery and ping.
 *
 * References: rosette/ros/session_supervisor.hpp; micro-ROS "Handle
 * reconnections and liveliness" tutorial.
 */

#include <array>
#include <cstdint>

#include "rosette/ros/session_supervisor.hpp"
#include "test_util.hpp"

namespace {

namespace net = rosette::net;
namespace ros = rosette::ros;
using ros::SessionState;

struct FakeNetwork {
    net::AddressSnapshot snapshot{};
    bool ready = false;
    uint32_t polls = 0u;
};

struct FakeApp {
    bool create_ok = true;
    uint32_t creates = 0u;
    uint32_t destroys = 0u;
};

FakeNetwork g_net;
FakeApp g_app;

const net::NetworkWatch kWatch{
    [](uint32_t) { ++g_net.polls; },
    [] { return g_net.ready; },
    []() -> const net::AddressSnapshot& { return g_net.snapshot; },
};

const ros::SessionCallbacks kApp{
    [] {
        ++g_app.creates;
        return g_app.create_ok;
    },
    [] { ++g_app.destroys; },
};

class FakeGate final : public ros::SessionGate {
public:
    bool reachable_ok = true;
    bool alive_ok = true;
    uint32_t reach_calls = 0u;

    bool reachable(uint32_t) noexcept override {
        ++reach_calls;
        return reachable_ok;
    }

    bool alive(uint32_t) noexcept override { return alive_ok; }
};

/// @brief Resets the fakes; the network holds `v4` (zero for none), ready when it has one.
void reset(std::array<uint8_t, 4> v4) {
    g_net = FakeNetwork{};
    g_app = FakeApp{};
    if (v4 != std::array<uint8_t, 4>{}) {
        (void)g_net.snapshot.v4.addresses.add(net::AddressFamily::ipv4,
                                              {v4[0], v4[1], v4[2], v4[3]}, 24u);
        g_net.ready = true;
    }
}

/// @brief Replaces the network's IPv4 address (all zero removes it) and sets readiness to match.
void hold(std::array<uint8_t, 4> v4) {
    g_net.snapshot.v4.addresses = net::AddressList{};
    g_net.ready = false;
    if (v4 != std::array<uint8_t, 4>{}) {
        (void)g_net.snapshot.v4.addresses.add(net::AddressFamily::ipv4,
                                              {v4[0], v4[1], v4[2], v4[3]}, 24u);
        g_net.ready = true;
    }
}

/// @brief Steps every 10 ms from `from` for `span` ms; returns the last state.
SessionState run(ros::SessionSupervisor& s, uint32_t from, uint32_t span) {
    SessionState state = s.state();
    for (uint32_t now = from; now < from + span; now += 10u) {
        state = s.step(now);
    }

    return state;
}

constexpr std::array<uint8_t, 4> kLinkLocal{169u, 254u, 17u, 78u};
constexpr std::array<uint8_t, 4> kLeased{192u, 168u, 32u, 79u};

void tc_waits_for_the_network() {
    reset({});
    ros::SessionSupervisor s{kWatch, kApp};
    check(run(s, 0u, 1000u) == SessionState::wait_network && g_app.creates == 0u,
          "001: no session while the network is not ready");
    check(g_net.polls == 100u, "001: the network is polled on every step while waiting");

    hold(kLeased);
    check(run(s, 1000u, 30u) == SessionState::running && g_app.creates == 1u,
          "001: the session is created once the network is ready");
    check(s.session_address() == kLeased, "001: the session is bound to the held address");

    const uint32_t polls = g_net.polls;
    (void)run(s, 1030u, 100u);
    check(g_net.polls == polls + 10u, "001: the network is polled on every step while running");
}

void tc_gate_decides_reach_and_liveness() {
    reset(kLeased);
    FakeGate gate;
    gate.reachable_ok = false;
    ros::SessionSupervisor s{kWatch, kApp, &gate, {1000u, 5000u}};
    check(run(s, 0u, 2500u) == SessionState::wait_network && g_app.creates == 0u,
          "002: no session while the gate's far end is unreachable");
    check(gate.reach_calls == 3u, "002: an unreachable gate is asked once per create_retry_ms");

    gate.reachable_ok = true;
    check(run(s, 2500u, 1000u) == SessionState::running, "002: reachable gate: session up");

    gate.alive_ok = false;
    (void)run(s, 3500u, 20u);
    check(g_app.destroys == 1u && s.state() == SessionState::wait_network,
          "002: the gate reporting its far end gone destroys the session");
}

void tc_failed_create_is_cleaned_up_and_retried() {
    reset(kLeased);
    g_app.create_ok = false;
    ros::SessionSupervisor s{kWatch, kApp, nullptr, {1000u, 5000u}};
    (void)run(s, 0u, 500u);
    check(g_app.creates == 1u && g_app.destroys == 1u,
          "003: a failed create is followed by destroy, which cleans up the partial session");

    (void)run(s, 500u, 1000u);
    check(g_app.creates == 2u, "003: create is retried after create_retry_ms, not sooner");

    g_app.create_ok = true;
    check(run(s, 1500u, 1000u) == SessionState::running && s.sessions_created() == 1u,
          "003: a later create succeeds");
}

void tc_moved_address_rebuilds() {
    reset(kLinkLocal);
    ros::SessionSupervisor s{kWatch, kApp};
    (void)run(s, 0u, 100u);
    check(s.session_address() == kLinkLocal, "004: first session on the link-local address");

    hold(kLeased);  // the DHCP upgrade replaces link-local in one poll
    (void)run(s, 100u, 100u);
    check(g_app.destroys == 1u && g_app.creates == 2u && s.state() == SessionState::running,
          "004: a moved address destroys the session and creates it again at once");
    check(s.session_address() == kLeased, "004: the new session is bound to the lease");
}

void tc_lost_address_gets_the_grace() {
    reset(kLeased);
    ros::SessionSupervisor s{kWatch, kApp, nullptr, {1000u, 5000u}};
    (void)run(s, 0u, 20000u);

    hold({});  // carrier dropped: every address released
    check(run(s, 20000u, 4000u) == SessionState::running && g_app.destroys == 0u,
          "005: the session survives an address loss shorter than the grace");

    hold(kLeased);
    check(run(s, 24000u, 10000u) == SessionState::running && g_app.creates == 1u,
          "005: the same address back within the grace keeps the session");

    hold({});
    check(run(s, 34000u, 4900u) == SessionState::running,
          "005: the grace is timed from this loss, not an earlier one");
    (void)run(s, 38900u, 200u);
    check(g_app.destroys == 1u && s.state() == SessionState::wait_network,
          "005: a loss longer than the grace destroys the session");

    check(run(s, 39100u, 10000u) == SessionState::wait_network && g_app.creates == 1u,
          "005: no new session until the network is ready again");
}

void tc_unready_network_gets_the_grace() {
    reset(kLeased);
    ros::SessionSupervisor s{kWatch, kApp, nullptr, {1000u, 5000u}};
    (void)run(s, 0u, 20000u);

    g_net.ready = false;  // the address stays, but the firmware's ready_when fails
    check(run(s, 20000u, 4000u) == SessionState::running, "006: within the grace: still running");
    (void)run(s, 24000u, 1100u);
    check(g_app.destroys == 1u, "006: ready_when failing past the grace destroys the session");

    reset({});
    net::AddressSnapshot v6_only{};
    (void)v6_only.v6.addresses.add(net::AddressFamily::ipv6,
                                   {0xFEu, 0x80u, 0u, 0u, 0u, 0u, 0u, 0u, 0x02u, 0x08u, 0xDCu,
                                    0xFFu, 0xFEu, 0x54u, 0x75u, 0x30u},
                                   64u);
    g_net.snapshot = v6_only;
    g_net.ready = true;
    ros::SessionSupervisor v6{kWatch, kApp, nullptr, {1000u, 5000u}};
    check(run(v6, 0u, 20000u) == SessionState::running && g_app.creates == 1u,
          "006: a session created with no IPv4 address is kept while ready_when holds");
}

}  // namespace

int main() {
    tc_waits_for_the_network();
    tc_gate_decides_reach_and_liveness();
    tc_failed_create_is_cleaned_up_and_retried();
    tc_moved_address_rebuilds();
    tc_lost_address_gets_the_grace();
    tc_unready_network_gets_the_grace();
    return test_summary();
}
