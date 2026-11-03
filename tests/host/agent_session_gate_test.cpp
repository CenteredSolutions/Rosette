// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file agent_session_gate_test.cpp
 * @brief `AgentSessionGate`: locating the agent, the liveness pings, and the time-sync schedule.
 *
 * The agent is a fake `AgentLink` whose answers each test sets in `g_agent`;
 * the locator and transport are fakes that record what the gate asked of
 * them. Nothing here touches micro-ROS.
 *
 * References: rosette/ros/agent_session_gate.hpp; micro-ROS "Handle
 * reconnections and liveliness" tutorial.
 */

#include <cstdint>

#include "rosette/ros/agent_session_gate.hpp"
#include "test_util.hpp"

namespace {

namespace net = rosette::net;
namespace ros = rosette::ros;

struct FakeAgent {
    bool attach_ok = true;
    bool answers = true;
    uint32_t attaches = 0u;
    uint32_t pings = 0u;
    uint32_t syncs = 0u;
};

FakeAgent g_agent;

const ros::AgentLink kLink{
    [](rosette::CustomTransport&) {
        ++g_agent.attaches;
        return g_agent.attach_ok;
    },
    [](int) {
        ++g_agent.pings;
        return g_agent.answers;
    },
    [](int) {
        ++g_agent.syncs;
        return g_agent.answers;
    },
};

class FakeLocator final : public rosette::AgentLocator {
public:
    bool finds = true;
    uint8_t last_octet = 10u;
    uint32_t resolves = 0u;

    size_t resolve(net::SocketAddress* out, size_t capacity) noexcept override {
        ++resolves;
        if (!finds || capacity == 0u) {
            return 0u;
        }

        out[0] = net::make_socket_address_v4({192u, 168u, 32u, last_octet}, 8888u);
        return 1u;
    }
};

class FakeTransport final : public rosette::CustomTransport {
public:
    net::SocketAddress endpoint{};

    bool configure_agent_endpoint(const net::SocketAddress& e) noexcept override {
        endpoint = e;
        return true;
    }

    bool open() noexcept override { return true; }

    bool close() noexcept override { return true; }

    size_t write(const uint8_t*, size_t, uint8_t*) noexcept override { return 0u; }

    size_t read(uint8_t*, size_t, int, uint8_t*) noexcept override { return 0u; }
};

/// @brief Calls alive() every 10 ms from `from` for `span` ms; returns false at the first false.
bool run_alive(ros::AgentSessionGate& gate, uint32_t from, uint32_t span) {
    for (uint32_t now = from; now < from + span; now += 10u) {
        if (!gate.alive(now)) {
            return false;
        }
    }

    return true;
}

void tc_locates_then_pings() {
    g_agent = FakeAgent{};
    FakeLocator locator;
    FakeTransport transport;
    ros::AgentSessionGate gate{transport, locator, kLink};

    locator.finds = false;
    check(!gate.reachable(0u) && g_agent.pings == 0u,
          "001: nothing located, so no ping and not reachable");
    check(g_agent.attaches == 1u, "001: the transport is attached before anything else");

    locator.finds = true;
    check(gate.reachable(1000u) && g_agent.pings == 1u, "001: located and answered, so reachable");
    check(transport.endpoint.address[3] == 10u && transport.endpoint.port == 8888u,
          "001: the transport holds the located endpoint");
    check(g_agent.attaches == 1u, "001: the transport is attached only once");
}

void tc_follows_an_agent_that_moved() {
    g_agent = FakeAgent{};
    FakeLocator locator;
    FakeTransport transport;
    ros::AgentSessionGate gate{transport, locator, kLink};

    (void)gate.reachable(0u);
    (void)gate.reachable(1000u);
    check(locator.resolves == 1u, "002: an answering endpoint is kept, not resolved again");

    g_agent.answers = false;
    check(!gate.reachable(2000u) && !gate.located(),
          "002: an unanswered ping forgets the endpoint");

    g_agent.answers = true;
    locator.last_octet = 11u;
    check(gate.reachable(3000u) && locator.resolves == 2u, "002: the next attempt resolves again");
    check(transport.endpoint.address[3] == 11u,
          "002: the transport follows the agent's new address");
}

void tc_reports_the_agent_gone_after_n_misses() {
    g_agent = FakeAgent{};
    FakeLocator locator;
    FakeTransport transport;
    ros::AgentGatePolicy policy{};
    policy.time_resync_period_ms = 0u;
    ros::AgentSessionGate gate{transport, locator, kLink, policy};
    (void)gate.reachable(0u);

    check(run_alive(gate, 0u, 10000u), "003: an answering agent stays alive");
    const uint32_t pings = g_agent.pings - 1u;  // less the one from reachable()
    check(pings == 4u, "003: pings are ping_period_ms apart (4 in 10 s after the initial sync)");

    g_agent.answers = false;
    check(gate.alive(12000u) && gate.alive(14000u), "003: two misses are tolerated");
    g_agent.answers = true;
    check(gate.alive(16000u), "003: an answer resets the count");
    g_agent.answers = false;
    check(gate.alive(18000u) && gate.alive(20000u) && !gate.alive(22000u),
          "003: the third consecutive miss reports the agent gone");
}

void tc_syncs_on_every_session_and_on_the_resync_period() {
    g_agent = FakeAgent{};
    FakeLocator locator;
    FakeTransport transport;
    ros::AgentGatePolicy policy{};
    policy.time_resync_period_ms = 5000u;
    ros::AgentSessionGate gate{transport, locator, kLink, policy};

    (void)gate.reachable(0u);
    (void)gate.alive(100u);
    check(g_agent.syncs == 1u, "004: the first alive() of a session syncs the clock");

    (void)run_alive(gate, 110u, 5000u);
    check(g_agent.syncs == 2u, "004: the clock is resynced after time_resync_period_ms");

    (void)gate.reachable(20000u);
    (void)gate.alive(20010u);
    check(g_agent.syncs == 3u, "004: a new session syncs again at once");
}

void tc_a_failing_sync_does_not_starve_the_pings() {
    g_agent = FakeAgent{};
    FakeLocator locator;
    FakeTransport transport;
    ros::AgentGatePolicy policy{};
    policy.time_resync_period_ms = 0u;
    ros::AgentSessionGate gate{transport, locator, kLink, policy};
    (void)gate.reachable(0u);

    // The agent vanishes as the session comes up: every sync retry is a miss.
    g_agent.answers = false;
    check(!run_alive(gate, 0u, 10000u), "005: failed syncs count as misses and end the session");
    check(g_agent.syncs == 3u, "005: exactly ping_failures exchanges before the agent is gone");
}

}  // namespace

int main() {
    tc_locates_then_pings();
    tc_follows_an_agent_that_moved();
    tc_reports_the_agent_gone_after_n_misses();
    tc_syncs_on_every_session_and_on_the_resync_period();
    tc_a_failing_sync_does_not_starve_the_pings();
    return test_summary();
}
