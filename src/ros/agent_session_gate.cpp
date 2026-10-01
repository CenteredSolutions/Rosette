/**
 * @file src/ros/agent_session_gate.cpp
 * @brief The micro-ROS agent's discovery, liveness, and time-sync schedule; see
 *        rosette/ros/agent_session_gate.hpp.
 *
 * Every exchange with the agent, a ping or a time synchronization, is a
 * round trip, so each one is also evidence of liveness: an answered
 * synchronization counts as an answered ping, and an unanswered one as a
 * miss. That keeps a failing resynchronization from starving the pings that
 * detect a lost agent.
 */

#include "rosette/ros/agent_session_gate.hpp"

#include <cstdio>

#include "rosette/net/inet.h"

namespace rosette::ros {
namespace {

/// @brief Whether a wrap-safe millisecond deadline has passed.
bool due(uint32_t deadline_ms, uint32_t now_ms) noexcept {
    return static_cast<int32_t>(now_ms - deadline_ms) >= 0;
}

}  // namespace

AgentSessionGate::AgentSessionGate(CustomTransport& transport, AgentLocator& locator,
                                   AgentLink link, AgentGatePolicy policy) noexcept
    : transport_(transport), locator_(locator), link_(link), policy_(policy) {}

bool AgentSessionGate::reachable(uint32_t /*now_ms*/) noexcept {
    if (!attached_) {
        if (!link_.attach(transport_)) {
            std::printf("[agent] transport could not be attached\n");
            return false;
        }

        attached_ = true;
    }

    if (!located_) {
        net::SocketAddress endpoint{};
        if (locator_.resolve(&endpoint, 1u) == 0u) {
            std::printf("[agent] not found\n");
            return false;
        }

        if (!transport_.configure_agent_endpoint(endpoint)) {
            std::printf("[agent] transport rejected the endpoint\n");
            return false;
        }

        agent_ = endpoint;
        located_ = true;
        char text[48] = {};
        std::printf("[agent] located at %s port %u\n",
                    net::inet::ntop(agent_, text, sizeof(text)) != nullptr ? text : "?",
                    static_cast<unsigned>(agent_.port));
    }

    // An agent that stops answering may have moved; resolve again next time.
    if (!link_.ping(policy_.ping_timeout_ms)) {
        std::printf("[agent] no answer to ping\n");
        located_ = false;
        return false;
    }

    session_fresh_ = true;
    return true;
}

bool AgentSessionGate::alive(uint32_t now_ms) noexcept {
    if (session_fresh_) {
        session_fresh_ = false;
        missed_pings_ = 0u;
        sync_due_ = true;
        next_sync_ms_ = now_ms;
        next_ping_ms_ = now_ms + policy_.ping_period_ms;
    }

    const bool sync = sync_due_ && due(next_sync_ms_, now_ms);
    if (!sync && !due(next_ping_ms_, now_ms)) {
        return true;
    }

    bool answered = false;
    if (sync) {
        answered = link_.sync_time(policy_.time_sync_timeout_ms);
        if (!answered) {
            next_sync_ms_ = now_ms + policy_.ping_period_ms;
        } else if (policy_.time_resync_period_ms != 0u) {
            next_sync_ms_ = now_ms + policy_.time_resync_period_ms;
        } else {
            sync_due_ = false;
        }
    } else {
        answered = link_.ping(policy_.ping_timeout_ms);
    }

    next_ping_ms_ = now_ms + policy_.ping_period_ms;
    if (answered) {
        missed_pings_ = 0u;
        return true;
    }

    ++missed_pings_;
    const uint8_t limit = policy_.ping_failures != 0u ? policy_.ping_failures : 1u;
    return missed_pings_ < limit;
}

}  // namespace rosette::ros
