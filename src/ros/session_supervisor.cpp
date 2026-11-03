// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file src/ros/session_supervisor.cpp
 * @brief The session state machine; see rosette/ros/session_supervisor.hpp.
 */

#include "rosette/ros/session_supervisor.hpp"

#include <cstdio>

namespace rosette::ros {
namespace {

/// @brief Whether a wrap-safe millisecond deadline has passed.
bool due(uint32_t deadline_ms, uint32_t now_ms) noexcept {
    return static_cast<int32_t>(now_ms - deadline_ms) >= 0;
}

/// @brief The IPv4 address a session binds to: the family's primary address, or zero for none.
std::array<uint8_t, 4> session_key(const net::AddressSnapshot& snapshot) noexcept {
    if (!snapshot.v4.has_address()) {
        return {};
    }

    const auto& a = snapshot.v4.address();
    return {a[0], a[1], a[2], a[3]};
}

}  // namespace

SessionSupervisor::SessionSupervisor(const net::NetworkWatch& network, SessionCallbacks app,
                                     SessionGate* gate, SupervisorPolicy policy) noexcept
    : network_(network), app_(app), gate_(gate), policy_(policy) {}

SessionState SessionSupervisor::step(uint32_t now_ms) noexcept {
    network_.poll(now_ms);

    switch (state_) {
        case SessionState::wait_network:
            wait_for_network(now_ms);
            break;
        case SessionState::create_session:
            create_session(now_ms);
            break;
        case SessionState::running:
            supervise_session(now_ms);
            break;
        case SessionState::destroy_session:
            app_.destroy();
            std::printf("[session] destroyed\n");
            state_ = SessionState::wait_network;
            break;
    }

    return state_;
}

void SessionSupervisor::wait_for_network(uint32_t now_ms) noexcept {
    if (!due(next_attempt_ms_, now_ms) || !network_.ready()) {
        return;
    }

    // The gate may block for a reply timeout, so an unreachable far end is
    // asked again only after the retry interval.
    if (gate_ != nullptr && !gate_->reachable(now_ms)) {
        next_attempt_ms_ = now_ms + policy_.create_retry_ms;
        return;
    }

    state_ = SessionState::create_session;
}

void SessionSupervisor::create_session(uint32_t now_ms) noexcept {
    bound_v4_ = session_key(network_.snapshot());
    losing_ = false;
    if (app_.create()) {
        ++sessions_created_;
        std::printf("[session] up on %u.%u.%u.%u (#%lu)\n", bound_v4_[0], bound_v4_[1],
                    bound_v4_[2], bound_v4_[3], static_cast<unsigned long>(sessions_created_));
        state_ = SessionState::running;
        return;
    }

    std::printf("[session] create failed; retrying in %lu ms\n",
                static_cast<unsigned long>(policy_.create_retry_ms));
    app_.destroy();
    next_attempt_ms_ = now_ms + policy_.create_retry_ms;
    state_ = SessionState::wait_network;
}

void SessionSupervisor::supervise_session(uint32_t now_ms) noexcept {
    if (gate_ != nullptr && !gate_->alive(now_ms)) {
        std::printf("[session] far end lost\n");
        state_ = SessionState::destroy_session;
        return;
    }

    // A different address means peers are sending to one the board no longer
    // holds; rebuild at once. A missing address may come back unchanged (a
    // cable reseated, a lease renewed after a blip), so it gets the grace.
    const std::array<uint8_t, 4> held = session_key(network_.snapshot());
    const bool held_any = held != std::array<uint8_t, 4>{};
    if (held_any && held != bound_v4_) {
        std::printf("[session] address moved to %u.%u.%u.%u\n", held[0], held[1], held[2], held[3]);
        state_ = SessionState::destroy_session;
        return;
    }

    const bool address_lost = !held_any && bound_v4_ != std::array<uint8_t, 4>{};
    if (!address_lost && network_.ready()) {
        losing_ = false;
        return;
    }

    if (!losing_) {
        losing_ = true;
        lost_since_ms_ = now_ms;
        std::printf("[session] network lost; holding the session for %lu ms\n",
                    static_cast<unsigned long>(policy_.address_loss_grace_ms));
        return;
    }

    if (due(lost_since_ms_ + policy_.address_loss_grace_ms, now_ms)) {
        state_ = SessionState::destroy_session;
    }
}

}  // namespace rosette::ros
