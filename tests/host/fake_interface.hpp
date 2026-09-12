#pragma once

#include <array>
#include <cstdint>

#include "rosette/net/interface.hpp"

/// Scriptable `NetInterface` for provider tests.
struct FakeInterface : rosette::net::NetInterface {
    int applies = 0;
    int clears = 0;
    bool pending_conflict = false;
    bool link = true;
    std::array<uint8_t, 16> last_address{};
    bool has_address = false;

    void apply(const rosette::net::AddressLease& lease) noexcept override {
        ++applies;
        has_address = lease.has_address;
        last_address = lease.address;
        if (!lease.has_address) {
            ++clears;
        }
    }
    bool conflict() noexcept override {
        const bool c = pending_conflict;
        pending_conflict = false;
        return c;
    }
    rosette::net::LinkState link_state() noexcept override {
        return link ? rosette::net::LinkState::up : rosette::net::LinkState::down;
    }

    void signal_conflict() noexcept { pending_conflict = true; }
};
