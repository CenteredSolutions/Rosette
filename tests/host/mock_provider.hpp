#pragma once

#include <array>
#include <cstdint>

#include "rosette/net/address_provider.hpp"

/// Scriptable `AddressProvider` for `AddressManager` / `NetworkAddress` tests.
struct MockProvider : rosette::net::AddressProvider {
    rosette::net::AddressLease lease_{};
    int begins = 0;
    int stops = 0;
    int polls = 0;
    bool started = false;

    explicit MockProvider(
            rosette::net::AddressFamily f = rosette::net::AddressFamily::ipv4) noexcept {
        lease_.family = f;
        lease_.source = (f == rosette::net::AddressFamily::ipv4)
                                ? rosette::net::AddressSource::dhcp
                                : rosette::net::AddressSource::slaac;
    }

    void begin() noexcept override {
        started = true;
        ++begins;
        if (lease_.state == rosette::net::AddressPhase::idle) {
            lease_.state = rosette::net::AddressPhase::acquiring;
        }
    }
    void stop() noexcept override {
        started = false;
        ++stops;
        const auto fam = lease_.family;
        const auto source = lease_.source;
        lease_ = rosette::net::AddressLease{};
        lease_.family = fam;
        lease_.source = source;
    }
    const rosette::net::AddressLease& poll(uint32_t /*now_ms*/) noexcept override {
        ++polls;
        return lease_;
    }

    // ── scripting helpers ───────────────────────────────────────────────────
    void bind(std::array<uint8_t, 4> ip = {10u, 0u, 0u, 7u}) noexcept {
        lease_.state = rosette::net::AddressPhase::bound;
        lease_.addresses.clear();
        lease_.add({ip[0], ip[1], ip[2], ip[3]}, 24u);
    }
    void bind_v6() noexcept {
        lease_.state = rosette::net::AddressPhase::bound;
        lease_.addresses.clear();
        lease_.add({0xFEu, 0x80u, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1u}, 64u);
    }
    void set_phase(rosette::net::AddressPhase p) noexcept {
        lease_.state = p;
        if (p != rosette::net::AddressPhase::bound) {
            lease_.addresses.clear();
        }
    }
    void signal_conflict() noexcept {
        rosette::net::set_flag(lease_.flags, rosette::net::AddressFlags::duplicate, true);
    }
    void clear_conflict() noexcept {
        rosette::net::set_flag(lease_.flags, rosette::net::AddressFlags::duplicate, false);
    }
};
