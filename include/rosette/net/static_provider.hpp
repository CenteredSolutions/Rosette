#pragma once

/**
 * @file net/static_provider.hpp
 * @brief `AddressProvider` for an operator-supplied fixed address.
 *
 * Writes the address through the `NetInterface` and binds on the first `poll()`;
 * there is nothing to renew. If the interface reports a duplicate it moves to
 * `failed` — a static address cannot be renumbered, so the manager falls
 * through to the next provider in the list.
 */

#include <array>
#include <cstdint>

#include "address_provider.hpp"
#include "interface.hpp"

namespace rosette::net {

class StaticAddressProvider : public AddressProvider {
public:
    struct Config {
        AddressFamily family = AddressFamily::ipv4;
        std::array<uint8_t, 16> address{};
        uint8_t prefix_len = 24u;
        std::array<uint8_t, 4> gateway_v4{};
    };

    explicit StaticAddressProvider(const Config& config,
                                   NetInterface& iface = null_net_interface()) noexcept
            : config_(config), iface_(iface) {
        reset_lease();
    }

    /// Convenience for an IPv4 static address:
    ///   static net::StaticAddressProvider ip{
    ///       net::StaticAddressProvider::v4({192,168,1,50}, 24, {192,168,1,1}), iface};
    static Config v4(std::array<uint8_t, 4> ip, uint8_t prefix_len = 24u,
                     std::array<uint8_t, 4> gateway = {}) noexcept {
        Config c{};
        c.family = AddressFamily::ipv4;
        c.prefix_len = prefix_len;
        c.gateway_v4 = gateway;
        for (std::size_t i = 0u; i < 4u; ++i) {
            c.address[i] = ip[i];
        }
        return c;
    }

    void begin() noexcept override {
        (void)iface_.conflict();
        started_ = true;
        failed_ = false;
        applied_ = false;
        reset_lease();
        lease_.state = AddressPhase::acquiring;
    }

    void stop() noexcept override {
        if (applied_) {
            iface_.apply(cleared_lease());
        }
        started_ = false;
        failed_ = false;
        applied_ = false;
        reset_lease();
    }

    const AddressLease& poll(uint32_t /*now_ms*/) noexcept override {
        set_flag(lease_.flags, AddressFlags::duplicate, false);
        if (!started_) {
            return lease_;
        }

        if (!failed_ && iface_.conflict()) {
            set_flag(lease_.flags, AddressFlags::duplicate, true);
            failed_ = true;
        }

        if (failed_) {
            if (applied_) {
                iface_.apply(cleared_lease());
                applied_ = false;
            }
            lease_.state = AddressPhase::failed;
            lease_.addresses.clear();
            return lease_;
        }

        if (!applied_) {
            iface_.apply(bound_lease());
            applied_ = true;
        }
        lease_ = bound_lease();
        lease_.state = AddressPhase::bound;
        return lease_;
    }

private:
    AddressLease bound_lease() const noexcept {
        AddressLease l{};
        l.family = config_.family;
        l.source = AddressSource::manual;
        l.state = AddressPhase::bound;
        l.add(config_.address, config_.prefix_len);
        l.gateway_v4 = config_.gateway_v4;
        return l;
    }
    AddressLease cleared_lease() const noexcept {
        AddressLease l{};
        l.family = config_.family;
        l.source = AddressSource::manual;
        return l;
    }
    void reset_lease() noexcept { lease_ = cleared_lease(); }

    Config config_;
    NetInterface& iface_;
    AddressLease lease_{};
    bool started_ = false;
    bool failed_ = false;
    bool applied_ = false;
};

}    // namespace rosette::net
