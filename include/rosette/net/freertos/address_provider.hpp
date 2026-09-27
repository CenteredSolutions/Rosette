#pragma once

/**
 * @file net/freertos/address_provider.hpp
 * @brief `AddressProvider`s that observe FreeRTOS+TCP's own addressing.
 *
 * FreeRTOS+TCP already runs DHCP, RFC 3927 AutoIP fallback
 * (`ipconfigDHCP_FALL_BACK_AUTO_IP`), IPv6 link-local, RA / SLAAC, and DAD.
 * Rosette does not re-implement any of that here — this provider only
 * translates the stack's endpoint state into an `AddressLease`. Conflict / DAD
 * reaction is the stack's job on this path, so it needs no `NetInterface`.
 *
 * One observer per family: give the v4 one to a `v4` `AddressManager` and the
 * v6 one to a `v6` `AddressManager` (each list is then just this single
 * provider). Wire `notify_network_event()` into
 * `vApplicationIPNetworkEventHook_Multi`.
 */

#include <array>
#include <cstdint>

extern "C" {
#include "FreeRTOS.h"
#include "FreeRTOS_IP.h"
}

#include "rosette/net/address_provider.hpp"
#include "rosette/net/address_state.hpp"
#include "rosette/platform/host_runtime.hpp"

namespace rosette::net::freertos {

class FreeRtosObserver : public rosette::net::AddressProvider {
public:
    explicit FreeRtosObserver(
            rosette::net::AddressFamily family = rosette::net::AddressFamily::ipv4) noexcept
            : family_(family) {
        lease_.family = family_;
    }

    void begin() noexcept override {
        started_ = true;
        rebuild();
    }

    void stop() noexcept override {
        started_ = false;
        lease_ = rosette::net::AddressLease{};
        lease_.family = family_;
    }

    const rosette::net::AddressLease& poll(uint32_t /*now_ms*/) noexcept override {
        rebuild();
        return lease_;
    }

    /// Call from `vApplicationIPNetworkEventHook_Multi`.
    static void notify_network_event(eIPCallbackEvent_t event,
                                     struct xNetworkEndPoint* /*endpoint*/) noexcept {
        net_up_ = (event == eNetworkUp);
    }

private:
    void rebuild() noexcept {
        using namespace rosette::net;
        lease_ = AddressLease{};
        lease_.family = family_;

        if (family_ == AddressFamily::ipv4) {
            const uint32_t v4 = rosette_platform_get_ipv4_addr();
            if (v4 != 0u) {
                const std::array<uint8_t, 16> a{
                    static_cast<uint8_t>(v4 >> 24), static_cast<uint8_t>(v4 >> 16),
                    static_cast<uint8_t>(v4 >> 8), static_cast<uint8_t>(v4)};
                lease_.source = IsIpv4LinkLocal({a[0], a[1], a[2], a[3]}) ? AddressSource::zeroconf
                                                                          : AddressSource::dhcp;
                lease_.add(a, 0u);
                lease_.state = AddressPhase::bound;
            } else {
                lease_.source = AddressSource::dhcp;
                lease_.state = net_up_ ? AddressPhase::acquiring : AddressPhase::idle;
            }
        } else {
            std::array<uint8_t, 16> v6{};
            if (rosette_platform_get_ipv6_addr(v6.data())) {
                lease_.source = AddressSource::slaac;    // link_local flag distinguishes fe80::
                lease_.add(v6, 64u);
                lease_.state = AddressPhase::bound;
            } else {
                lease_.source = AddressSource::slaac;
                lease_.state = net_up_ ? AddressPhase::acquiring : AddressPhase::idle;
            }
        }
        if (!started_) {
            lease_.state = AddressPhase::idle;
        }
    }

    rosette::net::AddressFamily family_;
    rosette::net::AddressLease lease_{};
    bool started_ = false;
    inline static volatile bool net_up_ = false;
};

}    // namespace rosette::net::freertos
