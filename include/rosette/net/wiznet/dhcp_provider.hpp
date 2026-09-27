#pragma once

/**
 * @file net/wiznet/dhcp_provider.hpp
 * @brief `AddressProvider` wrapping the superloop `DhcpPoller`.
 *
 * `begin()` arms one bounded DHCP exchange that the first `poll()` runs;
 * subsequent `poll()`s pump `DhcpPoller::poll()` (renewal at T1, retry after a
 * failure) and, on a conflict from the `NetInterface`, run
 * `DhcpPoller::handle_conflict()` (DHCPDECLINE + fresh DISCOVER).
 *
 * The address write is routed through `NetInterface::apply()` (via
 * `DhcpProtocol::set_apply`) so every address change in the system goes through
 * one path. A held lease that has aged past its validity is dropped so the
 * manager can fall through.
 */

#include <array>
#include <cstdint>

#include "rosette/net/address_provider.hpp"
#include "rosette/net/interface.hpp"

#include "device.hpp"
#include "dhcp.hpp"
#include "executor.hpp"
#include "socket_provider.hpp"

namespace rosette::net::wiznet {

/**
 * @brief `DhcpConfig` tuned for the cooperative `poll()` path: a short exchange
 *        timeout so a `poll()` never stalls the superloop for long.
 *
 *     static wiz::DhcpAddressProvider<Chip, Bus> dhcp{device, sockets, iface};
 *     static wiz::DhcpAddressProvider<Chip, Bus> dhcp{device, sockets, iface,
 *                                                     wiz::dhcp_config("my-host")};
 */
inline DhcpConfig dhcp_config(const char* hostname = nullptr) noexcept {
    DhcpConfig c{};
    c.exchange_timeout_ms = 10000u;
    c.retry_delay_ms = 15000u;
    c.host_name = hostname;
    return c;
}

template <typename Chip, typename Bus,
          template <typename> class ExecutorT = DefaultExecutor>
class DhcpAddressProvider : public rosette::net::AddressProvider {
public:
    using device_type = WiznetDevice<Chip, Bus>;
    using provider_type = WiznetSocketProvider<Chip, Bus, ExecutorT>;
    using poller_type = DhcpPoller<Chip, Bus, ExecutorT>;

    DhcpAddressProvider(device_type& device, provider_type& sockets,
                        rosette::net::NetInterface& iface =
                                rosette::net::null_net_interface(),
                        const DhcpConfig& config = dhcp_config()) noexcept
            : poller_(device, sockets, config), iface_(iface) {
        reset_lease();
        poller_.set_apply(&on_apply, this);
    }

    void begin() noexcept override {
        (void)iface_.conflict();
        started_ = true;
        need_acquire_ = true;
        has_lease_ = false;
        reset_lease();
        lease_.state = rosette::net::AddressPhase::acquiring;
    }

    void stop() noexcept override {
        started_ = false;
        need_acquire_ = false;
        has_lease_ = false;
        reset_lease();
    }

    const rosette::net::AddressLease& poll(uint32_t now_ms) noexcept override {
        if (!started_) {
            return lease_;
        }
        rosette::net::set_flag(lease_.flags, rosette::net::AddressFlags::duplicate, false);

        if (iface_.conflict()) {
            poller_.handle_conflict();
            rosette::net::set_flag(lease_.flags, rosette::net::AddressFlags::duplicate, true);
            need_acquire_ = false;
        } else if (need_acquire_) {
            need_acquire_ = false;
            (void)poller_.acquire();    // one bounded exchange (config.exchange_timeout_ms)
        } else {
            poller_.poll();            // renew at T1, retry after a failure
        }

        rebuild(now_ms);
        return lease_;
    }

    poller_type& poller() noexcept { return poller_; }

private:
    /// `DhcpProtocol` applier — every DHCP address write lands here.
    static void on_apply(const DhcpLease& dl, void* user) noexcept {
        using namespace rosette::net;
        auto* self = static_cast<DhcpAddressProvider*>(user);
        AddressLease l = self->lease_.empty();
        l.source = AddressSource::dhcp;
        if (dl.valid) {
            l.add(v4_bytes(dl.ip), Ipv4MaskToPrefix(dl.subnet), dl.lease_seconds, dl.lease_seconds);
            l.gateway_v4 = dl.gateway;
        }
        self->iface_.apply(l);
    }

    static std::array<uint8_t, 16> v4_bytes(const std::array<uint8_t, 4>& ip) noexcept {
        return {ip[0], ip[1], ip[2], ip[3]};
    }

    void reset_lease() noexcept {
        lease_ = rosette::net::AddressLease{};
        lease_.family = rosette::net::AddressFamily::ipv4;
        lease_.source = rosette::net::AddressSource::dhcp;
    }

    bool lease_expired(const DhcpLease& dl, uint32_t now_ms) const noexcept {
        if (dl.lease_seconds == 0u) {
            return false;
        }
        return (now_ms - lease_acquired_ms_) >= (dl.lease_seconds * 1000u);
    }

    void rebuild(uint32_t now_ms) noexcept {
        using namespace rosette::net;
        const bool conflict_edge = lease_.duplicate();

        DhcpLease dl{};
        if (poller_.lease(dl) && dl.valid && !lease_expired(dl, now_ms)) {
            if (!has_lease_) {
                lease_acquired_ms_ = now_ms;
                has_lease_ = true;
            }
            lease_.state = AddressPhase::bound;
            lease_.addresses.clear();
            lease_.add(v4_bytes(dl.ip), Ipv4MaskToPrefix(dl.subnet), dl.lease_seconds,
                       dl.lease_seconds);
            lease_.gateway_v4 = dl.gateway;
            lease_.ntp_count = dl.ntp_count > 2u ? 2u : dl.ntp_count;
            for (uint8_t i = 0u; i < lease_.ntp_count; ++i) {
                lease_.ntp_v4[i] = dl.ntp[i];
            }
        } else {
            has_lease_ = false;
            lease_.addresses.clear();
            lease_.ntp_count = 0u;
            // `failed` = an exchange completed unsuccessfully → let the manager
            // fall through now instead of waiting out the stage timeout.
            lease_.state = (poller_.state() == DhcpState::failed)
                                   ? AddressPhase::failed
                                   : AddressPhase::acquiring;
        }
        set_flag(lease_.flags, AddressFlags::duplicate, conflict_edge);
    }

    poller_type poller_;
    rosette::net::NetInterface& iface_;
    rosette::net::AddressLease lease_{};
    uint32_t lease_acquired_ms_ = 0u;
    bool started_ = false;
    bool need_acquire_ = false;
    bool has_lease_ = false;
};

}    // namespace rosette::net::wiznet
