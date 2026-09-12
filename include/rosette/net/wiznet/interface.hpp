#pragma once

/**
 * @file net/wiznet/interface.hpp
 * @brief `NetInterface` for a WIZnet hardware-socket chip.
 *
 * `apply()` programs `SIPR` / `SUBR` / `GAR` (writing `SIPR` makes the chip emit
 * a gratuitous ARP — the RFC 3927 announce), or zeroes them when the lease has
 * no address. `conflict()` drains the chip's IP-conflict latch, which
 * `WiznetDevice` sets under the bus lock from either the FreeRTOS ISR-dispatch
 * task (`service_socket_interrupts()`) or a superloop register read.
 *
 * IPv6 `apply()` is a no-op seam until the W6100/W6300 hardware-IPv6 path lands.
 */

#include <array>
#include <cstdint>

#include "rosette/net/address_state.hpp"
#include "rosette/net/interface.hpp"

#include "device.hpp"

namespace rosette::net::wiznet {

template <typename Chip, typename Bus>
class WiznetInterface : public rosette::net::NetInterface {
public:
    using device_type = WiznetDevice<Chip, Bus>;

    explicit WiznetInterface(device_type& device) noexcept : device_(device) {}

    void apply(const rosette::net::AddressLease& lease) noexcept override {
        if (lease.family != rosette::net::AddressFamily::ipv4) {
            return;    // hardware IPv6 is a future seam
        }
        if (!lease.has_address) {
            device_.configure_ipv4({}, {}, {});
            return;
        }
        const std::array<uint8_t, 4> ip{lease.address[0], lease.address[1],
                                        lease.address[2], lease.address[3]};
        device_.configure_ipv4(ip, rosette::net::Ipv4PrefixToMask(lease.prefix_len),
                               lease.gateway_v4);
    }

    bool conflict() noexcept override {
        (void)device_.poll_common_interrupt();    // superloop refresh; ~free on RTOS
        return device_.take_ip_conflict();
    }

    rosette::net::LinkState link_state() noexcept override {
        return device_.is_link_up() ? rosette::net::LinkState::up
                                    : rosette::net::LinkState::down;
    }

private:
    device_type& device_;
};

}    // namespace rosette::net::wiznet
