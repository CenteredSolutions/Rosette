/**
 * @file src/net/wiznet/dhcp_provider.cpp
 * @brief The WIZnet DHCP address provider; see rosette/net/wiznet/dhcp_provider.hpp.
 */

#include "rosette/net/wiznet/dhcp_provider.hpp"

namespace rosette::net::wiznet {
namespace {

std::array<uint8_t, 16> v4_bytes(const std::array<uint8_t, 4>& ip) noexcept {
    return {ip[0], ip[1], ip[2], ip[3]};
}

/// The client's lease as an `AddressLease` (empty when @p lease is null).
AddressLease to_address_lease(const DhcpLease* lease) noexcept {
    AddressLease l{};
    l.family = AddressFamily::ipv4;
    l.source = AddressSource::dhcp;
    if (lease == nullptr) {
        return l;
    }

    l.add(v4_bytes(lease->ip), Ipv4MaskToPrefix(lease->subnet), lease->lease_s, lease->lease_s);
    l.gateway_v4 = lease->gateway;
    l.ntp_count = lease->ntp_count > 2u ? 2u : lease->ntp_count;
    for (uint8_t i = 0u; i < l.ntp_count; ++i) {
        l.ntp_v4[i] = lease->ntp[i];
    }

    return l;
}

}  // namespace

DhcpAddressProvider::DhcpAddressProvider(SocketProvider& sockets, const std::array<uint8_t, 6>& mac,
                                         NetInterface& iface,
                                         const DhcpClientConfig& config) noexcept
    : client_(sockets, mac, config), iface_(iface), lease_(to_address_lease(nullptr)) {}

void DhcpAddressProvider::begin() noexcept {
    stop();
    (void)iface_.conflict();  // a latched conflict predates this provider's address
    started_ = true;
    holder_ = true;
    client_.start();
    lease_.state = AddressPhase::acquiring;
}

bool DhcpAddressProvider::begin_probe() noexcept {
    stop();
    started_ = true;
    client_.start();
    lease_.state = AddressPhase::acquiring;
    return true;
}

void DhcpAddressProvider::claim() noexcept {
    if (!started_ || holder_) {
        return;
    }

    holder_ = true;
    (void)iface_.conflict();  // a latched conflict belongs to the previous holder's address
    if (client_.has_lease()) {
        write_lease();
    }
}

void DhcpAddressProvider::stop() noexcept {
    if (written_) {
        remove_address();
    }

    client_.stop();
    started_ = false;
    holder_ = false;
    lease_ = to_address_lease(nullptr);
}

const AddressLease& DhcpAddressProvider::poll(uint32_t now_ms) noexcept {
    if (!started_) {
        return lease_;
    }

    // Only the holder reads the conflict signal. A conflict with no address of
    // ours on the chip is stale and dropped.
    bool duplicate = false;
    if (holder_ && iface_.conflict() && written_) {
        client_.decline(now_ms);
        remove_address();
        duplicate = true;
    }

    const DhcpEvent event = client_.poll(now_ms);
    if (holder_ && event == DhcpEvent::bound) {
        write_lease();
    } else if (holder_ && event == DhcpEvent::lost && written_) {
        remove_address();
    }

    rebuild(event, duplicate);
    return lease_;
}

void DhcpAddressProvider::write_lease() noexcept {
    iface_.apply(to_address_lease(&client_.lease()));
    written_ = true;
}

void DhcpAddressProvider::remove_address() noexcept {
    iface_.apply(to_address_lease(nullptr));
    written_ = false;
}

void DhcpAddressProvider::rebuild(DhcpEvent event, bool duplicate) noexcept {
    lease_ = to_address_lease(client_.has_lease() ? &client_.lease() : nullptr);
    if (client_.has_lease()) {
        lease_.state = AddressPhase::bound;
    } else if (event == DhcpEvent::failed || event == DhcpEvent::lost
               || client_.state() == DhcpClientState::init_wait) {
        lease_.state = AddressPhase::failed;
    } else {
        lease_.state = AddressPhase::acquiring;
    }

    set_flag(lease_.flags, AddressFlags::duplicate, duplicate);
}

}  // namespace rosette::net::wiznet
