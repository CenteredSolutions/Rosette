#pragma once

/**
 * @file net/wiznet/dhcp_provider.hpp
 * @brief The IPv4 `AddressProvider` that runs DHCP on a WIZnet hardware-socket chip.
 *
 * `DhcpAddressProvider` owns a `DhcpClient` and is the only place a DHCP
 * lease reaches the chip: it writes the lease through the `NetInterface` when
 * the client reports `bound`, and removes it when the client reports `lost`.
 * A renewal (`renewed`) writes nothing, so the address stays on the chip
 * through T1 and T2.
 *
 * It runs in one of two roles. As the holder (`begin()`, or `claim()` after a
 * probe) it writes the interface and reads its conflict signal: a conflict on
 * an address it wrote is declined (DHCPDECLINE) and the address removed. As a
 * prober (`begin_probe()`) it runs the same exchanges from whatever source
 * address the chip holds, but writes nothing and leaves the conflict signal
 * to the holder, so an `AddressManager` can look for a DHCP server while the
 * board keeps its IPv4 link-local address.
 *
 * Every `poll()` is non-blocking; the client holds a socket only while an
 * exchange is waiting for a reply.
 *
 *     static wiz::DhcpAddressProvider dhcp{sockets, mac, iface, wiz::dhcp_config("my-host")};
 */

#include <array>
#include <cstdint>

#include "rosette/net/address_provider.hpp"
#include "rosette/net/interface.hpp"
#include "rosette/net/socket.hpp"

#include "dhcp_client.hpp"

namespace rosette::net::wiznet {

/// @brief The default `DhcpClientConfig` with a host name (RFC 2132 §3.14) that must outlive it.
inline DhcpClientConfig dhcp_config(const char* host_name = nullptr) noexcept {
    DhcpClientConfig config{};
    config.host_name = host_name;
    return config;
}

class DhcpAddressProvider : public rosette::net::AddressProvider {
public:
    /**
     * @brief Creates an idle provider.
     *
     * @param[in] sockets  Opens the DHCP client's socket; must outlive the provider.
     * @param[in] mac      The interface MAC.
     * @param[in] iface    Where the lease is written and conflicts are read; must
     *                     outlive the provider.
     * @param[in] config   DHCP ports, timing, and host name.
     */
    DhcpAddressProvider(SocketProvider& sockets, const std::array<uint8_t, 6>& mac,
                        NetInterface& iface = null_net_interface(),
                        const DhcpClientConfig& config = dhcp_config()) noexcept;

    void begin() noexcept override;
    bool begin_probe() noexcept override;
    void claim() noexcept override;
    void stop() noexcept override;

    /**
     * @brief One non-blocking step: react to a conflict, advance DHCP, update the chip.
     *
     * As the holder, a conflict on the written address is declined first.
     * Then the client advances one step, and its event decides the chip
     * write: `bound` writes the lease, `lost` removes it, anything else
     * writes nothing. A prober only advances the client.
     *
     * @param[in] now_ms  Monotonic milliseconds.
     * @return The provider's lease: `bound` with the address while a lease is
     *   held (whether or not it is on the chip yet), `failed` while the
     *   client waits to retry after a failed exchange, a NAK, an expiry, or a
     *   decline, and `acquiring` otherwise. The `duplicate` flag is set for
     *   the one poll that declined an address.
     *
     * Verification:
     * - Correct means: no interface write clears the address during a probe
     *   or a renewal; a holder's conflict yields exactly one DECLINE and one
     *   clear.
     * - Covered by: tests/host/address_upgrade_test.cpp (TC-NET-UPG-001 to
     *   TC-NET-UPG-006).
     *
     * References: RFC 2131 §4.4.1, §4.4.5; RFC 3927 §1.9.
     */
    const AddressLease& poll(uint32_t now_ms) noexcept override;

    const DhcpClient& client() const noexcept { return client_; }

private:
    void write_lease() noexcept;
    void remove_address() noexcept;
    void rebuild(DhcpEvent event, bool duplicate) noexcept;

    DhcpClient client_;
    NetInterface& iface_;
    AddressLease lease_{};
    bool started_ = false;
    bool holder_ = false;
    bool written_ = false;  ///< The client's lease is on the interface.
};

}  // namespace rosette::net::wiznet
