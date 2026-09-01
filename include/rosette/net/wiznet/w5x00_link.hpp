/**
 * @file net/wiznet/w5x00_link.hpp
 * @brief WIZnet chip bring-up: buffers, PHY link, and DHCP or static addressing.
 *
 * Construct one `WiznetLink` after the board has registered the ioLibrary SPI
 * callbacks and reset the chip (`board_init()`), then:
 *
 *   rosette::net::wiznet::WiznetLink link(mac);
 *   while (!link.begin()) { rosette_platform_delay_ms(200); }   // waits for PHY + DHCP
 *   // link.ipv4() is now valid; call link.poll() in the main loop for lease renewal
 *
 * `begin()` runs the DHCP handshake to completion (or applies the static
 * fallback). `poll()` keeps the lease alive and must be called roughly once a
 * second from the superloop.
 */

#pragma once

#include <array>
#include <cstdint>

namespace rosette::net::wiznet {

/**
 * @brief Addressing mode for `WiznetLink`.
 */
enum class LinkMode : uint8_t {
    dhcp,     ///< Request an address from a DHCP server; fall back to `static_ip` on failure.
    static_ip ///< Use the supplied static address directly.
};

/**
 * @brief Static IPv4 settings used for `LinkMode::static_ip` or DHCP fallback.
 */
struct StaticAddress {
    std::array<uint8_t, 4> ip{192u, 168u, 1u, 200u};
    std::array<uint8_t, 4> netmask{255u, 255u, 255u, 0u};
    std::array<uint8_t, 4> gateway{192u, 168u, 1u, 1u};
    std::array<uint8_t, 4> dns{8u, 8u, 8u, 8u};
};

/**
 * @brief Owns the chip's network configuration lifecycle.
 */
class WiznetLink {
public:
    /**
     * @brief Prepares a link with the given MAC and addressing policy.
     *
     * @param[in] mac The 6-byte hardware address (see `board_get_mac_address`).
     * @param[in] mode DHCP or static.
     * @param[in] fallback Static settings for `static_ip` mode or DHCP failure.
     * @param[in] dhcp_socket Hardware socket number DHCP borrows during `begin()`.
     */
    WiznetLink(const std::array<uint8_t, 6>& mac,
               LinkMode mode = LinkMode::dhcp,
               const StaticAddress& fallback = {},
               uint8_t dhcp_socket = 3u) noexcept;

    WiznetLink(const WiznetLink&) = delete;
    WiznetLink& operator=(const WiznetLink&) = delete;

    /**
     * @brief Initializes chip buffers, waits for PHY link, and assigns an address.
     *
     * Idempotent: returns `true` immediately once the link is already up.
     * Blocks (with `rosette_platform_delay_ms`) through the PHY and DHCP waits,
     * up to the internal timeouts, then returns whether an address is assigned.
     *
     * @return `true` when the chip has a usable IPv4 address.
     */
    bool begin() noexcept;

    /**
     * @brief Services the DHCP lease timer. Call about once per second.
     */
    void poll() noexcept;

    /**
     * @brief Whether an IPv4 address is currently assigned.
     */
    bool ready() const noexcept { return ready_; }

    /**
     * @brief The current IPv4 address (zeros until `ready()`).
     */
    std::array<uint8_t, 4> ipv4() const noexcept { return ip_; }

private:
    void apply_netinfo(const std::array<uint8_t, 4>& ip,
                       const std::array<uint8_t, 4>& sn,
                       const std::array<uint8_t, 4>& gw,
                       const std::array<uint8_t, 4>& dns,
                       bool from_dhcp) noexcept;

    std::array<uint8_t, 6> mac_{};
    LinkMode mode_;
    StaticAddress fallback_;
    uint8_t dhcp_socket_;
    std::array<uint8_t, 4> ip_{};
    uint32_t last_tick_ms_ = 0u;
    bool chip_ready_ = false;
    bool ready_ = false;
    bool dhcp_started_ = false;
};

}  // namespace rosette::net::wiznet
