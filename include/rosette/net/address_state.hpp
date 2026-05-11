#pragma once

#include <array>
#include <cstdint>

namespace rosette::net {

/**
 * @brief Lifecycle state of interface address acquisition.
 *
 * Higher-level firmware often needs to know whether an interface is still
 * waiting for configuration, has a usable address, or failed to acquire one.
 * The exact mechanism varies by platform: a POSIX test harness may begin with
 * a static address, a WIZnet board may run an explicit DHCP client, and a
 * future stack may observe callback-driven state from FreeRTOS+TCP or lwIP.
 *
 * Normalizing that lifecycle into a small status enum keeps board firmware and
 * supervisor logic independent from any one network stack's callback model.
 */
enum class AddressStatus : uint8_t {
    inactive = 0u,
    acquiring,
    assigned,
    failed,
};

/**
 * @brief Address acquisition mode selected for an interface.
 */
enum class AddressMode : uint8_t {
    manual = 0u,
    dhcp_v4,
    slaac_v6,
    zeroconf_v4,
};

/**
 * @brief Quality of the currently assigned address.
 *
 * Link-local addresses are useful for local peer discovery and diagnostics but
 * are usually less capable than routed addresses for reaching ROS Agents and
 * other infrastructure.
 */
enum class AddressQuality : uint8_t {
    none = 0u,
    link_local,
    routable,
};

/**
 * @brief Platform-neutral snapshot of interface addressing state.
 *
 * Rosette uses this snapshot as the common reporting surface for address
 * providers. A provider may actively run DHCP, observe a third-party TCP/IP
 * stack, or simply expose a static test-harness address. Higher layers should
 * supervise this snapshot rather than making assumptions about how the address
 * was obtained.
 */
struct AddressSnapshot {
    bool active = false;
    bool assigned = false;
    AddressStatus status = AddressStatus::inactive;
    AddressMode mode = AddressMode::manual;
    AddressQuality quality = AddressQuality::none;
    std::array<uint8_t, 4> ipv4{};
    std::array<uint8_t, 16> ipv6{};
    bool has_ipv4 = false;
    bool has_ipv6 = false;
    uint32_t lease_remaining_s = 0u;
    std::array<std::array<uint8_t, 4>, 2> ntp_servers_v4{};
    uint8_t ntp_server_count = 0u;
};

/**
 * @brief Compare two address snapshots for supervisor-visible changes.
 *
 * Address providers typically update state from polling loops or callback-fed
 * observers. Equality lets higher layers decide whether a step materially
 * changed readiness, address quality, or NTP server information.
 *
 * @param[in] lhs First snapshot to compare.
 * @param[in] rhs Second snapshot to compare.
 * @return `true` when both snapshots describe the same supervisor-visible
 *         address state.
 */
inline bool operator==(const AddressSnapshot& lhs,
                       const AddressSnapshot& rhs) noexcept {
    return lhs.active == rhs.active &&
                 lhs.assigned == rhs.assigned &&
                 lhs.status == rhs.status &&
                 lhs.mode == rhs.mode &&
                 lhs.quality == rhs.quality &&
                 lhs.ipv4 == rhs.ipv4 &&
                 lhs.ipv6 == rhs.ipv6 &&
                 lhs.has_ipv4 == rhs.has_ipv4 &&
                 lhs.has_ipv6 == rhs.has_ipv6 &&
                 lhs.lease_remaining_s == rhs.lease_remaining_s &&
                 lhs.ntp_servers_v4 == rhs.ntp_servers_v4 &&
                 lhs.ntp_server_count == rhs.ntp_server_count;
}

/**
 * @brief Compare two address snapshots for supervisor-visible changes.
 *
 * @param[in] lhs First snapshot to compare.
 * @param[in] rhs Second snapshot to compare.
 * @return `true` when the snapshots differ.
 */
inline bool operator!=(const AddressSnapshot& lhs,
                       const AddressSnapshot& rhs) noexcept {
    return !(lhs == rhs);
}

/**
 * @brief Determine whether an IPv4 address is within the RFC 3927 link-local range.
 *
 * Link-local IPv4 addresses occupy `169.254.0.0/16` and indicate that the
 * interface has connectivity on the local link but not necessarily broader
 * routed reachability.
 *
 * @param[in] ipv4 IPv4 address to classify.
 * @return `true` when the address is link-local; otherwise `false`.
 *
 * References:
 * - RFC 3927 section 2.1.
 */
inline bool IsIpv4LinkLocal(const std::array<uint8_t, 4>& ipv4) noexcept {
    return ipv4[0] == 169u && ipv4[1] == 254u;
}

}    // namespace rosette::net
