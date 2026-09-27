/**
 * @file src/net/address_candidate.cpp
 * @brief Self-assigned address candidates; see rosette/net/address_candidate.hpp.
 */

#include "rosette/net/address_candidate.hpp"

#include "rosette/support/fnv1a.hpp"

namespace rosette::net {

std::array<uint8_t, 16> ipv6_address_candidate(const std::array<uint8_t, 16>& prefix,
                                               const std::array<uint8_t, 6>& mac,
                                               uint8_t conflicts) noexcept {
    std::array<uint8_t, 8> iid = modified_eui64(mac);
    if (conflicts != 0u || is_reserved_interface_id(iid)) {
        std::array<uint8_t, 17> input{0x06u, conflicts};
        for (std::size_t i = 0u; i < 8u; ++i) {
            input[3u + i] = prefix[i];
        }

        for (std::size_t i = 0u; i < 6u; ++i) {
            input[11u + i] = mac[i];
        }

        for (uint8_t salt = 0u;; ++salt) {
            input[2] = salt;
            const uint64_t h = rosette::support::fnv1a64(input.data(), input.size());
            for (std::size_t i = 0u; i < 8u; ++i) {
                iid[i] = static_cast<uint8_t>(h >> (56u - 8u * i));
            }

            if (!is_reserved_interface_id(iid)) {
                break;
            }
        }
    }

    std::array<uint8_t, 16> address{};
    for (std::size_t i = 0u; i < 8u; ++i) {
        address[i] = prefix[i];
        address[8u + i] = iid[i];
    }

    return address;
}

std::array<uint8_t, 4> ipv4_link_local_candidate(const std::array<uint8_t, 6>& mac,
                                                 uint32_t conflicts) noexcept {
    std::array<uint8_t, 11> input{0x04u};
    for (std::size_t i = 0u; i < 4u; ++i) {
        input[1u + i] = static_cast<uint8_t>(conflicts >> (8u * i));
    }

    for (std::size_t i = 0u; i < 6u; ++i) {
        input[5u + i] = mac[i];
    }

    const uint64_t h = rosette::support::fnv1a64(input.data(), input.size());
    return {169u, 254u, static_cast<uint8_t>(1u + (h >> 32u) % 254u), static_cast<uint8_t>(h)};
}

}  // namespace rosette::net
