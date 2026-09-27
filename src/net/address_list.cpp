/**
 * @file src/net/address_list.cpp
 * @brief `AddressList` insertion and primary-address selection; see rosette/net/address_state.hpp.
 */

#include "rosette/net/address_state.hpp"

namespace rosette::net {
namespace {

/// RFC 6724 rules 2 then 3 for a global-scope destination: scope, then preference.
int rank(const AddressEntry& e) noexcept {
    return (e.routable() ? 2 : 0) + (e.deprecated() ? 0 : 1);
}

}  // namespace

bool AddressList::add(AddressFamily family, const std::array<uint8_t, 16>& address,
                      uint8_t prefix_len, uint32_t valid_s, uint32_t preferred_s,
                      bool deprecated) noexcept {
    if (count_ >= kMaxFamilyAddresses) {
        return false;
    }

    AddressEntry& e = entries_[count_++];
    e.address = address;
    e.prefix_len = prefix_len;
    e.flags = classify(family, address, true);
    set_flag(e.flags, AddressFlags::deprecated, deprecated);
    e.valid_lifetime_s = valid_s;
    e.preferred_lifetime_s = preferred_s;
    return true;
}

const AddressEntry& AddressList::primary() const noexcept {
    std::size_t best = 0u;
    for (std::size_t i = 1u; i < count_; ++i) {
        if (rank(entries_[i]) > rank(entries_[best])) {
            best = i;
        }
    }

    return entries_[best];
}

}  // namespace rosette::net
