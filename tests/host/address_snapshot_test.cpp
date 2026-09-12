/**
 * @file address_snapshot_test.cpp
 * @brief `AddressSnapshot` / `FamilyAddressSnapshot` equality over the per-family
 *        fields, the `AddressFlags` bitset, and the link-local classifiers.
 */

#include "test_util.hpp"

#include "rosette/net/address_state.hpp"

using namespace rosette::net;

int main() {
    AddressSnapshot a{};
    AddressSnapshot b{};
    check(a == b, "default snapshots compare equal");

    b.v4.has_address = true;
    check(a != b, "a v4 address change is visible");
    a = b;
    check(a == b, "copy compares equal again");

    b.v4.address[3] = 5u;
    check(a != b, "a v4 octet change is visible");
    a = b;

    b.v6.state = AddressPhase::bound;
    check(a != b, "a v6 state change is visible");
    a = b;

    set_flag(b.v4.flags, AddressFlags::duplicate, true);
    check(a != b, "the duplicate flag is part of equality");
    check(b.v4.duplicate() && !a.v4.duplicate(), "duplicate() reads the bit");
    a = b;

    b.ntp_servers_v4[0] = {192u, 168u, 1u, 1u};
    b.ntp_server_count = 1u;
    check(a != b, "an NTP-server change is visible");

    // AddressFlags bit ops
    {
        AddressFlags f = AddressFlags::none;
        set_flag(f, AddressFlags::link_local, true);
        check(has_flag(f, AddressFlags::link_local), "set_flag sets the bit");
        set_flag(f, AddressFlags::routable, true);
        check(has_flag(f, AddressFlags::link_local) && has_flag(f, AddressFlags::routable),
              "two independent bits can be set at once");
        set_flag(f, AddressFlags::link_local, false);
        check(!has_flag(f, AddressFlags::link_local) && has_flag(f, AddressFlags::routable),
              "clearing one bit leaves the other");
    }

    // classify()
    {
        std::array<uint8_t, 16> ll_v4{169u, 254u, 3u, 9u};
        check(classify(AddressFamily::ipv4, ll_v4, true) == AddressFlags::link_local,
              "169.254.x.y classifies link-local");
        std::array<uint8_t, 16> routable_v4{192u, 168u, 1u, 5u};
        check(classify(AddressFamily::ipv4, routable_v4, true) == AddressFlags::routable,
              "192.168.x.y classifies routable");
        check(classify(AddressFamily::ipv4, routable_v4, false) == AddressFlags::none,
              "no address classifies to no flags");
        std::array<uint8_t, 16> ll_v6{0xFEu, 0x80u};
        check(classify(AddressFamily::ipv6, ll_v6, true) == AddressFlags::link_local,
              "fe80:: classifies link-local");
    }

    // classifiers
    check(IsIpv4LinkLocal({169u, 254u, 3u, 9u}), "169.254.x.y is link-local");
    check(!IsIpv4LinkLocal({169u, 253u, 3u, 9u}), "169.253.x.y is not");
    check(IsIpv6LinkLocal({0xFEu, 0x80u}), "fe80:: is link-local");
    check(IsIpv6LinkLocal({0xFEu, 0xB0u}), "feb0:: is still fe80::/10");
    check(!IsIpv6LinkLocal({0x20u, 0x01u}), "2001:: is not link-local");

    return test_summary();
}
