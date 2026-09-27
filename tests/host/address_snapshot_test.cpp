/**
 * @file address_snapshot_test.cpp
 * @brief `AddressSnapshot` / `FamilyAddressSnapshot` equality over the per-family
 *        fields, the `AddressFlags` bitset, the link-local classifiers, and
 *        primary-address selection (TC-NET-6724-001).
 */

#include "test_util.hpp"

#include "rosette/net/address_state.hpp"

using namespace rosette::net;

/// TC-NET-6724-001: primary-address selection.
void tc_net_6724_001() {
    const std::array<uint8_t, 16> ll{0xFEu, 0x80u, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1u};
    const std::array<uint8_t, 16> g1{0x20u, 0x01u, 0x0Du, 0xB8u, 0, 0, 0, 0,
                                     0,     0,     0,     0,     0, 0, 0, 1u};
    const std::array<uint8_t, 16> g2{0x20u, 0x01u, 0x0Du, 0xB8u, 0, 0, 0, 0,
                                     0,     0,     0,     0,     0, 0, 0, 2u};

    AddressList empty{};
    check(empty.primary().address == std::array<uint8_t, 16>{}, "empty list: all-zero primary");

    AddressList l{};
    l.add(AddressFamily::ipv6, ll, 64u);
    check(l.primary().address == ll, "link-local alone is primary");
    l.add(AddressFamily::ipv6, g1, 64u, 600u, 0u, true);
    check(l.primary().address == g1,
          "rule 2: a routable address, even deprecated, beats link-local");

    AddressList m{};
    m.add(AddressFamily::ipv6, g1, 64u, 600u, 0u, true);
    m.add(AddressFamily::ipv6, g2, 64u);
    check(m.primary().address == g2, "rule 3: preferred beats deprecated at the same scope");

    AddressList n{};
    n.add(AddressFamily::ipv6, g1, 64u);
    n.add(AddressFamily::ipv6, g2, 64u);
    check(n.primary().address == g1, "a tie keeps list order");
    check(!n.add(AddressFamily::ipv6, ll, 64u) && n.size() == kMaxFamilyAddresses,
          "a full list refuses another address");
    check(n[0].routable() && !n[0].link_local(), "add() derives the routable bit");
}

int main() {
    AddressSnapshot a{};
    AddressSnapshot b{};
    check(a == b, "default snapshots compare equal");

    b.v4.addresses.add(AddressFamily::ipv4, {192u, 168u, 1u, 4u}, 24u);
    check(a != b, "a v4 address change is visible");
    a = b;
    check(a == b, "copy compares equal again");

    b.v4.addresses.clear();
    b.v4.addresses.add(AddressFamily::ipv4, {192u, 168u, 1u, 5u}, 24u);
    check(a != b, "a v4 octet change is visible");
    a = b;

    b.v6.addresses.add(AddressFamily::ipv6,
                       {0xFEu, 0x80u, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1u}, 64u);
    check(a != b, "a second-family address is visible");
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

    tc_net_6724_001();

    // classifiers
    check(IsIpv4LinkLocal({169u, 254u, 3u, 9u}), "169.254.x.y is link-local");
    check(!IsIpv4LinkLocal({169u, 253u, 3u, 9u}), "169.253.x.y is not");
    check(IsIpv6LinkLocal({0xFEu, 0x80u}), "fe80:: is link-local");
    check(IsIpv6LinkLocal({0xFEu, 0xB0u}), "feb0:: is still fe80::/10");
    check(!IsIpv6LinkLocal({0x20u, 0x01u}), "2001:: is not link-local");

    return test_summary();
}
