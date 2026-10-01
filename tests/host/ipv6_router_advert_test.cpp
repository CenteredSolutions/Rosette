/**
 * @file ipv6_router_advert_test.cpp
 * @brief `parse_router_advert()` against captured and constructed advertisements.
 *
 * TC-NET-RA-001 captured RA, prefix option first (the layout the W6100's own
 * parser misreads); -002 the same options with the source link-layer address
 * first; -003 non-autonomous and non-/64 prefixes are not chosen; -004
 * malformed messages rejected; -005 non-link-local source rejected; -006
 * router lifetime 0 still yields the prefix.
 */

#include "test_util.hpp"

#include "rosette/net/ipv6_router_advert.hpp"

#include <array>
#include <cstdint>
#include <vector>

namespace {

using rosette::net::RouterAdvert;
using rosette::net::parse_router_advert;

const std::array<uint8_t, 16> kRouter{0xFE, 0x80, 0, 0, 0, 0, 0, 0,
                                      0x9C, 0x05, 0xD6, 0xFF, 0xFE, 0xBB, 0xC9, 0x0F};
const std::array<uint8_t, 16> kPrefix{0x26, 0x03, 0x80, 0x80, 0xF8, 0x06, 0xF4, 0x50};

// Captured on the test LAN 2026-09-26: header, prefix info, MTU, source
// link-layer address, RDNSS, in that order.
const std::vector<uint8_t> kHeader{0x86, 0x00, 0x83, 0xcd, 0x40, 0x08, 0x07, 0x08,
                                   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const std::vector<uint8_t> kPrefixOpt{0x03, 0x04, 0x40, 0xc0, 0x00, 0x01, 0x51, 0x80,
                                      0x00, 0x01, 0x51, 0x80, 0x00, 0x00, 0x00, 0x00,
                                      0x26, 0x03, 0x80, 0x80, 0xf8, 0x06, 0xf4, 0x50,
                                      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const std::vector<uint8_t> kMtuOpt{0x05, 0x01, 0x00, 0x00, 0x00, 0x00, 0x05, 0xdc};
const std::vector<uint8_t> kSllaOpt{0x01, 0x01, 0x9e, 0x05, 0xd6, 0xbb, 0xc9, 0x0f};
const std::vector<uint8_t> kRdnssOpt{0x19, 0x03, 0x00, 0x00, 0x00, 0x00, 0x07, 0x08,
                                     0x26, 0x03, 0x80, 0x80, 0xf8, 0x06, 0xf4, 0x50,
                                     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01};

std::vector<uint8_t> concat(std::initializer_list<std::vector<uint8_t>> parts)
{
    std::vector<uint8_t> out;
    for (const auto& p : parts) out.insert(out.end(), p.begin(), p.end());
    return out;
}

bool parse(const std::vector<uint8_t>& m, RouterAdvert& ra, const std::array<uint8_t, 16>& src = kRouter)
{
    return parse_router_advert(m.data(), m.size(), src, ra);
}

}  // namespace

int main()
{
    // TC-NET-RA-001
    {
        RouterAdvert ra{};
        const bool ok = parse(concat({kHeader, kPrefixOpt, kMtuOpt, kSllaOpt, kRdnssOpt}), ra);
        check(ok && ra.has_prefix, "captured RA (prefix option first): prefix found");
        check(ra.prefix == kPrefix && ra.prefix_length == 64u && ra.on_link,
              "captured RA: 2603:8080:f806:f450::/64, on-link");
        check(ra.valid_lifetime_s == 86400u && ra.preferred_lifetime_s == 86400u,
              "captured RA: valid and preferred lifetimes 86400 s");
        check(ra.router == kRouter && ra.router_lifetime_s == 1800u && ra.mtu == 1500u,
              "captured RA: router, router lifetime 1800 s, MTU 1500");
    }

    // TC-NET-RA-002
    {
        RouterAdvert ra{};
        const bool ok = parse(concat({kHeader, kSllaOpt, kRdnssOpt, kMtuOpt, kPrefixOpt}), ra);
        check(ok && ra.has_prefix && ra.prefix == kPrefix && ra.valid_lifetime_s == 86400u,
              "same options, source link-layer address first: same result");
    }

    // TC-NET-RA-003
    {
        auto not_autonomous = kPrefixOpt;
        not_autonomous[3] = 0x80u;
        RouterAdvert ra{};
        check(parse(concat({kHeader, not_autonomous}), ra) && !ra.has_prefix,
              "on-link-only prefix: valid RA, no prefix chosen");
        auto slash56 = kPrefixOpt;
        slash56[2] = 56u;
        check(parse(concat({kHeader, slash56, kPrefixOpt}), ra) && ra.has_prefix && ra.prefix_length == 64u,
              "/56 skipped; the following autonomous /64 chosen");
        auto preferred_above_valid = kPrefixOpt;
        preferred_above_valid[11] = 0x81u;
        check(parse(concat({kHeader, preferred_above_valid}), ra) && !ra.has_prefix,
              "preferred lifetime above valid lifetime: prefix ignored (RFC 4862 5.5.3 c)");
    }

    // TC-NET-RA-004
    {
        RouterAdvert ra{};
        auto zero_len = concat({kHeader, kMtuOpt});
        zero_len[17] = 0u;
        check(!parse(zero_len, ra), "zero-length option rejected");
        auto overrun = concat({kHeader, kMtuOpt});
        overrun[17] = 2u;
        check(!parse(overrun, ra), "option running past the message rejected");
        check(!parse(std::vector<uint8_t>(kHeader.begin(), kHeader.begin() + 12), ra), "short message rejected");
        auto wrong_type = kHeader;
        wrong_type[0] = 133u;
        check(!parse(wrong_type, ra), "router solicitation is not an advertisement");
        auto wrong_code = kHeader;
        wrong_code[1] = 1u;
        check(!parse(wrong_code, ra), "nonzero code rejected");
    }

    // TC-NET-RA-005
    {
        RouterAdvert ra{};
        check(!parse(concat({kHeader, kPrefixOpt}), ra, kPrefix),
              "global source address rejected (RFC 4861 6.1.2)");
    }

    // TC-NET-RA-006
    {
        auto no_router = kHeader;
        no_router[6] = 0u;
        no_router[7] = 0u;
        RouterAdvert ra{};
        check(parse(concat({no_router, kPrefixOpt}), ra) && ra.has_prefix && ra.router_lifetime_s == 0u,
              "router lifetime 0: prefix still usable, not a default router");
    }

    return test_summary();
}
